#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
lste_prompt_node
- 订阅 /lste/task
- 调用 MiniCPM (通过 vLLM API) 生成 prompt_A / prompt_B
- 发布 /lste/prompts (latched)
"""

import hashlib
import json
import os
import subprocess
import tempfile
import threading
import time
import urllib.request
from pathlib import Path

import rospy
from lste_msgs.msg import LstePrompts, LsteTask

from utils import prompts as prompt_utils


PROMPT_CACHE_VERSION = 1


def prompt_cache_key(task_parsed: dict, llm_prompt: str, model_name: str) -> str:
    """Return a stable key that changes with task, template, model, or cache format."""
    model_id = Path(str(model_name)).name
    payload = {
        "version": PROMPT_CACHE_VERSION,
        "model": model_id,
        "task": task_parsed,
        "llm_prompt": llm_prompt,
    }
    canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def load_prompt_cache(cache_path: Path, expected_key: str):
    try:
        with cache_path.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, ValueError, TypeError):
        return None

    if data.get("version") != PROMPT_CACHE_VERSION or data.get("cache_key") != expected_key:
        return None
    prompt_a = data.get("prompt_a")
    prompt_b_terms = data.get("prompt_b_terms")
    if not isinstance(prompt_a, str) or not prompt_a.strip():
        return None
    if not isinstance(prompt_b_terms, list) or not prompt_b_terms:
        return None
    if not all(isinstance(term, str) and term.strip() for term in prompt_b_terms):
        return None
    return data


def save_prompt_cache(cache_path: Path, data: dict):
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=cache_path.name + ".", suffix=".tmp", dir=str(cache_path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_name, cache_path)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


def task_to_task_parsed(msg: LsteTask) -> dict:
    """把 LsteTask 转成 8B-05B.py 期待的 task_parsed 结构。"""
    if msg.raw_json:
        try:
            data = json.loads(msg.raw_json)
            return data.get("task_parsed", data)
        except Exception as e:
            rospy.logwarn("Failed to parse raw_json in LsteTask: %s", e)

    # fallback: 用字段拼一个最小结构
    return {
        "target": {
            "name": msg.target_name,
            "attributes": list(msg.target_attributes),
        },
        "env": {
            "env_type_prior": list(msg.env_type_prior),
            "related_structures": list(msg.env_related_structures),
        },
        "obj_related": {
            "key_objects": list(msg.obj_key_objects),
            "negative_clues": list(msg.obj_negative_clues),
        },
        "target_ctx": {
            "left": msg.ctx_left,
            "right": msg.ctx_right,
        },
    }


class PromptNode:
    def __init__(self):
        rospy.init_node("lste_prompt_node")
        self.pub = rospy.Publisher("/lste/prompts", LstePrompts, queue_size=1, latch=True)

        # 允许通过参数覆盖 vLLM 地址和模型
        self.vllm_base_url = rospy.get_param("~vllm_base_url", str(prompt_utils.DEFAULT_VLLM_BASE_URL))
        self.vllm_model_name = rospy.get_param("~vllm_model_name", str(prompt_utils.DEFAULT_VLLM_MODEL_PATH))

        # 可选：生成后执行一个停止 vLLM 的命令（例如 pkill 或 tmux 控制命令）
        self.vllm_stop_command = rospy.get_param("~vllm_stop_command", "")
        workspace = Path(os.environ.get("LSTE_WS", Path(__file__).resolve().parents[3]))
        self.cache_enabled = bool(rospy.get_param("~cache_enabled", True))
        self.cache_dir = Path(rospy.get_param("~cache_dir", str(workspace / "runtime" / "prompt_cache")))
        self.vllm_start_timeout = float(rospy.get_param("~vllm_start_timeout", 240.0))
        self.republish_interval = float(rospy.get_param("~republish_interval", 5.0))
        self.latest_prompts = None
        self.latest_cache_key = ""
        self.processing_lock = threading.Lock()

        # The launcher uses this decision to avoid loading MiniCPM on a cache hit.
        rospy.set_param("~needs_vllm", "pending")

        rospy.loginfo("lste_prompt_node started. Waiting for /lste/task ...")
        self.sub = rospy.Subscriber("/lste/task", LsteTask, self.on_task, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.republish_interval), self.on_timer)

    def submit_vllm_stop(self):
        if not self.vllm_stop_command:
            return
        rospy.loginfo("Stopping external vLLM via command: %s", self.vllm_stop_command)
        try:
            subprocess.Popen(["bash", "-lc", self.vllm_stop_command])
        except subprocess.CalledProcessError as exc:
            rospy.logwarn("Stop command failed with return code %s", exc.returncode)
        except Exception as exc:
            rospy.logwarn("Failed to launch stop command: %s", exc)

    def wait_for_vllm(self) -> bool:
        probe_url = self.vllm_base_url.rstrip("/")
        if probe_url.endswith("/v1"):
            probe_url += "/models"
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        deadline = time.monotonic() + self.vllm_start_timeout
        rospy.loginfo("Waiting for MiniCPM vLLM at %s ...", probe_url)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                with opener.open(probe_url, timeout=2.0):
                    rospy.loginfo("MiniCPM vLLM is ready.")
                    return True
            except Exception:
                time.sleep(1.0)
        return False

    def publish_cached(self, msg: LsteTask, cached: dict, cache_key: str):
        out = LstePrompts()
        out.task_id = msg.task_id
        out.prompt_a = cached["prompt_a"]
        out.prompt_b_terms = list(cached["prompt_b_terms"])
        out.raw_llm_output = cached.get("raw_llm_output", "")
        self.pub.publish(out)
        self.latest_prompts = out
        self.latest_cache_key = cache_key
        return out

    def build_cache_record(self, task_parsed: dict, llm_json: dict, cache_key: str) -> dict:
        prompt_a_raw = llm_json.get("prompt_A", "")
        prompt_a = prompt_utils.fix_prompt_a(prompt_a_raw, task_parsed)

        prompt_b_raw = llm_json.get("prompt_B", [])
        target_name = task_parsed.get("target", {}).get("name", "")
        prompt_b_list = prompt_utils.clean_prompt_b_list(prompt_b_raw, target_name)
        if not prompt_b_list:
            prompt_b_list = prompt_utils.build_default_prompt_b(task_parsed, target_name)

        target_ctx = task_parsed.get("target_ctx") or {}
        ctx_terms_raw = [str(target_ctx[key]) for key in ("left", "right") if target_ctx.get(key)]
        if ctx_terms_raw:
            ctx_terms = prompt_utils.clean_prompt_b_list(ctx_terms_raw, target_name, drop_colors=False)
            existing = {term.lower() for term in prompt_b_list}
            for term in ctx_terms:
                if term.lower() not in existing:
                    prompt_b_list.append(term)
                    existing.add(term.lower())

        negative_clues = (task_parsed.get("obj_related") or {}).get("negative_clues", []) or []
        if negative_clues:
            neg_terms = prompt_utils.clean_prompt_b_list(negative_clues, target_name, drop_colors=False)
            existing = {term.lower() for term in prompt_b_list}
            for term in neg_terms:
                if term.lower() not in existing:
                    prompt_b_list.append(term)
                    existing.add(term.lower())

        try:
            raw_llm_output = json.dumps(llm_json, ensure_ascii=False)
        except Exception:
            raw_llm_output = str(llm_json)

        return {
            "version": PROMPT_CACHE_VERSION,
            "cache_key": cache_key,
            "model": Path(str(self.vllm_model_name)).name,
            "prompt_a": prompt_a,
            "prompt_b_terms": prompt_b_list,
            "raw_llm_output": raw_llm_output,
        }

    def on_task(self, msg: LsteTask):
        task_parsed = task_to_task_parsed(msg)
        llm_prompt = prompt_utils.build_llm_prompt_from_task(task_parsed)
        cache_key = prompt_cache_key(task_parsed, llm_prompt, self.vllm_model_name)
        cache_path = self.cache_dir / (cache_key + ".json")

        if not self.processing_lock.acquire(blocking=False):
            rospy.logwarn("Ignoring task_id=%s while prompt generation is already running", msg.task_id)
            return

        try:
            if (
                self.latest_prompts is not None
                and cache_key == self.latest_cache_key
                and self.latest_prompts.task_id == msg.task_id
            ):
                self.pub.publish(self.latest_prompts)
                return

            cached = load_prompt_cache(cache_path, cache_key) if self.cache_enabled else None
            if cached is not None:
                rospy.set_param("~needs_vllm", False)
                self.publish_cached(msg, cached, cache_key)
                rospy.loginfo(
                    "Prompt cache hit for task_id=%s (%s); MiniCPM is not required.",
                    msg.task_id,
                    cache_path,
                )
                return

            rospy.set_param("~needs_vllm", True)
            rospy.loginfo("Prompt cache miss for task_id=%s; requesting MiniCPM.", msg.task_id)
            if not self.wait_for_vllm():
                rospy.logerr("MiniCPM vLLM did not become ready within %.1f seconds", self.vllm_start_timeout)
                rospy.signal_shutdown("vllm_start_timeout")
                return

            try:
                llm_output = prompt_utils.call_local_llm(llm_prompt, self.vllm_base_url, self.vllm_model_name)
                llm_json = prompt_utils.parse_llm_json_output(llm_output)
            except Exception as e:
                rospy.logerr("Failed to call MiniCPM or parse its output: %s", e)
                rospy.signal_shutdown("prompt_failed")
                return

            cached = self.build_cache_record(task_parsed, llm_json, cache_key)
            if self.cache_enabled:
                try:
                    save_prompt_cache(cache_path, cached)
                    rospy.loginfo("Saved prompt cache: %s", cache_path)
                except Exception as exc:
                    rospy.logwarn("Failed to save prompt cache %s: %s", cache_path, exc)

            self.publish_cached(msg, cached, cache_key)
            rospy.set_param("~needs_vllm", False)
            rospy.loginfo(
                "Published /lste/prompts for task_id=%s. Will continue re-publishing until task completes.",
                msg.task_id,
            )
            self.submit_vllm_stop()
        finally:
            self.processing_lock.release()

    def on_timer(self, _event):
        if self.latest_prompts is not None:
            self.pub.publish(self.latest_prompts)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    PromptNode().spin()
