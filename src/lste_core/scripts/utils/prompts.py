#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Prompt helper utilities extracted from LSTE/Data_exchange/8B-05B.py
Lightweight: 不依赖 GroundingDINO / cv2，专供 lste_prompt_node 使用。
"""

import json
import os
from pathlib import Path

import httpx
import openai


def _get_workspace_root() -> Path:
    """Auto-detect workspace root from LSTE_WS env var or this file's location."""
    env = os.environ.get("LSTE_WS")
    if env:
        return Path(env)
    # This file is at WS/src/lste_core/scripts/utils/prompts.py
    return Path(__file__).resolve().parents[4]


PROJECT_ROOT = _get_workspace_root() / "model"
DEFAULT_VLLM_MODEL_PATH = PROJECT_ROOT / "MiniCPM" / "OpenBMB" / "MiniCPM4-0___5B"
DEFAULT_VLLM_BASE_URL = "http://localhost:8000/v1"

COLOR_KEYWORDS = {
    "red",
    "orange",
    "yellow",
    "green",
    "blue",
    "purple",
    "pink",
    "brown",
    "black",
    "white",
    "gray",
    "grey",
    "cyan",
    "magenta",
    "beige",
    "gold",
    "silver",
}


def _extract_env_type_strings(env_type_prior):
    env_types = []
    for item in env_type_prior or []:
        if isinstance(item, dict):
            type_name = item.get("type")
            if type_name:
                env_types.append(str(type_name))
        else:
            env_types.append(str(item))
    return env_types


def extract_color_terms(attributes):
    colors = []
    for attr in attributes or []:
        attr_lower = str(attr).lower()
        for color in sorted(COLOR_KEYWORDS):
            if color in attr_lower and color not in colors:
                colors.append(color)
    return colors


def build_llm_prompt_from_task(task_parsed: dict) -> str:
    target = task_parsed.get("target", {})
    env = task_parsed.get("env", {})
    obj_related = task_parsed.get("obj_related", {})

    target_name = target.get("name", "")
    target_attrs = target.get("attributes", []) or []
    allowed_colors = extract_color_terms(target_attrs)
    allowed_colors_str = ", ".join(allowed_colors) if allowed_colors else "none"

    related_structures = env.get("related_structures", [])
    key_objects = obj_related.get("key_objects", [])
    env_type_prior = _extract_env_type_strings(env.get("env_type_prior", []))

    # Keep the task JSON order stable. A set makes the LLM prompt (and its
    # persistent cache key) change across Python processes.
    env_objects = list(dict.fromkeys(related_structures + key_objects + env_type_prior))
    env_objects = [obj for obj in env_objects if obj.lower() not in target_name.lower()]
    env_objects_str = ", ".join(env_objects[:8]) if env_objects else "desk, chair, monitor, keyboard"

    prompt = f"""
Task: Generate GroundingDINO detection prompts.

=== TARGET TO DETECT ===
Object Name: {target_name}
Attributes: {', '.join(target_attrs) if target_attrs else 'none'}

=== ENVIRONMENT OBJECTS (select from these) ===
{env_objects_str}

=== YOUR TASK ===

1. Generate "prompt_A":
   - Describe the TARGET: {target_name}
   - Use ONLY ONE most important attribute from: {', '.join(target_attrs[:3]) if target_attrs else 'none'}
   - If you mention a color, you MUST choose from: {allowed_colors_str}. Do not invent new colors.
   - Keep it very short (3-5 words)
   - Format: [one adjective] + object name
   - Example: "red chair" or "small orange chair" (max 3 words)

2. Generate "prompt_B":
   - Select 4-6 items ONLY from: {env_objects_str}
   - Simple nouns (1-3 words each)
   - DO NOT include: {target_name}

=== OUTPUT FORMAT ===
Respond with ONLY this JSON (no markdown, no explanations):
{{
  "prompt_A": "<1-2 adjectives + {target_name}>",
  "prompt_B": ["<item1>", "<item2>", "<item3>", "<item4>"]
}}

IMPORTANT:
- prompt_A: Use MAXIMUM 1-2 adjectives, keep it under 5 words total
- prompt_A MUST contain: {target_name}
- Allowed color adjectives: {allowed_colors_str} (if 'none', do NOT mention any color)
- prompt_B items MUST be from: {env_objects_str}
""".strip()
    return prompt


def parse_llm_json_output(text: str) -> dict:
    if "```json" in text:
        start = text.find("```json") + 7
        end = text.find("```", start)
        if end != -1:
            json_str = text[start:end].strip()
            try:
                return json.loads(json_str)
            except json.JSONDecodeError:
                pass

    if "```python" in text:
        start = text.find("```python") + 9
        end = text.find("```", start)
        if end != -1:
            json_str = text[start:end].strip()
            try:
                return json.loads(json_str)
            except json.JSONDecodeError:
                pass

    if "```" in text:
        start = text.find("```") + 3
        end = text.find("```", start)
        if end != -1:
            json_str = text[start:end].strip()
            try:
                return json.loads(json_str)
            except json.JSONDecodeError:
                pass

    first = text.find("{")
    last = text.rfind("}")
    if first != -1 and last != -1 and last > first:
        json_str = text[first : last + 1]
        try:
            return json.loads(json_str)
        except json.JSONDecodeError:
            pass

    return {"prompt_A": "", "prompt_B": []}


def call_local_llm(prompt: str, base_url: str, model_name: str, max_new_tokens: int = 256) -> str:
    client = openai.Client(
        base_url=base_url,
        api_key="EMPTY",
        http_client=httpx.Client(trust_env=False),
    )
    response = client.chat.completions.create(
        model=model_name,
        messages=[
            {
                "role": "system",
                "content": "You are generating prompts for GroundingDINO. Treat each request as isolated. Never reference or rely on any previous conversation.",
            },
            {"role": "user", "content": prompt},
        ],
        temperature=0.4,
        max_tokens=max_new_tokens,
        top_p=0.8,
        extra_body=dict(add_special_tokens=True),
    )
    text = response.choices[0].message.content
    return text.strip()


def build_prompt_a_from_task(task_parsed: dict) -> str:
    target = task_parsed.get("target", {})
    name = target.get("name", "object")
    attrs = target.get("attributes", []) or []
    name_lower = name.lower()
    allowed_colors = extract_color_terms(attrs)

    first_attr = ""
    if attrs:
        first_attr = str(attrs[0]).strip()
        first_attr_lower = first_attr.lower()
        if allowed_colors and any(c in name_lower for c in allowed_colors):
            for c in allowed_colors:
                if c in first_attr_lower:
                    first_attr = ""
                    break
        words = first_attr.split()
        if len(words) > 3:
            first_attr = " ".join(words[:3])

    prompt_a = f"{first_attr} {name}".strip()
    prompt_a = " ".join(prompt_a.split())
    return prompt_a


def fix_prompt_a(prompt_a_raw, task_parsed: dict) -> str:
    target = task_parsed.get("target", {})
    t_name = target.get("name", "")
    attrs = target.get("attributes", []) or []
    allowed_colors = extract_color_terms(attrs)
    t_lower = t_name.lower()
    if isinstance(prompt_a_raw, (list, tuple)):
        prompt_a_raw = " ".join(str(x) for x in prompt_a_raw if x)
    elif not isinstance(prompt_a_raw, str):
        prompt_a_raw = str(prompt_a_raw)
    a = (prompt_a_raw or "").strip()
    bad_patterns = [
        "short detailed sentence",
        "short phrase for target object",
        "one short detailed sentence",
        "a short detailed sentence",
    ]
    lower_a = a.lower()
    bad = False
    if not a:
        bad = True
    if t_lower and t_lower not in lower_a:
        bad = True
    for pat in bad_patterns:
        if pat in lower_a:
            bad = True
            break
    if not bad and allowed_colors:
        for color in COLOR_KEYWORDS:
            if color in lower_a and color not in allowed_colors:
                bad = True
                break
    elif not bad and not allowed_colors:
        for color in COLOR_KEYWORDS:
            if color in lower_a:
                bad = True
                break
    if bad:
        return build_prompt_a_from_task(task_parsed)
    return a


def clean_prompt_b_list(prompt_b_raw, target_name: str, drop_colors: bool = True):
    cleaned = []
    t_lower = str(target_name).lower()
    for item in prompt_b_raw or []:
        s = str(item).strip()
        if not s:
            continue
        if t_lower and t_lower in s.lower():
            continue
        if drop_colors:
            if any(c in s.lower() for c in COLOR_KEYWORDS):
                continue
        cleaned.append(s)
    return cleaned


def build_default_prompt_b(task_parsed: dict, target_name: str):
    env = task_parsed.get("env", {}) or {}
    obj_related = task_parsed.get("obj_related", {}) or {}
    env_objs = list(env.get("related_structures", []) or [])
    env_type_prior = _extract_env_type_strings(env.get("env_type_prior", []) or [])
    key_objects = list(obj_related.get("key_objects", []) or [])
    merged = env_objs + env_type_prior + key_objects
    out = []
    t_lower = str(target_name).lower()
    for item in merged:
        if not item:
            continue
        s = str(item).strip()
        if not s:
            continue
        if t_lower and t_lower in s.lower():
            continue
        out.append(s)
    if not out:
        out = ["door", "table", "chair", "monitor"]
    return out
