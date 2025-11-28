#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
1. Read after_vlm_json (task_parsed from 8B VLM)
2. Send it to a small LLM (0.5B) to generate prompts for GroundingDINO
3. Use the generated prompts to run GroundingDINO on a given image
"""

import json
import argparse
from pathlib import Path
import cv2

import httpx

from groundingdino.util.inference import load_model, load_image, predict, annotate
import torch
import openai

# vLLM服务配置
_vllm_client = None
_vllm_base_url = "http://localhost:8000/v1"
_vllm_model_name = "/home/zrz/Desktop/LSTE/MiniCPM/OpenBMB/MiniCPM4-0___5B"


# ================= 0. vLLM & LLM 调用 =================

def init_vllm_client():
    """
    初始化vLLM客户端
    """
    global _vllm_client
    if _vllm_client is None:
        print(f"[LLM] 连接vLLM服务: {_vllm_base_url}")
        _vllm_client = openai.Client(
            base_url=_vllm_base_url,
            api_key="EMPTY",
            http_client=httpx.Client(trust_env=False)
        )
        try:
            _ = _vllm_client.chat.completions.create(
                model=_vllm_model_name,
                messages=[{"role": "user", "content": "hello"}],
                max_tokens=5,
                extra_body=dict(add_special_tokens=True),
            )
            print("[LLM] vLLM服务连接成功")
        except Exception as e:
            print(f"[LLM] 警告: vLLM服务连接失败: {e}")
            print("[LLM] 请确保vLLM服务已启动")
            print("[LLM] 启动命令: VLLM_USE_V1=0 vllm serve /home/zrz/Desktop/LSTE/MiniCPM/OpenBMB/MiniCPM4-0___5B --trust-remote-code --max-model-len 2048 --gpu-memory-utilization 0.6 --enforce-eager")
            raise
    return _vllm_client


def call_local_llm(prompt: str, max_new_tokens: int = 256) -> str:
    """
    通过vLLM调用MiniCPM 0.5B生成响应
    """
    client = init_vllm_client()
    try:
        response = client.chat.completions.create(
            model=_vllm_model_name,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.4,       # 稍微稳一点
            max_tokens=max_new_tokens,
            top_p=0.8,
            extra_body=dict(add_special_tokens=True),
        )
        text = response.choices[0].message.content
        return text.strip()
    except Exception as e:
        print(f"[LLM] 调用失败: {e}")
        raise


# ================= 1. 构造给 0.5B 的 Prompt =================

def build_llm_prompt_from_task(task_parsed: dict) -> str:
    """
    构造给 0.5B 的英文说明：
    - A：详细描述 target（颜色/形状/安装位置等）
    - B：环境结构名词，不包含 target 本身
    """
    target = task_parsed.get("target", {})
    env = task_parsed.get("env", {})
    obj_related = task_parsed.get("obj_related", {})

    target_name = target.get("name", "")
    target_attrs = target.get("attributes", [])
    
    # 提取环境相关物体
    related_structures = env.get("related_structures", [])
    key_objects = obj_related.get("key_objects", [])
    
    # 合并并去重
    env_objects = list(set(related_structures + key_objects))
    # 过滤掉和目标相同的
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
- prompt_B items MUST be from: {env_objects_str}
""".strip()

    return prompt


# ================= 2. 解析 LLM 输出 + 兜底策略 =================

def parse_llm_json_output(text: str) -> dict:
    """
    从 LLM 输出中提取 JSON；尽量鲁棒。
    """
    # 代码块中的 JSON
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

    # 直接找大括号
    first = text.find("{")
    last = text.rfind("}")
    if first != -1 and last != -1 and last > first:
        json_str = text[first:last + 1]
        try:
            return json.loads(json_str)
        except json.JSONDecodeError as e:
            print(f"[ERROR] JSON解析失败: {e}")
            print(f"[ERROR] 提取的字符串: {json_str[:200]}...")

    print(f"[WARNING] 无法解析LLM输出，使用默认提示词")
    print(f"[WARNING] LLM原始输出: {text[:500]}...")
    return {
        "prompt_A": "",
        "prompt_B": []
    }


def build_prompt_a_from_task(task_parsed: dict) -> str:
    """
    当 LLM 给的 prompt_A 不靠谱时，直接从 task_parsed 规则生成一个 A。
    限制：最多使用一个形容词
    """
    target = task_parsed.get("target", {})
    name = target.get("name", "object")
    attrs = target.get("attributes", []) or []

    # 只取第一个属性作为形容词
    if attrs:
        # 取第一个属性，去掉过于复杂的描述
        first_attr = str(attrs[0]).strip()
        # 如果属性太长（超过3个词），只取前面的词
        attr_words = first_attr.split()
        if len(attr_words) > 2:
            first_attr = " ".join(attr_words[:2])
        return f"{first_attr} {name}".strip()
    else:
        return name


def clean_prompt_b_list(prompt_B_list, target_name: str):
    """
    将 LLM 生成的 prompt_B 做简单清洗：
    - 去掉常见介词和冠词：near/on/in/at/by/.../the/a/an/of 等
    - 不允许包含 target_name
    - 每个短语最多保留前 3 个有效单词
    """
    if not isinstance(prompt_B_list, list):
        return []

    stopwords = {
        "near", "on", "in", "at", "by", "along", "around",
        "beside", "between", "inside", "outside", "over", "under",
        "the", "a", "an", "of", "to", "from", "with"
    }
    target_name = (target_name or "").lower().strip()

    cleaned = []
    for phrase in prompt_B_list:
        if not isinstance(phrase, str):
            continue
        lower = phrase.lower()
        # 删掉包含目标名称的条目
        if target_name and target_name in lower:
            continue

        words = [w.strip().lower() for w in phrase.split() if w.strip()]
        # 去停用词
        words = [w for w in words if w not in stopwords]
        if not words:
            continue
        # 最多保留三个词
        trimmed = " ".join(words[:3])
        if not trimmed:
            continue
        cleaned.append(trimmed)

    # 去重，保持顺序
    seen = set()
    final_list = []
    for p in cleaned:
        if p not in seen:
            seen.add(p)
            final_list.append(p)

    return final_list


def build_default_prompt_b(task_parsed: dict, target_name: str):
    """
    当 LLM 给的 B 不靠谱/为空时，从 env/obj_related 里自己凑一份 B。
    """
    env = task_parsed.get("env", {})
    obj_related = task_parsed.get("obj_related", {})

    related_structures = env.get("related_structures", []) or []
    env_types = env.get("env_type_prior", []) or []
    key_objects = obj_related.get("key_objects", []) or []

    candidates = []
    candidates.extend(related_structures)
    candidates.extend(env_types)
    candidates.extend(key_objects)

    # 清洗 + 去掉 target
    b_list = clean_prompt_b_list(candidates, target_name)

    # 限制长度
    if len(b_list) > 6:
        b_list = b_list[:6]

    # 最坏兜底
    if not b_list:
        # 给一些通用的结构物
        b_list = ["door", "wall", "corridor", "exit sign"]

    return b_list


def fix_prompt_a(prompt_a_raw: str, task_parsed: dict) -> str:
    """
    确保 prompt_A 里真的有 target 信息，而不是占位文字。
    不符合要求时，用 build_prompt_a_from_task 替换。
    """
    target_name = (task_parsed.get("target", {}) or {}).get("name", "")
    t_lower = target_name.lower() if target_name else ""
    a = (prompt_a_raw or "").strip()

    # 一些明显错误的占位关键词
    bad_patterns = [
        "short detailed sentence describing the target object",
        "short phrase for target object",
        "one short detailed sentence",
        "a short detailed sentence",
    ]

    lower_a = a.lower()
    bad = False
    if not a:
        bad = True
    if t_lower and t_lower not in lower_a:
        # prompt_A 里都不含目标名，那基本是错的
        bad = True
    for pat in bad_patterns:
        if pat in lower_a:
            bad = True
            break

    if bad:
        print("[PROMPT_A] LLM给的A不靠谱，改用规则生成。原A:", a)
        return build_prompt_a_from_task(task_parsed)
    else:
        return a


# ================= 3. GroundingDINO 调用 =================

def run_grounding_dino_on_image(
    image_path: str,
    prompt_A: str,
    prompt_B_list: list,
    config_path: str,
    weights_path: str,
    box_threshold: float = 0.35,
    text_threshold: float = 0.25,
    output_path=None,
):
    """
    Run GroundingDINO on a single image with A/B prompts.
    """
    model = load_model(config_path, weights_path)
    image_source, image = load_image(image_path)

    # 拼 caption
    prompt_B = " . ".join(prompt_B_list) if prompt_B_list else ""
    caption = prompt_A
    if prompt_B:
        caption = caption + " . " + prompt_B

    print(f"[DINO] Using caption: {caption}")

    # 尝试检测，处理空结果的情况
    try:
        boxes, logits, phrases = predict(
            model=model,
            image=image,
            caption=caption,
            box_threshold=box_threshold,
            text_threshold=text_threshold,
        )
    except RuntimeError as e:
        if "no elements" in str(e):
            print(f"[DINO] 没有检测到任何目标")
            print(f"[DINO] 阈值设置: box_threshold={box_threshold}, text_threshold={text_threshold}")
            print(f"[DINO] 建议: 尝试降低阈值或调整提示词")
            # 返回空结果
            import torch
            boxes = torch.tensor([])
            logits = torch.tensor([])
            phrases = []
        else:
            raise

    print(f"[DINO] Detected {len(boxes)} boxes.")
    for i, (p, s) in enumerate(zip(phrases, logits)):
        print(f"  #{i}: phrase='{p}', score={float(s):.3f}")

    # 保存图片
    if output_path is not None:
        if len(boxes) > 0:
            # 有检测结果，保存标注图片
            annotated_frame = annotate(
                image_source=image_source,
                boxes=boxes,
                logits=logits,
                phrases=phrases
            )
            cv2.imwrite(output_path, annotated_frame)
            print(f"[DINO] 标注图片已保存至: {output_path}")
        else:
            # 没有检测结果，保存原图
            cv2.imwrite(output_path, image_source)
            print(f"[DINO] 未检测到目标，原图已保存至: {output_path}")

    return boxes, logits, phrases


# ================= 4. 主流程 =================

def main():
    parser = argparse.ArgumentParser(
        description="Use after_vlm_json + 0.5B LLM to build GroundingDINO prompts and run detection."
    )
    parser.add_argument(
        "--json_path", type=str, required=True,
        help="Path to after_vlm_json file (the VLM 8B parsed JSON)."
    )
    parser.add_argument(
        "--image_path", type=str, required=True,
        help="Path to input image for GroundingDINO."
    )
    parser.add_argument(
        "--output_image", type=str, default=None,
        help="Path to save annotated image (optional)."
    )
    parser.add_argument(
        "--config_path", type=str,
        default="/home/zrz/Desktop/LSTE/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py",
        help="GroundingDINO config .py path."
    )
    parser.add_argument(
        "--weights_path", type=str,
        default="/home/zrz/Desktop/LSTE/GroundingDINO/weights/groundingdino_swint_ogc.pth",
        help="GroundingDINO weights .pth path."
    )
    parser.add_argument(
        "--vllm_url", type=str,
        default="http://localhost:8000/v1",
        help="vLLM服务地址"
    )
    parser.add_argument(
        "--vllm_model", type=str,
        default="/home/zrz/Desktop/LSTE/MiniCPM/OpenBMB/MiniCPM4-0___5B",
        help="vLLM服务中的模型名称/路径"
    )
    args = parser.parse_args()

    json_path = Path(args.json_path)
    if not json_path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")

    # 1) 读取 after_vlm_json
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    task_parsed = data.get("task_parsed", data)

    # 提前拿 target_name，后面 A/B 都要用
    target_name = (task_parsed.get("target", {}) or {}).get("name", "")

    # 2) 构造喂给 0.5B LLM 的 prompt
    llm_prompt = build_llm_prompt_from_task(task_parsed)
    print("============ [LLM PROMPT] ============")
    print(llm_prompt)
    print("======================================")

    # 3) 设置vLLM参数并调用 0.5B LLM
    global _vllm_base_url, _vllm_model_name
    _vllm_base_url = args.vllm_url
    _vllm_model_name = args.vllm_model
    
    llm_output = call_local_llm(llm_prompt)
    print("============ [LLM RAW OUTPUT] ============")
    print(llm_output)
    print("==========================================")

    # 4) 解析 LLM 输出为 JSON
    llm_json = parse_llm_json_output(llm_output)
    prompt_A_raw = llm_json.get("prompt_A", "")
    prompt_B_raw = llm_json.get("prompt_B", [])

    # ==== A：先修，再用 ====
    prompt_A = fix_prompt_a(prompt_A_raw, task_parsed)

    # ==== B：清洗 + 兜底 ====
    prompt_B_list = clean_prompt_b_list(prompt_B_raw, target_name)
    if not prompt_B_list:
        print("[PROMPT_B] LLM给的B为空或无效，使用env/obj_related规则构造。")
        prompt_B_list = build_default_prompt_b(task_parsed, target_name)

    print(f"[Parsed] prompt_A: {prompt_A}")
    print(f"[Parsed] prompt_B_list (cleaned): {prompt_B_list}")
    
    # 保存LLM生成的提示词到result目录
    result_dir = Path(args.json_path).parent / "result"
    result_dir.mkdir(exist_ok=True)
    
    llm_prompts_output = result_dir / "llm_generated_prompts.json"
    with open(llm_prompts_output, "w", encoding="utf-8") as f:
        json.dump({
            "prompt_A": prompt_A,
            "prompt_B": prompt_B_list,
            "raw_llm_output": llm_output,
            "source_json": str(args.json_path),
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }, f, indent=2, ensure_ascii=False)
    print(f"[Saved] LLM提示词已保存至: {llm_prompts_output}")
    print("")

    # 5) 使用 prompt_A + prompt_B 调 GroundingDINO
    boxes, logits, phrases = run_grounding_dino_on_image(
        image_path=args.image_path,
        prompt_A=prompt_A,
        prompt_B_list=prompt_B_list,
        config_path=args.config_path,
        weights_path=args.weights_path,
        output_path=args.output_image,
    )

    # 打印检测结果摘要
    print("")
    print("[DETECTION SUMMARY]")
    print(f"  Total detected: {len(boxes)} objects")
    if len(boxes) > 0:
        print("  Detected objects:")
        for i, (phrase, score) in enumerate(zip(phrases, logits)):
            print(f"    - {phrase}: {float(score):.3f}")
    else:
        print("  No objects detected.")
    print("")


if __name__ == "__main__":
    main()
