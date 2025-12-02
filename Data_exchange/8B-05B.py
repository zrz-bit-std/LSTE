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
import numpy as np

import httpx

from groundingdino.util.inference import load_model, load_image, predict, annotate
import torch
import openai
from torchvision.ops import box_convert

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_VLLM_MODEL_PATH = PROJECT_ROOT / "MiniCPM" / "OpenBMB" / "MiniCPM4-0___5B"
DEFAULT_DINO_CONFIG_PATH = PROJECT_ROOT / "GroundingDINO" / "groundingdino" / "config" / "GroundingDINO_SwinT_OGC.py"
DEFAULT_DINO_WEIGHTS_PATH = PROJECT_ROOT / "GroundingDINO" / "weights" / "groundingdino_swint_ogc.pth"

COLOR_KEYWORDS = {
    "red", "orange", "yellow", "green", "blue", "purple",
    "pink", "brown", "black", "white", "gray", "grey",
    "cyan", "magenta", "beige", "gold", "silver"
}

COLOR_HSV_RANGES = {
    "red": [((0, 70, 50), (10, 255, 255)), ((160, 70, 50), (180, 255, 255))],
    "orange": [((11, 80, 50), (25, 255, 255))],
    "yellow": [((26, 80, 50), (35, 255, 255))],
    "green": [((36, 40, 40), (85, 255, 255))],
    "blue": [((86, 50, 40), (130, 255, 255))],
    "purple": [((131, 40, 40), (155, 255, 255))],
    "pink": [((156, 40, 40), (169, 255, 255))],
    "brown": [((10, 40, 20), (20, 255, 200))],
    "black": [((0, 0, 0), (180, 255, 40))],
    "white": [((0, 0, 200), (180, 40, 255))],
    "gray": [((0, 0, 40), (180, 40, 200))],
    "grey": [((0, 0, 40), (180, 40, 200))],
}


def _extract_env_type_strings(env_type_prior):
    """Normalize env_type_prior into a list of readable strings."""
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
    """Extract color keywords from target attribute descriptions."""
    colors = []
    for attr in attributes or []:
        attr_lower = str(attr).lower()
        for color in COLOR_KEYWORDS:
            if color in attr_lower and color not in colors:
                colors.append(color)
    return colors


def detect_missing_attributes(detected_phrases, required_terms):
    """Check if required terms appear in any of the detection phrases."""
    missing = []
    phrase_lowers = [str(p).lower() for p in detected_phrases or []]
    for term in required_terms or []:
        if not any(term in phrase for phrase in phrase_lowers):
            missing.append(term)
    return missing


def _crop_box(image_source: np.ndarray, box: torch.Tensor) -> np.ndarray:
    if image_source is None or box is None:
        return None
    h, w, _ = image_source.shape
    scale = torch.tensor([w, h, w, h], dtype=torch.float32)
    scaled = (box * scale).unsqueeze(0)
    xyxy = box_convert(boxes=scaled, in_fmt="cxcywh", out_fmt="xyxy")[0]
    x1, y1, x2, y2 = xyxy.tolist()
    x1 = max(0, min(w - 1, int(x1)))
    y1 = max(0, min(h - 1, int(y1)))
    x2 = max(x1 + 1, min(w, int(x2)))
    y2 = max(y1 + 1, min(h, int(y2)))
    return image_source[y1:y2, x1:x2]


def analyze_color_attributes(image_source, boxes, color_terms, ratio_threshold=0.01):
    """Return list of color terms missing from detected boxes via HSV ratio check."""
    if not color_terms or len(boxes) == 0:
        return []

    missing = []
    for color in color_terms:
        hsv_ranges = COLOR_HSV_RANGES.get(color)
        if not hsv_ranges:
            # 没有定义 HSV 范围的颜色直接跳过
            continue
        color_found = False
        for box in boxes:
            crop = _crop_box(image_source, box)
            if crop is None or crop.size == 0:
                continue
            hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)
            mask_total = None
            for lower, upper in hsv_ranges:
                lower_np = np.array(lower, dtype=np.uint8)
                upper_np = np.array(upper, dtype=np.uint8)
                mask = cv2.inRange(hsv, lower_np, upper_np)
                mask_total = mask if mask_total is None else cv2.bitwise_or(mask_total, mask)
            if mask_total is None:
                continue
            ratio = float(cv2.countNonZero(mask_total)) / float(mask_total.size)
            if ratio >= ratio_threshold:
                color_found = True
                break
        if not color_found:
            missing.append(color)
    return missing

def boxes_to_xyxy(boxes, image_shape):
    """Convert cxcywh boxes (normalized 0-1) to pixel xyxy."""
    if boxes is None:
        return []
    if isinstance(boxes, torch.Tensor):
        boxes_tensor = boxes
    else:
        boxes_tensor = torch.tensor(boxes)
    if boxes_tensor.numel() == 0:
        return []
    if boxes_tensor.ndim == 1:
        boxes_tensor = boxes_tensor.unsqueeze(0)
    h, w = image_shape[:2]
    scale = torch.tensor([w, h, w, h], dtype=torch.float32, device=boxes_tensor.device)
    xyxy = box_convert(boxes=boxes_tensor, in_fmt="cxcywh", out_fmt="xyxy")
    xyxy = xyxy * scale
    return xyxy.cpu().numpy().tolist()


def draw_custom_visualization(
    image_source,
    target_boxes,
    target_logits,
    target_phrases,
    env_boxes,
    env_logits,
    env_phrases,
    target_ctx,
    output_path,
):
    """Render custom visualization highlighting target, prompt_B objects, and ctx neighbors."""
    if output_path is None:
        return
    canvas = cv2.cvtColor(image_source, cv2.COLOR_RGB2BGR)
    h, w, _ = canvas.shape

    palette = {
        "target": {"edge": (48, 72, 255), "label_bg": (48, 72, 255)},
        "env": {"edge": (0, 204, 255), "label_bg": (0, 204, 255)},
        "ctx": {"edge": (72, 201, 176), "label_bg": (72, 201, 176)},
    }

    target_boxes_xy = boxes_to_xyxy(target_boxes, canvas.shape)
    env_boxes_xy = boxes_to_xyxy(env_boxes, canvas.shape)

    ctx_terms = []
    for key in ("left", "right"):
        val = target_ctx.get(key)
        if val and str(val).strip().lower() != "none":
            ctx_terms.append(str(val).lower())

    def draw_label(img, text, origin, bg_color):
        (text_w, text_h), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        x, y = origin
        y = max(text_h + 4, y)
        rect_pt1 = (x, y - text_h - baseline - 4)
        rect_pt2 = (x + text_w + 6, y + 2)
        cv2.rectangle(img, rect_pt1, rect_pt2, bg_color, -1)
        cv2.putText(
            img,
            text,
            (x + 3, y - baseline - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

    def draw_boxes(boxes_xy, phrases, scores, color_key, thickness=2):
        color = palette[color_key]["edge"]
        label_bg = palette[color_key]["label_bg"]
        for idx, box in enumerate(boxes_xy):
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness, lineType=cv2.LINE_AA)
            phrase = ""
            if phrases is not None and idx < len(phrases):
                phrase = str(phrases[idx]).strip()
            score_val = ""
            if scores is not None and idx < len(scores):
                score_float = float(scores[idx])
                score_val = f"{score_float:.2f}"
            label = " ".join(x for x in [phrase, score_val] if x).strip()
            if label:
                draw_label(canvas, label, (x1, y1 - 5), label_bg)

    # target: red with translucent fill
    if target_boxes_xy:
        overlay = canvas.copy()
        fill_color = palette["target"]["edge"]
        for box in target_boxes_xy:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), fill_color, -1)
        alpha = 0.25
        cv2.addWeighted(overlay, alpha, canvas, 1 - alpha, 0, canvas)
        draw_boxes(target_boxes_xy, target_phrases, target_logits, "target", thickness=3)

    # prompt_B objects
    if env_boxes_xy:
        draw_boxes(env_boxes_xy, env_phrases, env_logits, "env", thickness=2)

    # ctx neighbors
    ctx_boxes_xy = []
    ctx_labels = []
    ctx_scores = []
    if ctx_terms:
        for boxes_xy, phrases, scores in (
            (target_boxes_xy, target_phrases, target_logits),
            (env_boxes_xy, env_phrases, env_logits),
        ):
            for idx, box in enumerate(boxes_xy):
                phrase = ""
                score_val = 0.0
                if phrases is not None and idx < len(phrases):
                    phrase = phrases[idx]
                if scores is not None and idx < len(scores):
                    score_val = scores[idx]
                phrase_low = str(phrase).lower()
                if any(term in phrase_low for term in ctx_terms):
                    ctx_boxes_xy.append(box)
                    ctx_labels.append(str(phrase))
                    ctx_scores.append(score_val)
        if ctx_boxes_xy:
            draw_boxes(ctx_boxes_xy, ctx_labels, ctx_scores, "ctx", thickness=3)

    cv2.imwrite(output_path, canvas)


# vLLM服务配置
_vllm_base_url = "http://localhost:8000/v1"
_vllm_model_name = str(DEFAULT_VLLM_MODEL_PATH)


# ================= 0. vLLM & LLM 调用 =================

def init_vllm_client():
    """初始化一个全新的 vLLM 客户端，确保无历史上下文。"""
    print(f"[LLM] 新建 vLLM 客户端连接: {_vllm_base_url}")
    client = openai.Client(
        base_url=_vllm_base_url,
        api_key="EMPTY",
        http_client=httpx.Client(trust_env=False)
    )
    try:
        _ = client.chat.completions.create(
            model=_vllm_model_name,
            messages=[{"role": "user", "content": "hello"}],
            max_tokens=5,
            extra_body=dict(add_special_tokens=True),
        )
        print("[LLM] vLLM服务连接成功")
    except Exception as e:
        print(f"[LLM] 警告: vLLM服务连接失败: {e}")
        print("[LLM] 请确保vLLM服务已启动")
        print(f"[LLM] 启动命令: VLLM_USE_V1=0 vllm serve {_vllm_model_name} --trust-remote-code --max-model-len 2048 --gpu-memory-utilization 0.6 --enforce-eager")
        raise
    return client


def call_local_llm(prompt: str, max_new_tokens: int = 256) -> str:
    """
    通过vLLM调用MiniCPM 0.5B生成响应
    """
    client = init_vllm_client()
    try:
        response = client.chat.completions.create(
            model=_vllm_model_name,
            messages=[
                {
                    "role": "system",
                    "content": "You are generating prompts for GroundingDINO. Treat each request as isolated. Never reference or rely on any previous conversation."
                },
                {"role": "user", "content": prompt}
            ],
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
    target_attrs = target.get("attributes", []) or []
    allowed_colors = extract_color_terms(target_attrs)
    allowed_colors_str = ", ".join(allowed_colors) if allowed_colors else "none"
    
    # 提取环境相关物体
    related_structures = env.get("related_structures", [])
    key_objects = obj_related.get("key_objects", [])
    env_type_prior = _extract_env_type_strings(env.get("env_type_prior", []))
    
    # 合并并去重
    env_objects = list(set(related_structures + key_objects + env_type_prior))
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

    name_lower = name.lower()
    allowed_colors = extract_color_terms(attrs)

    # 只取第一个属性作为形容词
    if attrs:
        # 取第一个属性，去掉过于复杂的描述
        first_attr = str(attrs[0]).strip()
        # 如果属性是颜色并且目标名已包含该颜色，则跳过
        first_attr_lower = first_attr.lower()
        color_in_attr = None
        for color in allowed_colors:
            if color in first_attr_lower:
                color_in_attr = color
                break
        if color_in_attr and color_in_attr in name_lower:
            first_attr = ""

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
    env_types = _extract_env_type_strings(env.get("env_type_prior", [])) or []
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
    target_info = (task_parsed.get("target", {}) or {})
    target_name = target_info.get("name", "")
    target_attrs = target_info.get("attributes", []) or []
    allowed_colors = extract_color_terms(target_attrs)
    t_lower = target_name.lower() if target_name else ""

    a_raw = prompt_a_raw
    if isinstance(a_raw, list):
        filtered_parts = []
        for part in a_raw:
            if not isinstance(part, (str, int, float)):
                continue
            part_str = str(part).strip()
            if not part_str:
                continue
            if t_lower and t_lower not in part_str.lower():
                # 丢弃不包含目标名称的片段
                continue
            filtered_parts.append(part_str)
        if not filtered_parts:
            a_raw = ""
        else:
            a_raw = " ".join(filtered_parts)
    elif a_raw is None:
        a_raw = ""
    elif not isinstance(a_raw, str):
        a_raw = str(a_raw)

    a = a_raw.strip()

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

    if not bad and allowed_colors:
        for color in COLOR_KEYWORDS:
            if color in lower_a and color not in allowed_colors:
                bad = True
                print(f"[PROMPT_A] 颜色 {color} 不在允许列表 {allowed_colors} 中，丢弃该提示。")
                break
    elif not bad and not allowed_colors:
        for color in COLOR_KEYWORDS:
            if color in lower_a:
                bad = True
                print("[PROMPT_A] 不应包含颜色描述，但检测到颜色词，丢弃该提示。")
                break

    if bad:
        print("[PROMPT_A] LLM给的A不靠谱，改用规则生成。原A:", a)
        return build_prompt_a_from_task(task_parsed)
    else:
        return a


# ================= 3. GroundingDINO 调用 =================

def run_grounding_dino_with_caption(
    model,
    image_source,
    image,
    caption: str,
    run_label: str,
    box_threshold: float = 0.35,
    text_threshold: float = 0.25,
    output_path=None,
):
    """Run GroundingDINO once with a prepared caption."""
    caption = (caption or "").strip()
    if not caption:
        raise ValueError("Caption for GroundingDINO cannot be empty.")

    print(f"[DINO][{run_label}] Using caption: {caption}")

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
            print(f"[DINO][{run_label}] 没有检测到任何目标")
            print(f"[DINO][{run_label}] 阈值设置: box_threshold={box_threshold}, text_threshold={text_threshold}")
            boxes = torch.tensor([])
            logits = torch.tensor([])
            phrases = []
        else:
            raise

    print(f"[DINO][{run_label}] Detected {len(boxes)} boxes.")
    for i, (p, s) in enumerate(zip(phrases, logits)):
        print(f"  [{run_label}] #{i}: phrase='{p}', score={float(s):.3f}")

    if output_path is not None:
        if len(boxes) > 0:
            annotated_frame = annotate(
                image_source=image_source,
                boxes=boxes,
                logits=logits,
                phrases=phrases
            )
            cv2.imwrite(output_path, annotated_frame)
            print(f"[DINO][{run_label}] 标注图片已保存至: {output_path}")
        else:
            cv2.imwrite(output_path, image_source)
            print(f"[DINO][{run_label}] 未检测到目标，原图已保存至: {output_path}")

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
        default=str(DEFAULT_DINO_CONFIG_PATH),
        help="GroundingDINO config .py path."
    )
    parser.add_argument(
        "--weights_path", type=str,
        default=str(DEFAULT_DINO_WEIGHTS_PATH),
        help="GroundingDINO weights .pth path."
    )
    parser.add_argument(
        "--vllm_url", type=str,
        default="http://localhost:8000/v1",
        help="vLLM服务地址"
    )
    parser.add_argument(
        "--vllm_model", type=str,
        default=str(DEFAULT_VLLM_MODEL_PATH),
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

    # 提前拿 target 信息，后面 A/B + 属性检测都要用
    target_info = (task_parsed.get("target", {}) or {})
    target_name = target_info.get("name", "")
    target_attrs = target_info.get("attributes", []) or []
    required_color_terms = extract_color_terms(target_attrs)

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

    # === 强制加入 target_ctx 里的邻居物体 ===
    target_ctx = task_parsed.get("target_ctx") or {}
    ctx_terms_raw = []
    for key in ("left", "right"):
        val = target_ctx.get(key)
        if val:
            ctx_terms_raw.append(str(val))
    ctx_terms = clean_prompt_b_list(ctx_terms_raw, target_name)
    if ctx_terms:
        # 保证 prompt_B 至少包含 target_ctx 中的邻居物体
        existing_lower = {p.lower() for p in prompt_B_list}
        for term in ctx_terms:
            if term.lower() not in existing_lower:
                prompt_B_list.append(term)
                existing_lower.add(term.lower())

    print(f"[Parsed] prompt_A: {prompt_A}")
    print(f"[Parsed] prompt_B_list (cleaned): {prompt_B_list}")
    
    # 保存LLM生成的提示词到result目录
    result_dir = Path(args.json_path).parent / "result"
    result_dir.mkdir(exist_ok=True)
    
    llm_prompts_output = result_dir / "llm_generated_prompts.json"
    sanitized_llm_output = llm_output
    try:
        llm_json_raw = json.loads(llm_output)
        if isinstance(llm_json_raw, dict):
            raw_a = llm_json_raw.get("prompt_A")
            raw_b = llm_json_raw.get("prompt_B")
            sanitized_llm_output = json.dumps({
                "prompt_A": raw_a,
                "prompt_B": raw_b
            }, ensure_ascii=False, indent=2)
    except Exception:
        pass

    with open(llm_prompts_output, "w", encoding="utf-8") as f:
        json.dump({
            "prompt_A": prompt_A,
            "prompt_B": prompt_B_list,
            "raw_llm_output": sanitized_llm_output,
            "source_json": str(args.json_path),
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }, f, indent=2, ensure_ascii=False)
    print(f"[Saved] LLM提示词已保存至: {llm_prompts_output}")
    print("")

    # 5) 使用 prompt_A / prompt_B 分别调用 GroundingDINO
    model = load_model(args.config_path, args.weights_path)
    image_source, image = load_image(args.image_path)

    target_boxes, target_logits, target_phrases = run_grounding_dino_with_caption(
        model=model,
        image_source=image_source,
        image=image,
        caption=prompt_A,
        run_label="TARGET",
        box_threshold=0.35,
        text_threshold=0.25,
        output_path=None,
    )

    env_caption = " . ".join(prompt_B_list)
    env_boxes, env_logits, env_phrases = run_grounding_dino_with_caption(
        model=model,
        image_source=image_source,
        image=image,
        caption=env_caption,
        run_label="ENV",
        box_threshold=0.35,
        text_threshold=0.25,
        output_path=None,
    )

    # 保存原始检测结果（含 boxes）供评分脚本使用
    def _save_detections(path, boxes, logits, phrases):
        payload = {
            "boxes": boxes.tolist() if hasattr(boxes, "tolist") else boxes,
            "logits": [float(x) for x in logits],
            "phrases": [str(p) for p in phrases],
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)

    target_det_output = result_dir / "target_detections.json"
    env_det_output = result_dir / "env_detections.json"
    _save_detections(target_det_output, target_boxes, target_logits, target_phrases)
    _save_detections(env_det_output, env_boxes, env_logits, env_phrases)
    print(f"[Saved] 目标检测结果: {target_det_output}")
    print(f"[Saved] 环境检测结果: {env_det_output}")

    attribute_filtered_due_to_color = False
    failed_colors = []
    if required_color_terms and len(target_boxes) > 0:
        failed_colors = analyze_color_attributes(image_source, target_boxes, required_color_terms)
        if failed_colors:
            attribute_filtered_due_to_color = True
            print(f"[WARNING][TARGET ATTR] 检测框未满足颜色属性: {failed_colors}")
        else:
            print(f"[TARGET ATTR] 所需颜色属性已在检测框中出现: {required_color_terms}")
    elif required_color_terms:
        print("[WARNING][TARGET ATTR] 未检测到目标框，无法验证颜色属性。")

    print("")
    print("[DETECTION SUMMARY]")
    print(f"  [TARGET] Total detected: {len(target_boxes)} objects")
    if len(target_boxes) > 0:
        print("  [TARGET] Detected objects:")
        for phrase, score in zip(target_phrases, target_logits):
            print(f"    - {phrase}: {float(score):.3f}")
    else:
        if attribute_filtered_due_to_color:
            print("  [TARGET] No objects detected (filtered by attribute check).")
        else:
            print("  [TARGET] No objects detected.")

    print(f"  [ENV] Total detected: {len(env_boxes)} objects")
    if len(env_boxes) > 0:
        print("  [ENV] Detected objects:")
        for phrase, score in zip(env_phrases, env_logits):
            print(f"    - {phrase}: {float(score):.3f}")
    else:
        print("  [ENV] No objects detected.")
    print("")

    if args.output_image:
        draw_custom_visualization(
            image_source=image_source,
            target_boxes=target_boxes,
            target_logits=target_logits,
            target_phrases=target_phrases,
            env_boxes=env_boxes,
            env_logits=env_logits,
            env_phrases=env_phrases,
            target_ctx=target_ctx,
            output_path=args.output_image,
        )
        print(f"[OUTPUT] 自定义可视化已保存至: {args.output_image}")


if __name__ == "__main__":
    main()
