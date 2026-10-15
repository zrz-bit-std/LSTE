#!/usr/bin/env python3
"""计算 GroundingDINO 检测结果的环境一致性得分 S_env。"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

ScorePair = Tuple[str, float]


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def logistic01(x: float, midpoint: float, steepness: float) -> float:
    """Sigmoid-style压缩，让覆盖率差异呈现更激进的分数梯度。"""
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    try:
        value = 1.0 / (1.0 + math.exp(-steepness * (x - midpoint)))
    except OverflowError:
        value = 0.0 if x < midpoint else 1.0
    return clamp(value)


def load_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"未找到文件: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"无法解析JSON文件 {path}: {exc}") from exc


def select_section(payload: Any, key_path: Optional[str]) -> Any:
    if not key_path:
        return payload
    section = payload
    for key in key_path.split("."):
        if isinstance(section, dict) and key in section:
            section = section[key]
        else:
            raise ValueError(f"detections JSON 中未找到 key 路径 '{key_path}'")
    return section


def _pairs_from_lists(phrases: Sequence[Any], logits: Sequence[Any]) -> List[ScorePair]:
    if len(phrases) != len(logits):
        raise ValueError("phrases 和 logits 长度不一致")
    pairs: List[ScorePair] = []
    for phrase, logit in zip(phrases, logits):
        try:
            score = float(logit)
        except (TypeError, ValueError) as exc:
            raise ValueError("logits 列表包含非数值元素") from exc
        pairs.append((str(phrase), score))
    return pairs


def _pairs_from_entries(entries: Iterable[Any]) -> Optional[List[ScorePair]]:
    pairs: List[ScorePair] = []
    for item in entries:
        if not isinstance(item, dict):
            continue
        phrase = item.get("phrase") or item.get("text") or item.get("label")
        score_val = (
            item.get("logit")
            or item.get("score")
            or item.get("confidence")
            or item.get("value")
        )
        if phrase is None or score_val is None:
            continue
        try:
            score = float(score_val)
        except (TypeError, ValueError):
            continue
        pairs.append((str(phrase), score))
    return pairs or None


def extract_detection_pairs(det_json: Any) -> List[ScorePair]:
    if isinstance(det_json, dict):
        phrases = det_json.get("phrases") or det_json.get("texts")
        logits = det_json.get("logits") or det_json.get("scores")
        if phrases is not None and logits is not None:
            return _pairs_from_lists(phrases, logits)
        for key in ("env", "detections", "objects", "results", "env_output"):
            if key in det_json:
                try:
                    return extract_detection_pairs(det_json[key])
                except ValueError:
                    continue
    elif isinstance(det_json, list):
        pairs = _pairs_from_entries(det_json)
        if pairs:
            return pairs
    raise ValueError("检测JSON中未找到有效的 phrases/logits 信息")


def normalize_terms(terms: Iterable[Any]) -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    for term in terms or []:
        text = str(term).strip()
        if not text:
            continue
        lower = text.lower()
        if lower not in mapping:
            mapping[lower] = text
    return mapping


def load_prompt_b_terms(path: Path, key: Optional[str]) -> List[str]:
    data = load_json(path)
    candidate_keys = [key] if key else []
    candidate_keys += ["prompt_B", "prompt_b", "promptB", "prompt_b_list"]
    for candidate in candidate_keys:
        if not candidate:
            continue
        value = data
        for part in candidate.split("."):
            if isinstance(value, dict) and part in value:
                value = value[part]
            else:
                value = None
                break
        if isinstance(value, list):
            return [str(item) for item in value]
    raise ValueError(f"文件 {path} 中未找到 prompt_B 列表，可用 --prompt-b-key 手动指定路径")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="计算环境一致性得分 S_env")
    parser.add_argument(
        "--detections",
        required=True,
        help="GroundingDINO 环境检测结果 JSON（需要 phrases/logits）",
    )
    parser.add_argument(
        "--detections-key",
        help="如果 JSON 有嵌套结构，可用 '.' 语法选择字段",
    )
    parser.add_argument("--output", required=True, help="写入 S_env 结果的 JSON 文件")
    parser.add_argument(
        "--task-json",
        required=True,
        help="after_vlm*.json 路径，用于读取 related_structures 与 negative_clues",
    )
    parser.add_argument(
        "--prompt-b-json",
        required=False,
        help="llm_generated_prompts.json 路径，用于读取 prompt_B 列表",
    )
    parser.add_argument(
        "--prompt-b-key",
        help="prompt_B 在 JSON 中的 key，默认自动探测，可用 a.b.c 形式",
    )
    parser.add_argument(
        "--prompt-b-term",
        action="append",
        default=None,
        help="额外补充的 prompt_B 物体名称，可多次添加",
    )
    parser.add_argument(
        "--include-related-structures",
        action="store_true",
        help="是否将 task_json.env.related_structures 也计入正向关键词（默认仅使用 prompt_B 清洗后的列表）",
    )
    parser.add_argument(
        "--lambda-neg",
        type=float,
        default=0.7,
        help="负面线索的惩罚权重（默认0.7）",
    )
    parser.add_argument(
        "--pos-midpoint",
        type=float,
        default=0.4,
        help="正向覆盖率 Sigmoid 中点（默认0.4，越低越容易得高分）",
    )
    parser.add_argument(
        "--pos-steepness",
        type=float,
        default=10.0,
        help="正向覆盖率 Sigmoid 斜率（默认10，更大代表更激进）",
    )
    parser.add_argument(
        "--neg-midpoint",
        type=float,
        default=0.15,
        help="负向命中率 Sigmoid 中点（默认0.15，越低越容易被惩罚）",
    )
    parser.add_argument(
        "--neg-steepness",
        type=float,
        default=12.0,
        help="负向命中率 Sigmoid 斜率（默认12，越大越激进）",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    try:
        detection_json = load_json(Path(args.detections))
        detection_section = select_section(detection_json, args.detections_key)
        pairs = extract_detection_pairs(detection_section)
    except (FileNotFoundError, ValueError) as exc:
        print(f"[error] 无法读取检测结果: {exc}", file=sys.stderr)
        return 2

    try:
        task_data = load_json(Path(args.task_json))
        task_root = task_data.get("task_parsed", task_data) or {}
        env = (task_root.get("env") or {})
        related_structures = env.get("related_structures", []) or []
        obj_related = task_root.get("obj_related") or {}
        negative_clues = obj_related.get("negative_clues", []) or []
    except (FileNotFoundError, ValueError) as exc:
        print(f"[error] 无法读取 task_json: {exc}", file=sys.stderr)
        return 2

    prompt_terms: List[str] = []
    if args.prompt_b_json:
        try:
            prompt_terms.extend(
                load_prompt_b_terms(Path(args.prompt_b_json), args.prompt_b_key)
            )
        except (FileNotFoundError, ValueError) as exc:
            print(f"[error] 无法读取 prompt_B: {exc}", file=sys.stderr)
            return 2
    if args.prompt_b_term:
        prompt_terms.extend(args.prompt_b_term)

    if args.include_related_structures:
        prompt_terms.extend(related_structures)

    if not prompt_terms:
        print("[warning] 未提供 prompt_B 词汇，正向关键词集合为空", file=sys.stderr)

    pos_map = normalize_terms(prompt_terms)
    neg_map = normalize_terms(negative_clues)

    detected_pos: Dict[str, float] = {}
    detected_neg: Dict[str, float] = {}
    phrases_lower = [(phrase, phrase.lower()) for phrase, _ in pairs]

    for phrase, phrase_low in phrases_lower:
        for term in pos_map.keys():
            if term and term in phrase_low:
                detected_pos.setdefault(term, 0.0)
        for term in neg_map.keys():
            if term and term in phrase_low:
                detected_neg.setdefault(term, 0.0)

    pos_ratio = (
        len(detected_pos) / len(pos_map)
        if pos_map
        else 0.0
    )
    neg_ratio = (
        len(detected_neg) / len(neg_map)
        if neg_map
        else 0.0
    )

    pos_score = logistic01(pos_ratio, args.pos_midpoint, args.pos_steepness)
    neg_score = logistic01(neg_ratio, args.neg_midpoint, args.neg_steepness)
    raw_env = pos_score - args.lambda_neg * neg_score
    s_env = clamp(raw_env, low=-1.0, high=1.0)

    output_payload = {
        "skip_frame": False,
        "S_env": s_env,
        "details": {
            "pos_terms_total": len(pos_map),
            "pos_hits": len(detected_pos),
            "neg_terms_total": len(neg_map),
            "neg_hits": len(detected_neg),
            "pos_ratio": pos_ratio,
            "neg_ratio": neg_ratio,
            "pos_score": pos_score,
            "neg_score": neg_score,
            "lambda_neg": args.lambda_neg,
            "pos_terms": sorted(pos_map.values()),
            "neg_terms": sorted(neg_map.values()),
            "detected_pos_terms": sorted(pos_map[t] for t in detected_pos),
            "detected_neg_terms": sorted(neg_map[t] for t in detected_neg),
        },
        "source": str(Path(args.detections).resolve()),
    }

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(output_payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    print(
        f"[info] S_env={s_env:.3f} (pos_hits={len(detected_pos)}, neg_hits={len(detected_neg)})",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
