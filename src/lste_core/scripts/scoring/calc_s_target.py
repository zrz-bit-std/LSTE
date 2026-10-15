#!/usr/bin/env python3
"""Compute S_target based on GroundingDINO detections."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
import math
import re
from typing import Any, Iterable, List, Optional, Sequence, Tuple

ScorePair = Tuple[str, float]

PARTIAL_MATCH_THRESHOLD = 0.6  # 至少匹配目标关键词 60%
COMMON_STOPWORDS = {
    "the",
    "a",
    "an",
    "of",
    "with",
    "and",
    "on",
    "in",
    "at",
    "object",
    "item",
    "thing",
}


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def load_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"未找到文件: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"无法解析JSON文件 {path}: {exc}") from exc


def determine_target_name(args: argparse.Namespace) -> str:
    if args.target_name:
        name = args.target_name.strip()
        if not name:
            raise ValueError("--target-name 不能为空字符串")
        return name

    if not args.task_json:
        raise ValueError("必须提供 --target-name 或 --task-json 之一")

    data = load_json(Path(args.task_json))
    root = data.get("task_parsed", data)
    target = (root or {}).get("target", {}) or {}
    name = target.get("name")
    if not name:
        raise ValueError(f"task_json: {args.task_json} 中未找到 target.name")
    return str(name).strip()


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
        for key in ("target", "detections", "objects", "results", "target_output"):
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


def tokenize_phrase(text: str) -> List[str]:
    tokens = re.split(r"[^a-z0-9]+", text.lower())
    return [tok for tok in tokens if tok and tok not in COMMON_STOPWORDS]


def is_partial_match(target_tokens: Sequence[str], phrase: str) -> bool:
    if not target_tokens:
        return False
    phrase_tokens = tokenize_phrase(phrase)
    if not phrase_tokens:
        return False
    hits = sum(1 for tok in target_tokens if tok in phrase_tokens)
    min_hits = max(1, int(math.ceil(len(target_tokens) * PARTIAL_MATCH_THRESHOLD)))
    return hits >= min_hits


def compute_s_target(pairs: Sequence[ScorePair], target_name: str) -> Tuple[Optional[float], Optional[str]]:
    target_lower = target_name.lower()
    target_tokens = tokenize_phrase(target_name)
    candidates: List[Tuple[float, str]] = []
    for phrase, score in pairs:
        phrase_lower = phrase.lower()
        if target_lower and target_lower in phrase_lower:
            candidates.append((score, phrase))
            continue
        if is_partial_match(target_tokens, phrase):
            candidates.append((score, phrase))
    if not candidates:
        return None, None

    best_score, best_phrase = max(candidates, key=lambda item: item[0])
    return clamp(best_score), best_phrase


def write_output(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="计算当前帧的 S_target，并输出给聚合脚本使用"
    )
    parser.add_argument(
        "--detections",
        required=True,
        help="GroundingDINO 检测结果 JSON（需要包含 phrases/logits）",
    )
    parser.add_argument(
        "--detections-key",
        help="如果检测JSON需要先进入某个字段，可用 '.' 语法指定，如 target_run.detections",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="写入 S_target 结果的 JSON 文件路径",
    )

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--target-name", help="目标名称，默认与 VLM 的 target.name 保持一致")
    target_group.add_argument(
        "--task-json",
        help="after_vlm*.json 路径，从中读取 target.name",
    )

    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    try:
        target_name = determine_target_name(args)
    except (FileNotFoundError, ValueError) as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 2

    try:
        detection_json = load_json(Path(args.detections))
        detection_section = select_section(detection_json, args.detections_key)
        pairs = extract_detection_pairs(detection_section)
    except (FileNotFoundError, ValueError) as exc:
        # 兜底：无法解析检测结果时输出 0 分，并标记 detected=false
        output_path = Path(args.output)
        payload = {
            "skip_frame": False,
            "detected": False,
            "S_target": 0.0,
            "best_phrase": None,
            "target_name": target_name,
            "message": f"无法解析检测结果: {exc}",
            "source": str(Path(args.detections).resolve()),
        }
        write_output(output_path, payload)
        print(f"[warning] 无法读取检测结果，S_target=0: {exc}", file=sys.stderr)
        return 0

    score, phrase = compute_s_target(pairs, target_name)

    output_path = Path(args.output)
    if score is None:
        payload = {
            "skip_frame": False,  # 不跳帧，返回0分
            "detected": False,
            "S_target": 0.0,
            "best_phrase": None,
            "target_name": target_name,
            "message": f"在检测结果中未找到包含 '{target_name}' 的框",
            "source": str(Path(args.detections).resolve()),
        }
        write_output(output_path, payload)
        print("[info] 未检测到目标，S_target=0，标记 detected=false", file=sys.stderr)
        return 0

    payload = {
        "skip_frame": False,
        "detected": True,
        "S_target": score,
        "best_phrase": phrase,
        "target_name": target_name,
        "source": str(Path(args.detections).resolve()),
    }
    write_output(output_path, payload)
    print(f"[info] S_target={score:.3f} (phrase='{phrase}')", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
