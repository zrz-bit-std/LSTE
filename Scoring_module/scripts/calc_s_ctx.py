#!/usr/bin/env python3
"""计算参考图上下文一致性得分 S_ctx（基于 target_ctx 最近邻物体）。"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence


CTX_STOPWORDS = {
    "near",
    "on",
    "in",
    "at",
    "by",
    "along",
    "around",
    "beside",
    "between",
    "inside",
    "outside",
    "over",
    "under",
    "the",
    "a",
    "an",
    "of",
    "to",
    "from",
    "with",
}


@dataclass
class DetEntry:
    phrase: str
    score: float
    box: List[float]


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


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
            raise ValueError(f"JSON 中未找到 key 路径 '{key_path}'")
    return section


def _extract_entries_from_lists(phrases: Sequence[Any], logits: Sequence[Any], boxes: Sequence[Any]) -> List[DetEntry]:
    if len(phrases) != len(logits) or len(phrases) != len(boxes):
        raise ValueError("phrases / logits / boxes 长度不一致")
    entries: List[DetEntry] = []
    for p, s, b in zip(phrases, logits, boxes):
        try:
            score = float(s)
        except (TypeError, ValueError) as exc:
            raise ValueError("logits 列表包含非数值元素") from exc
        if not isinstance(b, (list, tuple)) or len(b) != 4:
            raise ValueError("boxes 应为长度4的列表/元组")
        entries.append(DetEntry(str(p), score, [float(x) for x in b]))
    return entries


def _extract_entries_from_items(items: Iterable[Any]) -> Optional[List[DetEntry]]:
    entries: List[DetEntry] = []
    for item in items:
        if not isinstance(item, dict):
            continue
        phrase = item.get("phrase") or item.get("text") or item.get("label")
        score_val = (
            item.get("logit")
            or item.get("score")
            or item.get("confidence")
            or item.get("value")
        )
        box = item.get("box") or item.get("bbox") or item.get("boxes")
        if phrase is None or score_val is None or box is None:
            continue
        if isinstance(box, list) and box and isinstance(box[0], (int, float)):
            box_vec = box
        elif isinstance(box, (list, tuple)) and len(box) == 4:
            box_vec = list(box)
        else:
            continue
        try:
            score = float(score_val)
        except (TypeError, ValueError):
            continue
        entries.append(DetEntry(str(phrase), score, [float(x) for x in box_vec]))
    return entries or None


def extract_entries(det_json: Any) -> List[DetEntry]:
    if isinstance(det_json, dict):
        phrases = det_json.get("phrases") or det_json.get("texts")
        logits = det_json.get("logits") or det_json.get("scores")
        boxes = det_json.get("boxes") or det_json.get("bboxes")
        if phrases is not None and logits is not None and boxes is not None:
            return _extract_entries_from_lists(phrases, logits, boxes)
        for key in ("detections", "objects", "results", "target", "env", "target_output", "env_output"):
            if key in det_json:
                try:
                    return extract_entries(det_json[key])
                except ValueError:
                    continue
    elif isinstance(det_json, list):
        entries = _extract_entries_from_items(det_json)
        if entries:
            return entries
    raise ValueError("检测JSON中未找到有效的 phrases/logits/boxes 信息")


def center_of_box(box: Sequence[float]) -> tuple[float, float]:
    x1, y1, x2, y2 = box
    return (0.5 * (x1 + x2), 0.5 * (y1 + y2))


def compute_distance(b1: Sequence[float], b2: Sequence[float]) -> float:
    cx1, cy1 = center_of_box(b1)
    cx2, cy2 = center_of_box(b2)
    return math.hypot(cx1 - cx2, cy1 - cy2)


def normalize_ctx_terms(target_ctx: dict, target_name: str) -> List[str]:
    """Clean ctx terms using the same rules as prompt_B cleaning."""
    raw_terms = []
    for key in ("left", "right"):
        val = target_ctx.get(key)
        if val:
            raw_terms.append(str(val))

    t_lower = (target_name or "").lower()
    cleaned: List[str] = []
    for term in raw_terms:
        # 去停用词、去空白
        words = [w.strip().lower() for w in term.split() if w.strip()]
        words = [w for w in words if w not in CTX_STOPWORDS]
        if not words:
            continue
        # 限制最多 3 个词，保持与 prompt_B 截断一致
        trimmed = " ".join(words[:3])
        if not trimmed:
            continue
        if t_lower and t_lower in trimmed:
            continue
        cleaned.append(trimmed)

    # 去重，保持顺序
    seen = set()
    final: List[str] = []
    for term in cleaned:
        if term not in seen:
            seen.add(term)
            final.append(term)
    return final


def best_ctx_score_for_target(target_entry: DetEntry, others: List[DetEntry], ctx_terms: List[str]) -> Dict[str, Any]:
    distances = []
    for entry in others:
        dist = compute_distance(target_entry.box, entry.box)
        distances.append((dist, entry))
    distances.sort(key=lambda x: x[0])

    max_possible_dist = math.sqrt(2.0) if distances else 1.0  # 归一化上限，坐标已是 0-1
    topk_cutoff = max(1, math.ceil(0.3 * len(distances))) if distances else 0

    def proximity_score(dist_norm: float, thresh: float = 0.5) -> float:
        return clamp(1.0 - clamp(dist_norm / thresh, low=0.0, high=1.0), low=0.0, high=1.0)

    found: Dict[str, Any] = {}
    found_count = 0
    nearest_hits = 0  # 兼容旧字段：排名前 ctx_len 视为最近邻
    top30_hits = 0
    prox_scores: List[float] = []

    for term in ctx_terms:
        term_low = term.lower()
        term_matches = []
        for idx, (dist, entry) in enumerate(distances):
            if term_low in entry.phrase.lower():
                dist_norm = dist / max_possible_dist if max_possible_dist > 0 else dist
                term_matches.append(
                    {
                        "phrase": entry.phrase,
                        "score": entry.score,
                        "distance": dist,
                        "distance_norm": dist_norm,
                        "rank": idx,
                        "in_top30pct": idx < topk_cutoff if topk_cutoff else False,
                        "box": entry.box,
                    }
                )
        if not term_matches:
            found[term] = None
            continue

        # 选距离最近的作为 best
        best_match = min(term_matches, key=lambda m: m["distance"])
        found_count += 1
        if best_match["rank"] < len(ctx_terms):
            nearest_hits += 1
        if best_match["in_top30pct"]:
            top30_hits += 1

        prox_scores.append(proximity_score(best_match["distance_norm"]))
        found[term] = {
            "best": best_match,
            "matches": term_matches,
        }

    ctx_len = len(ctx_terms)
    coverage_ratio = (top30_hits / ctx_len) if ctx_len else 0.0
    prox_avg = (sum(prox_scores) / ctx_len) if ctx_len else 0.0
    raw = 0.5 * coverage_ratio + 0.5 * prox_avg
    # 如果完全未命中任何 ctx，给予轻微负分，保持与旧逻辑一致
    if found_count == 0:
        raw = -0.4

    return {
        "raw_score": raw,
        "coverage_ratio": coverage_ratio,
        "proximity_avg": prox_avg,
        "top30_hits": top30_hits,
        "top30_cutoff": topk_cutoff,
        "found_count": found_count,
        "nearest_hits": nearest_hits,
        "found": found,
        "best_target_phrase": target_entry.phrase,
        "target_box": target_entry.box,
    }


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="计算 S_ctx（参考图上下文最近邻一致性）")
    parser.add_argument(
        "--target-detections",
        required=True,
        help="GroundingDINO 目标检测结果 JSON（含 target boxes/phrases/logits/boxes）",
    )
    parser.add_argument(
        "--target-key",
        help="如 JSON 有嵌套可用 '.' 语法进入，比如 target_run.detections",
    )
    parser.add_argument(
        "--env-detections",
        help="可选：环境检测结果 JSON，会作为邻居候选加入",
    )
    parser.add_argument(
        "--env-key",
        help="环境检测 JSON 的嵌套 key 路径",
    )
    parser.add_argument("--output", required=True, help="写入 S_ctx 结果的 JSON 文件")
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--target-name", help="目标名称，默认与 VLM 的 target.name 保持一致")
    target_group.add_argument("--task-json", help="after_vlm*.json 路径，从中读取 target.name 与 target_ctx")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    try:
        target_data = load_json(Path(args.target_detections))
        target_section = select_section(target_data, args.target_key)
        target_entries = extract_entries(target_section)
    except (FileNotFoundError, ValueError) as exc:
        payload = {
            "skip_frame": False,
            "S_ctx": -1.0,
            "raw_score": -1.0,
            "ctx_terms": [],
            "found_count": 0,
            "nearest_hits": 0,
            "best_target_phrase": None,
            "found": {},
            "target_center": None,
            "ctx_centers": {},
            "reason": f"无法读取 target 检测结果: {exc}",
            "sources": {
                "target": str(Path(args.target_detections).resolve()),
                "env": str(Path(args.env_detections).resolve()) if args.env_detections else None,
            },
        }
        Path(args.output).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"[warning] 无法读取 target 检测结果，S_ctx=-1: {exc}", file=sys.stderr)
        return 0

    env_entries: List[DetEntry] = []
    if args.env_detections:
        try:
            env_data = load_json(Path(args.env_detections))
            env_section = select_section(env_data, args.env_key)
            env_entries = extract_entries(env_section)
        except (FileNotFoundError, ValueError) as exc:
            print(f"[warning] 读取 env 检测失败，忽略: {exc}", file=sys.stderr)

    if args.target_name:
        target_name = args.target_name.strip()
        target_ctx = {}
    else:
        try:
            task_json = load_json(Path(args.task_json))
            task_root = task_json.get("task_parsed", task_json) or {}
            target_name = str((task_root.get("target", {}) or {}).get("name", "")).strip()
            target_ctx = task_root.get("target_ctx", {}) or {}
        except (FileNotFoundError, ValueError) as exc:
            payload = {
                "skip_frame": False,
                "S_ctx": -1.0,
                "raw_score": -1.0,
                "ctx_terms": [],
                "found_count": 0,
                "nearest_hits": 0,
                "best_target_phrase": None,
                "found": {},
                "target_center": None,
                "ctx_centers": {},
                "reason": f"无法读取 task_json: {exc}",
            }
            Path(args.output).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            print(f"[warning] 无法读取 task_json，S_ctx=-1: {exc}", file=sys.stderr)
            return 0

    ctx_terms = normalize_ctx_terms(target_ctx, target_name)
    if not ctx_terms:
        payload = {
            "skip_frame": False,
            "S_ctx": -1.0,
            "raw_score": -1.0,
            "ctx_terms": [],
            "found_count": 0,
            "nearest_hits": 0,
            "best_target_phrase": None,
            "found": {},
            "target_center": None,
            "ctx_centers": {},
            "reason": "target_ctx 为空，S_ctx=-1",
        }
        Path(args.output).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print("[warning] target_ctx 为空，S_ctx=-1", file=sys.stderr)
        return 0

    # 找到目标框
    t_lower = target_name.lower()
    target_boxes = [e for e in target_entries if t_lower and t_lower in e.phrase.lower()]
    if not target_boxes:
        payload = {
            "skip_frame": False,
            "S_ctx": -1.0,
            "raw_score": -1.0,
            "ctx_terms": ctx_terms,
            "found_count": 0,
            "nearest_hits": 0,
            "best_target_phrase": None,
            "found": {},
            "target_center": None,
            "ctx_centers": {},
            "reason": "未检测到目标，S_ctx=-1",
        }
        Path(args.output).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print("[warning] 未检测到目标，S_ctx=-1", file=sys.stderr)
        return 0

    # 组合邻居候选（其他目标检测 + 环境检测）
    combined_neighbors = env_entries + target_entries

    best_detail: Optional[Dict[str, Any]] = None
    best_score = -1.0
    for t_entry in target_boxes:
        others = [e for e in combined_neighbors if e is not t_entry]
        if not others:
            continue
        detail = best_ctx_score_for_target(t_entry, others, ctx_terms)
        score = clamp(detail["raw_score"], low=-1.0, high=1.0)
        detail["S_ctx"] = score
        if score > best_score:
            best_score = score
            best_detail = detail

    if best_detail is None:
        payload = {
            "skip_frame": False,
            "S_ctx": -1.0,
            "raw_score": -1.0,
            "ctx_terms": ctx_terms,
            "found_count": 0,
            "nearest_hits": 0,
            "best_target_phrase": None,
            "found": {},
            "target_center": None,
            "ctx_centers": {},
            "reason": "无邻居框，S_ctx=-1",
        }
        Path(args.output).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print("[warning] 无邻居框，S_ctx=-1", file=sys.stderr)
        return 0

    # 输出
    target_box = best_detail.get("target_box")
    target_center = center_of_box(target_box) if target_box else None
    ctx_centers = {}
    for term, match in best_detail.get("found", {}).items():
        if match:
            best = match.get("best") if isinstance(match, dict) else None
            if best and best.get("box"):
                ctx_centers[term] = center_of_box(best["box"])

    payload = {
        "skip_frame": False,
        "S_ctx": best_detail.get("S_ctx", 0.0),
        "raw_score": best_detail.get("raw_score", 0.0),
        "coverage_ratio": best_detail.get("coverage_ratio", 0.0),
        "proximity_avg": best_detail.get("proximity_avg", 0.0),
        "top30_hits": best_detail.get("top30_hits", 0),
        "top30_cutoff": best_detail.get("top30_cutoff", 0),
        "ctx_terms": ctx_terms,
        "found_count": best_detail.get("found_count", 0),
        "nearest_hits": best_detail.get("nearest_hits", 0),
        "best_target_phrase": best_detail.get("best_target_phrase"),
        "found": best_detail.get("found"),
        "target_center": target_center,
        "ctx_centers": ctx_centers,
        "sources": {
            "target": str(Path(args.target_detections).resolve()),
            "env": str(Path(args.env_detections).resolve()) if args.env_detections else None,
        },
    }

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(
        f"[info] S_ctx={payload['S_ctx']:.3f} (found={payload['found_count']}, nearest_hits={payload['nearest_hits']})",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
