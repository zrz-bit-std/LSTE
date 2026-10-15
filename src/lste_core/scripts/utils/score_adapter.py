#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Wrap the standalone scoring scripts so ROS nodes can reuse them directly."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from lste_msgs.msg import LsteDetections, LsteDetection, LsteTask

from scoring import aggregate_target_score as agg_mod
from scoring import calc_s_ctx as ctx_mod
from scoring import calc_s_env as env_mod
from scoring import calc_s_target as target_mod


@dataclass
class EnvScoreParams:
    lambda_neg: float = 0.7
    pos_midpoint: float = 0.25
    pos_steepness: float = 6.0
    neg_midpoint: float = 0.15
    neg_steepness: float = 12.0


@dataclass
class ScoreResult:
    detected: bool
    s_target: float
    s_env: float
    s_ctx: float
    s_total: float
    pos_ratio: float
    neg_ratio: float
    ctx_coverage: float
    ctx_proximity: float
    target_center: Optional[Tuple[float, float]]
    ctx_centers: Dict[str, Tuple[float, float]]


DEFAULT_ENV_PARAMS = EnvScoreParams()
DEFAULT_WEIGHTS = dict(agg_mod.DEFAULT_WEIGHTS)


def _cxcywh_to_xyxy(det: LsteDetection) -> Optional[List[float]]:
    try:
        cx = float(det.cx)
        cy = float(det.cy)
        w = float(det.w)
        h = float(det.h)
    except (TypeError, ValueError):
        return None
    if w <= 0.0 or h <= 0.0:
        return None
    x1 = cx - 0.5 * w
    y1 = cy - 0.5 * h
    x2 = cx + 0.5 * w
    y2 = cy + 0.5 * h
    return [
        max(0.0, min(1.0, x1)),
        max(0.0, min(1.0, y1)),
        max(0.0, min(1.0, x2)),
        max(0.0, min(1.0, y2)),
    ]


def _to_score_pairs(detections: Sequence[LsteDetection]) -> List[target_mod.ScorePair]:
    pairs: List[target_mod.ScorePair] = []
    for det in detections or []:
        label = (det.label or "").strip()
        try:
            score = float(det.score)
        except (TypeError, ValueError):
            continue
        pairs.append((label, score))
    return pairs


def _make_ctx_entries(detections: Sequence[LsteDetection]) -> List[ctx_mod.DetEntry]:
    entries: List[ctx_mod.DetEntry] = []
    for det in detections or []:
        label = (det.label or "").strip()
        if not label:
            continue
        box = _cxcywh_to_xyxy(det)
        if box is None:
            continue
        try:
            score = float(det.score)
        except (TypeError, ValueError):
            continue
        entries.append(ctx_mod.DetEntry(label, score, box))
    return entries


def _combine_prompt_terms(task: LsteTask, detections: LsteDetections) -> List[str]:
    terms: List[str] = []
    for seq in (
        detections.prompt_b_terms,
        task.env_related_structures,
        task.obj_key_objects,
        task.env_type_prior,
    ):
        for item in (seq or []):
            text = str(item).strip()
            if text:
                terms.append(text)
    return terms


def _combine_negative_terms(task: LsteTask) -> List[str]:
    terms: List[str] = []
    for item in (task.obj_negative_clues or []):
        text = str(item).strip()
        if text:
            terms.append(text)
    return terms


def _compute_s_target(
    target_name: str,
    target_dets: Sequence[LsteDetection],
) -> Tuple[float, bool]:
    pairs = _to_score_pairs(target_dets)
    if not pairs:
        return 0.0, False
    score, best_phrase = target_mod.compute_s_target(pairs, target_name or "")
    if score is None:
        return 0.0, False
    final_score = target_mod.clamp(score, low=0.0, high=1.0)
    detected = best_phrase is not None
    return final_score, detected


def _compute_s_env(
    env_pairs: Sequence[target_mod.ScorePair],
    positive_terms: Iterable[str],
    negative_terms: Iterable[str],
    params: EnvScoreParams,
) -> Tuple[float, float, float]:
    # 如果有效环境检测少于2个，直接不给 env 得分
    if len(env_pairs) < 2:
        return 0.0, 0.0, 0.0
    pos_map = env_mod.normalize_terms(positive_terms)
    neg_map = env_mod.normalize_terms(negative_terms)

    detected_pos: Dict[str, float] = {}
    detected_neg: Dict[str, float] = {}
    phrases_lower = [(phrase, phrase.lower()) for phrase, _ in env_pairs if phrase]

    for _, phrase_low in phrases_lower:
        for term in pos_map.keys():
            if term and term in phrase_low:
                detected_pos.setdefault(term, 0.0)
        for term in neg_map.keys():
            if term and term in phrase_low:
                detected_neg.setdefault(term, 0.0)

    pos_ratio = (len(detected_pos) / len(pos_map)) if pos_map else 0.0
    neg_ratio = (len(detected_neg) / len(neg_map)) if neg_map else 0.0

    pos_score = env_mod.logistic01(pos_ratio, params.pos_midpoint, params.pos_steepness)
    neg_score = env_mod.logistic01(neg_ratio, params.neg_midpoint, params.neg_steepness)
    raw_env = pos_score - params.lambda_neg * neg_score
    s_env = env_mod.clamp(raw_env, low=-1.0, high=1.0)
    return s_env, pos_ratio, neg_ratio


def _compute_s_ctx(
    task: LsteTask,
    target_dets: Sequence[LsteDetection],
    env_dets: Sequence[LsteDetection],
) -> Tuple[float, float, float, Optional[Tuple[float, float]], Dict[str, Tuple[float, float]]]:
    ctx_terms = ctx_mod.normalize_ctx_terms(
        {"left": task.ctx_left, "right": task.ctx_right},
        task.target_name,
    )
    if not ctx_terms:
        return -1.0, 0.0, 0.0, None, {}

    target_entries = _make_ctx_entries(target_dets)
    if not target_entries:
        return -1.0, 0.0, 0.0, None, {}

    env_entries = _make_ctx_entries(env_dets)
    combined = env_entries + target_entries
    best_detail: Optional[Dict[str, Any]] = None
    best_score = -1.1
    for entry in target_entries:
        others = [e for e in combined if e is not entry]
        if not others:
            continue
        detail = ctx_mod.best_ctx_score_for_target(entry, others, ctx_terms)
        score = ctx_mod.clamp(detail.get("raw_score", 0.0), low=-1.0, high=1.0)
        detail["S_ctx"] = score
        if score > best_score:
            best_score = score
            best_detail = detail

    if best_detail is None:
        return -1.0, 0.0, 0.0, None, {}

    target_center = None
    target_box = best_detail.get("target_box")
    if target_box:
        target_center = ctx_mod.center_of_box(target_box)

    ctx_centers: Dict[str, Tuple[float, float]] = {}
    for term, match in (best_detail.get("found") or {}).items():
        if not match:
            continue
        best_match = match.get("best")
        if best_match and best_match.get("box"):
            ctx_centers[term] = ctx_mod.center_of_box(best_match["box"])

    coverage = float(best_detail.get("coverage_ratio", 0.0))
    proximity = float(best_detail.get("proximity_avg", 0.0))
    s_ctx = float(best_detail.get("S_ctx", best_score))
    return s_ctx, coverage, proximity, target_center, ctx_centers


def compute_scores(
    task: LsteTask,
    detections: LsteDetections,
    weights: Optional[Dict[str, float]] = None,
    env_params: Optional[EnvScoreParams] = None,
) -> ScoreResult:
    weights = weights or dict(agg_mod.DEFAULT_WEIGHTS)
    env_params = env_params or DEFAULT_ENV_PARAMS

    s_target, detected = _compute_s_target(task.target_name, detections.target_dets)
    s_env, pos_ratio, neg_ratio = _compute_s_env(
        _to_score_pairs(detections.env_dets),
        _combine_prompt_terms(task, detections),
        _combine_negative_terms(task),
        env_params,
    )
    s_ctx, ctx_coverage, ctx_proximity, target_center, ctx_centers = _compute_s_ctx(
        task,
        detections.target_dets,
        detections.env_dets,
    )

    scores_payload = {"target": s_target, "env": s_env, "ctx": s_ctx}
    s_total = agg_mod.compute_total_score(weights, scores_payload)

    return ScoreResult(
        detected=detected,
        s_target=s_target,
        s_env=s_env,
        s_ctx=s_ctx,
        s_total=s_total,
        pos_ratio=pos_ratio,
        neg_ratio=neg_ratio,
        ctx_coverage=ctx_coverage,
        ctx_proximity=ctx_proximity,
        target_center=target_center,
        ctx_centers=ctx_centers,
    )
