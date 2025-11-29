#!/usr/bin/env python3
"""Aggregate S_target, S_env, and S_ctx to produce a per-frame target likelihood.

Tunable hyper-parameters across the scoring pipeline (方便调试)：
1) 聚合权重（本脚本）：--w-target / --w-env / --w-ctx （默认 0.5 / 0.25 / 0.25）
2) S_target：无额外超参，取包含目标名的最大 logit
3) S_env（calc_s_env.py）：
   - --lambda-neg （负面线索惩罚权重，默认 0.7）
   - --pos-midpoint / --pos-steepness （正向覆盖 Sigmoid 形状，默认 0.4 / 10）
   - --neg-midpoint / --neg-steepness （负向命中 Sigmoid 形状，默认 0.15 / 12）
4) S_ctx（calc_s_ctx.py）：
   - 最近邻 vs 检出占比权重固定为 0.6 / 0.4；可修改 raw_score 公式或 clamp 范围 [-1,1]

调整思路：假阳性多时提高 w_env 或 w_ctx；目标置信度偏低但上下文强时降低 w_target 并提高 w_ctx；
环境误检导致扣分过猛时调低 lambda-neg 或增大 neg-midpoint；上下文要求太苛刻时减小 S_ctx 中最近邻权重。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, Optional, Tuple

# ===== 超参（集中修改） =====
SCORE_MIN = -1.0  # 单项得分下限（允许负分）
SCORE_MAX = 1.0   # 单项得分上限
TOTAL_MIN = 0.0   # 总分下限
TOTAL_MAX = 1.0   # 总分上限

DEFAULT_WEIGHTS: Dict[str, float] = {
    "target": 0.2,
    "env": 0.3,
    "ctx": 0.5,
}


class SkipFrame(Exception):
    """Raised when a metric script indicates the current frame should be skipped."""

    def __init__(self, source: Path, reason: str, details: Optional[dict] = None) -> None:
        super().__init__(reason)
        self.source = source
        self.reason = reason
        self.details = details or {}


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    """Clamp *value* into the inclusive [low, high] range."""
    return max(low, min(high, value))


def parse_score_file(path: Path, fallback_key: Optional[str] = None) -> Tuple[Optional[float], dict]:
    """Extract a numeric score and meta from *path*.

    The helper is intentionally flexible so future metric scripts can output either
    a bare float (e.g. ``0.87``) or a JSON blob such as ``{"score": 0.87}`` or
    ``{"S_target": 0.87}``. The ``fallback_key`` parameter hints which JSON key
    should be checked when multiple fields exist.
    """

    try:
        content = path.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        # 缺文件时直接返回 0 分，附带元数据标记
        return 0.0, {"missing": True, "path": str(path)}
    except OSError as exc:
        raise RuntimeError(f"Failed to read score file '{path}': {exc}") from exc

    if not content:
        raise ValueError(f"Score file '{path}' is empty.")

    # Try parsing as a bare float first.
    try:
        return float(content), {}
    except ValueError:
        pass

    try:
        payload = json.loads(content)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"Cannot parse score file '{path}' as float or JSON: {exc}"
        ) from exc

    if isinstance(payload, dict) and payload.get("skip_frame"):
        reason = payload.get("reason") or payload.get("message") or "frame skipped"
        raise SkipFrame(path, str(reason), payload)

    candidate_keys = ["score", "value"]
    if fallback_key:
        candidate_keys.insert(0, fallback_key)

    for key in candidate_keys:
        if key in payload:
            try:
                return float(payload[key]), payload
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Value for key '{key}' in file '{path}' is not numeric."
                ) from exc

    raise ValueError(
        f"File '{path}' does not contain any of the expected keys: {candidate_keys}"
    )


def resolve_score(
    name: str,
    args: argparse.Namespace,
    fallback_key: Optional[str] = None,
) -> Tuple[float, dict]:
    """Resolve the score for *name* from either a direct CLI value or a file."""

    manual_value = getattr(args, f"{name}_score")
    if manual_value is not None:
        return clamp(manual_value, low=SCORE_MIN, high=SCORE_MAX), {}

    path_arg = getattr(args, f"{name}_input")
    if path_arg is None:
        raise ValueError(
            f"Missing both --{name}-score and --{name}-input. "
            f"Provide one so the aggregator can read {name} output."
        )

    score, meta = parse_score_file(Path(path_arg), fallback_key=fallback_key)
    score = 0.0 if score is None else score
    return clamp(score, low=SCORE_MIN, high=SCORE_MAX), meta


def compute_total_score(weights: Dict[str, float], scores: Dict[str, float]) -> float:
    """Compute the weighted sum of the per-metric scores and clamp to [TOTAL_MIN, TOTAL_MAX]."""
    total = (
        weights["target"] * scores["target"]
        + weights["env"] * scores["env"]
        + weights["ctx"] * scores["ctx"]
    )
    return clamp(total, low=TOTAL_MIN, high=TOTAL_MAX)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Combine S_target, S_env, and S_ctx outputs into a single per-frame "
            "target likelihood score."
        )
    )

    # Each metric can be provided via a direct numeric score or via a file path
    # produced by the yet-to-be-written metric scripts.
    for metric in ("target", "env", "ctx"):
        parser.add_argument(
            f"--{metric}-score",
            type=float,
            default=None,
            help=(
                f"Directly provide the S_{metric} value (0-1). "
                "Overrides --{metric}-input when set."
            ),
        )
        parser.add_argument(
            f"--{metric}-input",
            metavar="PATH",
            default=None,
            help=(
                f"Path to the file emitted by the {metric} metric script. "
                "The file can contain a bare float or JSON with a 'score' field."
            ),
        )

    parser.add_argument(
        "--w-target",
        type=float,
        default=DEFAULT_WEIGHTS["target"],
        help="Weight for S_target (default: %(default)s)",
    )
    parser.add_argument(
        "--w-env",
        type=float,
        default=DEFAULT_WEIGHTS["env"],
        help="Weight for S_env (default: %(default)s)",
    )
    parser.add_argument(
        "--w-ctx",
        type=float,
        default=DEFAULT_WEIGHTS["ctx"],
        help="Weight for S_ctx (default: %(default)s)",
    )

    parser.add_argument(
        "--raw",
        action="store_true",
        help=(
            "When set, only print the final score. Otherwise a JSON summary is emitted."
        ),
    )

    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)

    try:
        scores = {}
        metas = {}
        scores["target"], metas["target"] = resolve_score("target", args, fallback_key="S_target")
        scores["env"], metas["env"] = resolve_score("env", args, fallback_key="S_env")
        scores["ctx"], metas["ctx"] = resolve_score("ctx", args, fallback_key="S_ctx")
    except SkipFrame as exc:
        print(
            f"[skip] Metric file '{exc.source}' requested skip: {exc.reason}",
            file=sys.stderr,
        )
        payload = {
            "skip_frame": True,
            "reason": exc.reason,
            "source": str(exc.source),
        }
        if exc.details:
            payload["details"] = exc.details
        if args.raw:
            print("skip")
        else:
            json.dump(payload, sys.stdout, indent=2, ensure_ascii=False)
            sys.stdout.write("\n")
        return 3
    except ValueError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 2

    weights = {
        "target": args.w_target,
        "env": args.w_env,
        "ctx": args.w_ctx,
    }

    total = compute_total_score(weights, scores)

    if args.raw:
        print(f"{total:.6f}")
    else:
        payload = {
            "scores": {
                "S_target": scores["target"],
                "S_env": scores["env"],
                "S_ctx": scores["ctx"],
            },
            "weights": weights,
            "S_total": total,
            "detected": metas.get("target", {}).get("detected", True),
        }
        json.dump(payload, sys.stdout, indent=2)
        sys.stdout.write("\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
