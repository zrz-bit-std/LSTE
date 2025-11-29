#!/usr/bin/env python3
"""
简单可视化三个子分数与总分，生成柱状图 PNG。

输入：
- --target PATH : calc_s_target 的输出 JSON
- --env PATH    : calc_s_env 的输出 JSON
- --ctx PATH    : calc_s_ctx 的输出 JSON
- --total PATH  : aggregate_target_score 的输出 JSON
- --output PATH : 输出 PNG 路径
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict

import matplotlib.pyplot as plt


def load_score(path: Path, key: str) -> float:
    if not path.exists():
        return 0.0
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict) and data.get("skip_frame"):
        return 0.0
    # 尝试多种 key
    for k in (key, "score", "value"):
        if isinstance(data, dict) and k in data:
            try:
                return float(data[k])
            except (TypeError, ValueError):
                pass
    # 直接裸浮点
    try:
        return float(data)
    except Exception:
        return 0.0


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="可视化 S_target/S_env/S_ctx 及总分")
    ap.add_argument("--target", required=True, help="S_target JSON 路径")
    ap.add_argument("--env", required=True, help="S_env JSON 路径")
    ap.add_argument("--ctx", required=True, help="S_ctx JSON 路径")
    ap.add_argument("--total", required=True, help="aggregate 输出 JSON 路径")
    ap.add_argument("--output", required=True, help="输出 PNG 路径")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    target_path = Path(args.target)
    env_path = Path(args.env)
    ctx_path = Path(args.ctx)
    total_path = Path(args.total)

    scores: Dict[str, float] = {
        "S_target": load_score(target_path, "S_target"),
        "S_env": load_score(env_path, "S_env"),
        "S_ctx": load_score(ctx_path, "S_ctx"),
    }
    total_score = 0.0
    if total_path.exists():
        total_data: Any = json.loads(total_path.read_text(encoding="utf-8"))
        if isinstance(total_data, dict):
            if total_data.get("skip_frame"):
                total_score = 0.0
            elif "S_total" in total_data:
                try:
                    total_score = float(total_data.get("S_total"))
                except Exception:
                    total_score = 0.0
            elif "score" in total_data or "value" in total_data:
                try:
                    total_score = float(total_data.get("score") or total_data.get("value"))
                except Exception:
                    total_score = 0.0
        else:
            try:
                total_score = float(total_data)
            except Exception:
                total_score = 0.0
    scores["S_total"] = total_score

    labels = list(scores.keys())
    values = [scores[k] for k in labels]

    plt.figure(figsize=(6, 4))
    bars = plt.bar(labels, values, color=["#4C72B0", "#55A868", "#C44E52", "#8172B3"])
    plt.ylim(-1, 1)
    plt.axhline(0, color="gray", linewidth=0.8)
    plt.title("Per-frame Scores")
    for bar, val in zip(bars, values):
        plt.text(bar.get_x() + bar.get_width() / 2, val + 0.02 * (1 if val >= 0 else -1), f"{val:.2f}",
                 ha="center", va="bottom" if val >= 0 else "top")
    plt.tight_layout()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=150)
    print(f"[vis] saved to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
