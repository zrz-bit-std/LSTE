#!/usr/bin/env bash
set -euo pipefail

# 配置
# 可修改下列变量指向想要可视化的 JSON
TEST_NAME="${TEST_NAME:-default_test}"
JSON_PATH="${JSON_PATH:-}"
TREE_ROOT="${TREE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/tree}"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/visualized/${TEST_NAME}"
mkdir -p "$VIS_DIR"

# 如果未指定 JSON_PATH，则自动取指定 test_name 目录下最新的 JSON
if [[ -z "$JSON_PATH" ]]; then
  JSON_PATH="$(find "$TREE_ROOT/$TEST_NAME" -name 'access_topo_*.json' -type f 2>/dev/null | sort | tail -n1 || true)"
fi

if [[ -z "$JSON_PATH" || ! -f "$JSON_PATH" ]]; then
  echo "未找到 JSON，检查 JSON_PATH 或 tree 目录：$JSON_PATH" >&2
  exit 1
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
OUT_PATH="$VIS_DIR/topo_${timestamp}.png"

python "$TOOLS_DIR/visualize_access_topo.py" --json "$JSON_PATH" --save "$OUT_PATH"

echo "可视化已保存：$OUT_PATH"
