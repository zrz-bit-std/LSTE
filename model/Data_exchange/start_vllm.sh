#!/bin/bash
# ============================================================
# 启动 vLLM 服务用于 MiniCPM 0.5B
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ========== 颜色输出 ==========
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ========== 配置 ==========
DEFAULT_MODEL_PATH="${PROJECT_ROOT}/MiniCPM/OpenBMB/MiniCPM4-0___5B"
MODEL_PATH="${MODEL_PATH:-$DEFAULT_MODEL_PATH}"
PORT=8000
MAX_MODEL_LEN=2048
GPU_MEM_UTIL=0.6

# ========== 主程序 ==========
print_info "========================================"
print_info "  启动 MiniCPM 0.5B vLLM 服务"
print_info "========================================"
echo ""

# 检查模型路径
if [ ! -d "$MODEL_PATH" ]; then
    print_error "模型路径不存在: $MODEL_PATH"
    exit 1
fi
print_success "✓ 模型路径: $MODEL_PATH"

# 检查端口是否被占用
if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null 2>&1 ; then
    print_warning "端口 $PORT 已被占用"
    print_info "正在尝试停止已有服务..."
    lsof -ti:$PORT | xargs kill -9 2>/dev/null || true
    sleep 2
fi

echo ""
print_info "服务配置:"
print_info "  - 模型: $MODEL_PATH"
print_info "  - 端口: $PORT"
print_info "  - 最大长度: $MAX_MODEL_LEN"
print_info "  - GPU内存利用率: $GPU_MEM_UTIL"
echo ""

print_info "正在启动 vLLM 服务..."
print_info "提示: 按 Ctrl+C 可以停止服务"
echo ""

# 激活环境并启动vLLM
CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate dino

# 安装vllm和openai（如果还没安装）
print_info "检查依赖..."
pip list | grep -q "^vllm " || pip install vllm
pip list | grep -q "^openai " || pip install openai

echo ""
print_success "开始启动服务..."
echo ""

# 启动vLLM服务
VLLM_USE_V1=0 vllm serve "$MODEL_PATH" \
    --trust-remote-code \
    --max-model-len $MAX_MODEL_LEN \
    --gpu-memory-utilization $GPU_MEM_UTIL \
    --port $PORT \
    --enforce-eager
