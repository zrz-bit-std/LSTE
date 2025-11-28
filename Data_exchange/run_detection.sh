#!/bin/bash
# ============================================================
# 运行脚本：使用VLM输出 -> MiniCPM 0.5B -> GroundingDINO 进行目标检测
# ============================================================

# 设置错误时退出
set -e

# ========== 可配置参数 ==========
# JSON文件路径（VLM 8B的输出）
JSON_PATH="/home/zrz/Desktop/LSTE/Data_exchange/vlm_prompt/after_vlm03.json"

# 待检测的图片路径（请根据实际情况修改）
IMAGE_PATH="/home/zrz/Desktop/LSTE/GroundingDINO/test/pic/corridor.png"

# 输出的标注图片路径（可选）
OUTPUT_IMAGE="/home/zrz/Desktop/LSTE/Data_exchange/vlm_prompt/result/detection_result.jpg"

# GroundingDINO 配置文件
CONFIG_PATH="/home/zrz/Desktop/LSTE/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"

# GroundingDINO 权重文件
WEIGHTS_PATH="/home/zrz/Desktop/LSTE/GroundingDINO/weights/groundingdino_swint_ogc.pth"

# vLLM 服务地址
VLLM_URL="http://localhost:8000/v1"

# vLLM 模型路径
VLLM_MODEL="/home/zrz/Desktop/LSTE/MiniCPM/OpenBMB/MiniCPM4-0___5B"

# Python脚本路径
SCRIPT_PATH="/home/zrz/Desktop/LSTE/Data_exchange/8B-05B.py"

# ========== 颜色输出 ==========
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ========== 打印信息函数 ==========
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

# ========== 主程序 ==========
print_info "========================================"
print_info "  VLM->MiniCPM->GroundingDINO 检测流程"
print_info "========================================"
echo ""

# 1. 检查文件是否存在
print_info "检查必要文件..."
if [ ! -f "$JSON_PATH" ]; then
    print_error "JSON文件不存在: $JSON_PATH"
    exit 1
fi
print_success "✓ JSON文件: $JSON_PATH"

if [ ! -f "$IMAGE_PATH" ]; then
    print_warning "图片文件不存在: $IMAGE_PATH"
    print_warning "请修改脚本中的 IMAGE_PATH 变量指向实际的图片路径"
    exit 1
fi
print_success "✓ 图片文件: $IMAGE_PATH"

if [ ! -f "$CONFIG_PATH" ]; then
    print_error "配置文件不存在: $CONFIG_PATH"
    exit 1
fi
print_success "✓ 配置文件: $CONFIG_PATH"

if [ ! -f "$WEIGHTS_PATH" ]; then
    print_error "权重文件不存在: $WEIGHTS_PATH"
    exit 1
fi
print_success "✓ 权重文件: $WEIGHTS_PATH"

if [ ! -d "$VLLM_MODEL" ]; then
    print_error "vLLM模型路径不存在: $VLLM_MODEL"
    exit 1
fi
print_success "✓ vLLM模型: $VLLM_MODEL"

if [ ! -f "$SCRIPT_PATH" ]; then
    print_error "Python脚本不存在: $SCRIPT_PATH"
    exit 1
fi
print_success "✓ Python脚本: $SCRIPT_PATH"

echo ""

# 2. 检查vLLM服务是否运行
print_info "检查vLLM服务..."
if curl -s "http://localhost:8000/health" > /dev/null 2>&1; then
    print_success "✓ vLLM服务正在运行"
else
    print_warning "vLLM服务未运行!"
    print_info "请在另一个终端启动vLLM服务："
    echo ""
    echo "  VLLM_USE_V1=0 vllm serve $VLLM_MODEL \\"
    echo "      --trust-remote-code \\"
    echo "      --max-model-len 2048 \\"
    echo "      --gpu-memory-utilization 0.6 \\"
    echo "      --enforce-eager"
    echo ""
    read -p "是否继续运行? (y/N): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "已取消"
        exit 1
    fi
fi

echo ""

# 3. 激活conda环境
print_info "激活conda环境..."
# 获取conda路径
CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh"

# 激活dino环境用于GroundingDINO
print_info "激活 dino 环境..."
conda activate dino
if [ $? -ne 0 ]; then
    print_error "无法激活 dino 环境，请确保环境已创建"
    print_info "提示：可以运行 conda env list 查看可用环境"
    exit 1
fi
print_success "✓ conda环境已激活: dino"

echo ""

# 4. 运行Python脚本
print_info "开始运行检测流程..."
print_info "这将包括以下步骤："
print_info "  1. 读取 after_vlm.json"
print_info "  2. 调用 MiniCPM 0.5B 生成检测提示词"
print_info "  3. 使用 GroundingDINO 进行目标检测"
echo ""

python "$SCRIPT_PATH" \
    --json_path "$JSON_PATH" \
    --image_path "$IMAGE_PATH" \
    --output_image "$OUTPUT_IMAGE" \
    --config_path "$CONFIG_PATH" \
    --weights_path "$WEIGHTS_PATH" \
    --vllm_url "$VLLM_URL" \
    --vllm_model "$VLLM_MODEL"

if [ $? -eq 0 ]; then
    echo ""
    print_success "========================================"
    print_success "  检测完成！"
    print_success "========================================"
    if [ -f "$OUTPUT_IMAGE" ]; then
        print_success "标注图片已保存至: $OUTPUT_IMAGE"
    fi
else
    print_error "检测过程中发生错误"
    exit 1
fi
