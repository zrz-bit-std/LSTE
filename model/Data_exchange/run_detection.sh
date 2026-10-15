#!/bin/bash
# ============================================================
# 运行脚本：使用VLM输出 -> MiniCPM 0.5B -> GroundingDINO 进行目标检测
# ============================================================

# 设置错误时退出
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ========== 可配置参数 ==========
# JSON文件路径（VLM 8B的输出）
JSON_PATH="${JSON_PATH:-${SCRIPT_DIR}/vlm_prompt/lab/green_cup.json}"

# 待检测的图片路径（请根据实际情况修改）
IMAGE_PATH="${IMAGE_PATH:-${PROJECT_ROOT}/Pic_path/Corridor/corridor2.jpg}"

# 输出的标注图片路径（可选）
OUTPUT_IMAGE="${OUTPUT_IMAGE:-${SCRIPT_DIR}/vlm_prompt/lab/result/detection_result.jpg}"

# GroundingDINO 配置文件
# CONFIG_PATH="${CONFIG_PATH:-${PROJECT_ROOT}/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py}"
CONFIG_PATH="${CONFIG_PATH:-${PROJECT_ROOT}/GroundingDINO/groundingdino/config/GroundingDINO_SwinB_cfg.py}"


# GroundingDINO 权重文件
# WEIGHTS_PATH="${WEIGHTS_PATH:-${PROJECT_ROOT}/GroundingDINO/weights/groundingdino_swint_ogc.pth}"
WEIGHTS_PATH="${WEIGHTS_PATH:-${PROJECT_ROOT}/GroundingDINO/weights/groundingdino_swinb_cogcoor.pth}"

# vLLM 服务地址
VLLM_URL="http://localhost:8000/v1"

# vLLM 模型路径
VLLM_MODEL="${VLLM_MODEL:-${PROJECT_ROOT}/MiniCPM/OpenBMB/MiniCPM4-0___5B}"

# Python脚本路径
SCRIPT_PATH="${SCRIPT_PATH:-${SCRIPT_DIR}/8B-05B.py}"
SCORING_DIR="${SCORING_DIR:-${PROJECT_ROOT}/Scoring_module/scripts}"
VIS_SCRIPT="${VIS_SCRIPT:-${PROJECT_ROOT}/Scoring_module/vis/plot_scores.py}"
VIS_OUTPUT_DIR="${VIS_OUTPUT_DIR:-${PROJECT_ROOT}/Scoring_module/vis}"
BADGE_SCRIPT="${BADGE_SCRIPT:-${PROJECT_ROOT}/Scoring_module/vis/annotate_score_badge.py}"

# 结果目录（与 JSON 同级）以及日志/提示词路径
RESULT_DIR="$(dirname "$JSON_PATH")/result"
PROMPT_RESULT_PATH="$RESULT_DIR/llm_generated_prompts.json"
LOG_DIR="$RESULT_DIR/log"
mkdir -p "$RESULT_DIR" "$LOG_DIR" "$VIS_OUTPUT_DIR"

# 日志输出路径
LOG_PATH="$LOG_DIR/run_detection_$(date '+%Y%m%d-%H%M%S').log"

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

# ========== 启动日志记录 ==========
exec > >(tee -a "$LOG_PATH")
exec 2>&1
echo ""

# ========== 主程序 ==========
print_info "本次运行日志文件: $LOG_PATH"
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
print_info "  2. 调用 MiniCPM 0.5B 生成目标/环境提示词"
print_info "  3. 分别用 GroundingDINO 对目标和环境做两次检测"
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
    if [ -f "$PROMPT_RESULT_PATH" ]; then
        print_success "LLM 提示词记录: $PROMPT_RESULT_PATH"
    fi

    # ========== 评分步骤 ==========
    set +e  # 评分阶段允许非零退出码（如 skip_frame）
    # 所有评分输出统一放在 VIS_OUTPUT_DIR
    S_TARGET_JSON="$VIS_OUTPUT_DIR/S_target.json"
    S_ENV_JSON="$VIS_OUTPUT_DIR/S_env.json"
    S_CTX_JSON="$VIS_OUTPUT_DIR/S_ctx.json"
    S_TOTAL_JSON="$VIS_OUTPUT_DIR/S_total.json"
    SCORE_VIS="$VIS_OUTPUT_DIR/score_vis.png"

    # 清理旧的评分结果，避免使用陈旧文件
    rm -f "$S_TARGET_JSON" "$S_ENV_JSON" "$S_CTX_JSON" "$S_TOTAL_JSON" "$SCORE_VIS"

    TARGET_DET_JSON="$RESULT_DIR/target_detections.json"
    ENV_DET_JSON="$RESULT_DIR/env_detections.json"

    if [ -f "$TARGET_DET_JSON" ]; then
        print_info "计算 S_target..."
        python "$SCORING_DIR/calc_s_target.py" \
            --detections "$TARGET_DET_JSON" \
            --output "$S_TARGET_JSON" \
            --task-json "$JSON_PATH"
    else
        print_warning "未找到目标检测结果: $TARGET_DET_JSON"
    fi

    if [ -f "$ENV_DET_JSON" ]; then
        print_info "计算 S_env..."
        python "$SCORING_DIR/calc_s_env.py" \
            --detections "$ENV_DET_JSON" \
            --output "$S_ENV_JSON" \
            --task-json "$JSON_PATH" \
            --prompt-b-json "$PROMPT_RESULT_PATH"
    else
        print_warning "未找到环境检测结果: $ENV_DET_JSON"
    fi

    if [ -f "$TARGET_DET_JSON" ]; then
        print_info "计算 S_ctx..."
        python "$SCORING_DIR/calc_s_ctx.py" \
            --target-detections "$TARGET_DET_JSON" \
            --env-detections "$ENV_DET_JSON" \
            --output "$S_CTX_JSON" \
            --task-json "$JSON_PATH"
    fi

    print_info "聚合分数..."
    python "$SCORING_DIR/aggregate_target_score.py" \
        --target-input "$S_TARGET_JSON" \
        --env-input "$S_ENV_JSON" \
        --ctx-input "$S_CTX_JSON" > "$S_TOTAL_JSON"
    agg_status=$?
    if [ $agg_status -eq 3 ]; then
        print_warning "聚合器返回 skip_frame，未生成总分。"
    elif [ $agg_status -ne 0 ]; then
        print_warning "聚合器运行异常，退出码: $agg_status"
    else
        print_success "总分结果: $S_TOTAL_JSON"
    fi

    # ========== 可视化 ==========
    if [ $agg_status -eq 0 ]; then
        print_info "生成分数可视化..."
        python "$VIS_SCRIPT" \
            --target "$S_TARGET_JSON" \
            --env "$S_ENV_JSON" \
            --ctx "$S_CTX_JSON" \
            --total "$S_TOTAL_JSON" \
            --output "$SCORE_VIS" || print_warning "可视化生成失败"
        if [ -f "$SCORE_VIS" ]; then
            print_success "分数可视化已保存至: $SCORE_VIS"
        fi
    fi

    if [ -f "$OUTPUT_IMAGE" ] && [ -f "$TARGET_DET_JSON" ] && [ -f "$S_TOTAL_JSON" ] && [ -f "$BADGE_SCRIPT" ]; then
        print_info "刷新目标可视化标记..."
        python "$BADGE_SCRIPT" \
            --image "$OUTPUT_IMAGE" \
            --target-detections "$TARGET_DET_JSON" \
            --score-json "$S_TOTAL_JSON" \
            --threshold 0.7 \
            --output "$OUTPUT_IMAGE" || print_warning "目标标记渲染失败"
    fi

    set -e  # 恢复严格模式
else
    print_error "检测过程中发生错误"
    exit 1
fi
