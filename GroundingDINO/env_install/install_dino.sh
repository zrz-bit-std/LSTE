#!/bin/bash

# GroundingDINO 一键部署脚本（面向 RTX 4080 / CUDA 12 系列显卡）
# 主要流程：
#   1. 检测 GPU/驱动
#   2. 自动创建并激活 conda 环境
#   3. 安装 PyTorch 2.2.2 (CUDA 12.1) 及依赖
#   4. 安装 GroundingDINO（含 CUDA 扩展）
#   5. 下载 bert-base-uncased 到本地，便于离线使用
#   6. 生成 setup_env.sh，后续 source 即可复用环境

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GROUNDINGDINO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_NAME="${GROUNDINGDINO_ENV_NAME:-dino}"
PYTHON_VERSION="${GROUNDINGDINO_PYTHON_VERSION:-3.10}"
TORCH_VERSION="${GROUNDINGDINO_TORCH_VERSION:-2.2.2}"
TORCHVISION_VERSION="${GROUNDINGDINO_TORCHVISION_VERSION:-0.17.2}"
TORCHAUDIO_VERSION="${GROUNDINGDINO_TORCHAUDIO_VERSION:-2.2.2}"
TORCH_INDEX_URL="${GROUNDINGDINO_TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu121}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()  { echo -e "${BLUE}[STEP]${NC} $1"; }

detect_gpu() {
    log_step "检测 NVIDIA GPU"
    if ! command -v nvidia-smi &>/dev/null; then
        log_warn "未检测到 nvidia-smi，跳过 GPU 信息检测"
        return
    fi

    local gpu_info
    gpu_info=$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader | head -n 1)
    log_info "GPU 信息: ${gpu_info}"

    local gpu_mem
    gpu_mem=$(echo "$gpu_info" | awk -F',' '{print $2}' | tr -dc '0-9')
    if [[ -n "$gpu_mem" && "$gpu_mem" -lt 12000 ]]; then
        log_warn "检测到显存 < 12GB，GroundingDINO 可能需要降低 batch/分辨率"
    fi
}

ensure_conda() {
    if ! command -v conda &>/dev/null; then
        log_error "未找到 conda，请先安装 Miniconda/Anaconda 后重试"
        exit 1
    fi
    log_info "已检测到 conda: $(conda --version)"
    eval "$(conda shell.bash hook)"
}

activate_env() {
    log_step "准备 conda 环境: ${ENV_NAME}"
    if conda env list | grep -q "^${ENV_NAME} "; then
        log_info "环境 ${ENV_NAME} 已存在，直接激活"
    else
        log_info "环境 ${ENV_NAME} 不存在，正在创建 (Python ${PYTHON_VERSION})"
        conda create -y -n "${ENV_NAME}" "python=${PYTHON_VERSION}"
    fi
    conda activate "${ENV_NAME}"
    log_info "当前激活环境: ${CONDA_DEFAULT_ENV}"
}

install_pytorch() {
    log_step "安装 PyTorch ${TORCH_VERSION} (CUDA 12.1)"
    pip install --upgrade pip setuptools wheel
    pip install --index-url "${TORCH_INDEX_URL}" \
        "torch==${TORCH_VERSION}" \
        "torchvision==${TORCHVISION_VERSION}" \
        "torchaudio==${TORCHAUDIO_VERSION}"
}

install_dependencies() {
    log_step "安装 GroundingDINO 依赖"
    pip install "numpy<2.0" opencv-python pycocotools supervision addict yapf shapely onnx onnxruntime
    pip install "transformers==4.37.2" "timm==0.9.16"

    log_info "安装 GroundingDINO (含 CUDA 扩展)"
    pushd "${GROUNDINGDINO_ROOT}" >/dev/null
    pip install -e . --no-deps --no-build-isolation
    popd >/dev/null
}

ensure_bert_model() {
    log_step "检测/下载 bert-base-uncased"
    local bert_dir="${GROUNDINGDINO_ROOT}/bert-base-uncased"
    if [[ -f "${bert_dir}/config.json" ]]; then
        log_info "BERT 模型已存在: ${bert_dir}"
        return
    fi

    python - <<PY
from pathlib import Path
from transformers import BertTokenizer, BertModel

target = Path(r"${bert_dir}")
target.mkdir(parents=True, exist_ok=True)

print("开始下载 bert-base-uncased ...")
tokenizer = BertTokenizer.from_pretrained("bert-base-uncased")
tokenizer.save_pretrained(target)
model = BertModel.from_pretrained("bert-base-uncased")
model.save_pretrained(target)
print("BERT 模型已保存到:", target)
PY
}

verify_installation() {
    log_step "验证 GroundingDINO 安装"
    pushd "${GROUNDINGDINO_ROOT}" >/dev/null
    python - <<'PY'
import torch
from groundingdino.util.inference import load_model
from pathlib import Path

config_path = Path("groundingdino/config/GroundingDINO_SwinT_OGC.py")
weights_path = Path("weights/groundingdino_swint_ogc.pth")

print(f"PyTorch 版本: {torch.__version__}")
print(f"CUDA 可用: {torch.cuda.is_available()}")
print(f"配置文件存在: {config_path.exists()}")
print(f"权重文件存在: {weights_path.exists()}")

try:
    load_model(str(config_path), str(weights_path))
    print("✅ 模型载入（即使缺少权重也会尝试，若失败请下载权重）")
except Exception as exc:
    print(f"⚠️  模型载入失败: {exc}")
PY
    popd >/dev/null
}

create_setup_env_script() {
    log_step "生成 setup_env.sh 便捷脚本"
    local conda_base
    conda_base="$(conda info --base)"
    local torch_lib_dir
    torch_lib_dir=$(
        python - <<'PY'
import torch
from pathlib import Path
print(Path(torch.__file__).resolve().parent / "lib")
PY
    )

    cat > "${GROUNDINGDINO_ROOT}/setup_env.sh" <<ENVEOF
#!/bin/bash
source "${conda_base}/etc/profile.d/conda.sh"
conda activate ${ENV_NAME}
export CUDA_HOME="\${CUDA_HOME:-/usr/local/cuda}"
export LD_LIBRARY_PATH="${torch_lib_dir}:\$LD_LIBRARY_PATH"
echo "✅ GroundingDINO 环境已激活 (${ENV_NAME})"
ENVEOF
    chmod +x "${GROUNDINGDINO_ROOT}/setup_env.sh"
    log_info "setup_env.sh 已生成 (source setup_env.sh 即可)"
}

main() {
    echo "========================================="
    echo " GroundingDINO 一键部署 (RTX 4080 / CUDA12)"
    echo "========================================="

    detect_gpu
    ensure_conda
    activate_env
    install_pytorch
    install_dependencies
    ensure_bert_model
    mkdir -p "${GROUNDINGDINO_ROOT}/weights"
    verify_installation
    create_setup_env_script

    echo ""
    log_info "部署完成！"
    log_info " - Conda 环境: ${ENV_NAME}"
    log_info " - PyTorch: ${TORCH_VERSION} (cu121)"
    log_info " - GroundingDINO 根目录: ${GROUNDINGDINO_ROOT}"
    log_info "下一步："
    log_info "  1) 下载权重: wget -P weights https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth"
    log_info "  2) source setup_env.sh"
    log_info "  3) python test/test.py"
}

main "$@"
