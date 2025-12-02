#!/bin/bash

# ============================================================================
# MiniCPM 一键部署脚本
# 支持多种推理框架：HuggingFace, vLLM, SGLang, CPM.cu, InfLLM-V2
# ============================================================================

set -e  # 遇到错误立即退出

TORCH_INDEX_URL="${MINICPM_TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu121}"
TORCH_VERSION="${MINICPM_TORCH_VERSION:-2.2.2}"
TORCHVISION_VERSION="${MINICPM_TORCHVISION_VERSION:-0.17.2}"
TORCHAUDIO_VERSION="${MINICPM_TORCHAUDIO_VERSION:-2.2.2}"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

detect_gpu() {
    log_step "检测 NVIDIA GPU"
    if ! command -v nvidia-smi &>/dev/null; then
        log_warn "未找到 nvidia-smi，跳过 GPU 信息检测"
        return
    fi
    local gpu_info
    gpu_info=$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader | head -n 1)
    log_info "GPU 信息: ${gpu_info}"

    local gpu_mem
    gpu_mem=$(echo "$gpu_info" | awk -F',' '{print $2}' | tr -dc '0-9')
    if [[ -n "$gpu_mem" && "$gpu_mem" -lt 12000 ]]; then
        log_warn "显存低于 12GB，建议优先使用 MiniCPM4-0.5B 模型"
    fi
}

# 检查conda环境
check_conda() {
    if ! command -v conda &> /dev/null; then
        log_error "Conda未安装，请先安装Miniconda或Anaconda"
        exit 1
    fi
    log_info "Conda已安装"
}

# 激活conda环境
activate_conda_env() {
    local env_name=$1
    log_step "激活conda环境: $env_name"
    
    # 初始化conda
    eval "$(conda shell.bash hook)"
    
    # 检查环境是否存在
    if conda env list | grep -q "^$env_name "; then
        log_info "环境 $env_name 已存在，正在激活..."
        conda activate $env_name
    else
        log_warn "环境 $env_name 不存在，正在创建..."
        conda create -n $env_name python=3.10 -y
        conda activate $env_name
    fi
    
    log_info "当前环境: $(conda info --envs | grep '*' | awk '{print $1}')"
}

# 安装基础依赖
install_base_requirements() {
    log_step "安装基础依赖包..."

    pip install --upgrade pip setuptools wheel
    
    if [ -f "requirements.txt" ]; then
        log_info "从 requirements.txt 安装依赖..."
        pip install -r requirements.txt
    else
        log_warn "未找到 requirements.txt，手动安装核心依赖..."
        pip install transformers>=4.36.2 gradio>=4.26.0 \
                    openai>=1.17.1 tiktoken>=0.6.0 loguru>=0.7.2 \
                    sentence_transformers>=2.6.1 sse_starlette>=2.1.0 \
                    Pillow>=10.3.0 timm>=0.9.16 sentencepiece>=0.2.0
    fi

    log_info "安装 PyTorch ${TORCH_VERSION} (CUDA 12.1) 及配套组件"
    pip install --index-url "${TORCH_INDEX_URL}" \
        "torch==${TORCH_VERSION}" \
        "torchvision==${TORCHVISION_VERSION}" \
        "torchaudio==${TORCHAUDIO_VERSION}"
    
    # 安装accelerate（运行demo必需）
    log_info "安装accelerate依赖..."
    pip install accelerate
    
    # 安装modelscope（用于下载模型）
    log_info "安装modelscope依赖..."
    pip install modelscope
    
    log_info "基础依赖安装完成"
}

# 安装InfLLM-V2稀疏注意力支持
install_infllm_v2() {
    log_step "安装InfLLM-V2稀疏注意力支持..."
    
    local install_dir="./infllmv2_cuda_impl"
    
    if [ -d "$install_dir" ]; then
        log_warn "InfLLM-V2目录已存在，跳过克隆"
    else
        log_info "克隆InfLLM-V2仓库..."
        # 先尝试feature_infer分支，如果失败则克隆主分支
        if ! git clone -b feature_infer https://github.com/OpenBMB/infllmv2_cuda_impl.git 2>/dev/null; then
            log_warn "feature_infer分支不存在，尝试克隆主分支..."
            if ! git clone https://github.com/OpenBMB/infllmv2_cuda_impl.git 2>/dev/null; then
                log_error "克隆InfLLM-V2仓库失败，请检查网络连接或仓库是否存在"
                log_warn "跳过InfLLM-V2安装，您可以稍后手动安装"
                return 1
            fi
        fi
    fi
    
    cd infllmv2_cuda_impl
    log_info "初始化子模块..."
    if ! git submodule update --init --recursive; then
        log_warn "子模块初始化失败，继续尝试安装..."
    fi
    
    log_info "安装InfLLM-V2..."
    if pip install -e .; then
        log_info "InfLLM-V2安装完成"
    else
        log_error "InfLLM-V2安装失败"
        cd ..
        return 1
    fi
    
    cd ..
}

# 安装vLLM (标准版本)
install_vllm_standard() {
    log_step "安装vLLM标准版本..."
    
    pip install -U vllm --pre --extra-index-url https://wheels.vllm.ai/nightly
    
    log_info "vLLM标准版本安装完成"
}

# 安装vLLM (EAGLE3推测解码版本)
install_vllm_eagle3() {
    log_step "安装vLLM EAGLE3推测解码版本..."
    
    local install_dir="./vllm"
    
    if [ -d "$install_dir" ]; then
        log_warn "vLLM目录已存在，跳过克隆"
    else
        log_info "克隆vLLM EAGLE3仓库..."
        git clone https://github.com/LDLINGLINGLING/vllm.git
    fi
    
    cd vllm
    log_info "安装vLLM EAGLE3..."
    pip install -e .
    
    cd ..
    log_info "vLLM EAGLE3版本安装完成"
}

# 安装SGLang (标准版本)
install_sglang_standard() {
    log_step "安装SGLang标准版本..."
    
    local install_dir="./sglang"
    
    if [ -d "$install_dir" ]; then
        log_warn "SGLang目录已存在，跳过克隆"
    else
        log_info "克隆SGLang仓库..."
        git clone -b openbmb https://github.com/OpenBMB/sglang.git
    fi
    
    cd sglang
    log_info "升级pip..."
    pip install --upgrade pip
    
    log_info "安装SGLang..."
    pip install -e "python[all]"
    
    cd ..
    log_info "SGLang标准版本安装完成"
}

# 安装SGLang (EAGLE3推测解码版本)
install_sglang_eagle3() {
    log_step "安装SGLang EAGLE3推测解码版本..."
    
    local install_dir="./sglang_eagle3"
    
    if [ -d "$install_dir" ]; then
        log_warn "SGLang EAGLE3目录已存在，跳过克隆"
    else
        log_info "克隆SGLang EAGLE3仓库..."
        git clone https://github.com/LDLINGLINGLING/sglang.git sglang_eagle3
    fi
    
    cd sglang_eagle3
    log_info "安装SGLang EAGLE3..."
    pip install -e .
    
    cd ..
    log_info "SGLang EAGLE3版本安装完成"
}

# 安装CPM.cu
install_cpm_cu() {
    log_step "安装CPM.cu推理框架..."
    
    local install_dir="./CPM.cu"
    
    if [ -d "$install_dir" ]; then
        log_warn "CPM.cu目录已存在，跳过克隆"
    else
        log_info "克隆CPM.cu仓库..."
        git clone https://github.com/OpenBMB/CPM.cu.git --recursive
    fi
    
    cd CPM.cu
    log_info "安装CPM.cu..."
    python3 setup.py install
    
    cd ..
    log_info "CPM.cu安装完成"
}

# 下载MiniCPM模型
download_minicpm_model() {
    log_step "下载MiniCPM模型..."
    
    echo ""
    echo "请选择要下载的模型："
    echo "  1) MiniCPM4-0.5B (推荐，轻量级版本) ⭐"
    echo "  2) MiniCPM4.1-8B (最新版本)"
    echo "  3) MiniCPM4-8B"
    echo "  4) MiniCPM3-4B"
    echo "  0) 跳过模型下载"
    echo -n "请选择 [0-4] (默认1): "
    read model_choice
    
    # 默认选择0.5B模型
    if [ -z "$model_choice" ]; then
        model_choice=1
    fi
    
    local model_name=""
    case $model_choice in
        1)
            model_name="OpenBMB/MiniCPM4-0.5B"
            ;;
        2)
            model_name="OpenBMB/MiniCPM4.1-8B"
            ;;
        3)
            model_name="OpenBMB/MiniCPM4-8B"
            ;;
        4)
            model_name="OpenBMB/MiniCPM3-4B"
            ;;
        0)
            log_info "跳过模型下载"
            return 0
            ;;
        *)
            log_error "无效选项"
            return 1
            ;;
    esac
    
    log_info "正在下载模型: $model_name"
    
    # 创建下载脚本
    cat > /tmp/modelscope_download.py << EOF
from modelscope import snapshot_download
import sys

try:
    model_dir = snapshot_download('$model_name', cache_dir='./', revision='master')
    print(f"模型下载成功，保存路径: {model_dir}")
except Exception as e:
    print(f"模型下载失败: {e}")
    sys.exit(1)
EOF
    
    # 执行下载
    if python /tmp/modelscope_download.py; then
        log_info "模型下载完成"
        rm /tmp/modelscope_download.py
    else
        log_error "模型下载失败"
        rm /tmp/modelscope_download.py
        return 1
    fi
}
install_finetune_requirements() {
    log_step "安装微调相关依赖..."
    
    if [ -f "finetune/requirements.txt" ]; then
        log_info "从 finetune/requirements.txt 安装依赖..."
        pip install -r finetune/requirements.txt
    else
        log_warn "未找到 finetune/requirements.txt，手动安装核心依赖..."
        pip install jieba>=0.42.1 ruamel_yaml>=0.18.5 rouge_chinese>=1.0.3 \
                    jupyter>=1.0.0 datasets>=2.16.1 peft>=0.7.1 \
                    deepspeed>=0.13.1 flash_attn>=2.5.1
    fi
    
    log_info "微调依赖安装完成"
}

auto_install_for_4080() {
    log_info "检测到 --auto-4080 选项，开始一键部署 (RTX 4080 12GB)"
    detect_gpu
    install_base_requirements
    install_vllm_standard
    install_cpm_cu
    log_info "自动安装完成。默认未下载模型，请运行脚本后选择选项 9 获取所需模型。"
}

# 显示菜单
show_menu() {
    echo ""
    echo "======================================================"
    echo "           MiniCPM 环境部署脚本"
    echo "======================================================"
    echo "请选择要安装的组件："
    echo ""
    echo "基础组件："
    echo "  1) 基础依赖 (HuggingFace Transformers + 核心包)"
    echo "  2) InfLLM-V2 稀疏注意力支持"
    echo ""
    echo "推理框架："
    echo "  3) vLLM 标准版"
    echo "  4) vLLM EAGLE3 推测解码版"
    echo "  5) SGLang 标准版"
    echo "  6) SGLang EAGLE3 推测解码版"
    echo "  7) CPM.cu 推理框架 (推荐，最高性能)"
    echo ""
    echo "高级功能："
    echo "  8) 微调依赖 (DeepSpeed + PEFT + Flash Attention)"
    echo "  9) 下载MiniCPM模型 (使用ModelScope)"
    echo ""
    echo "一键安装："
    echo " 10) 完整安装 (基础+InfLLM-V2+CPM.cu)"
    echo " 11) 推理全套 (基础+InfLLM-V2+vLLM+SGLang+CPM.cu)"
    echo " 12) vLLM专用 (基础+vLLM标准版，跳过InfLLM-V2)"
    echo ""
    echo "  0) 退出"
    echo "====================================================="
    echo -n "请输入选项 [0-12]: "
}

# 主函数
main() {
    local mode="${1:-}"
    log_info "欢迎使用MiniCPM一键部署脚本"
    log_info "脚本位置: $(pwd)"
    
    # 检查conda
    check_conda
    
    # 激活环境
    activate_conda_env "minicpm"

    if [[ "${mode}" == "--auto-4080" || "${AUTO_DEPLOY_4080:-}" == "1" ]]; then
        auto_install_for_4080
        return
    fi
    
    # 显示菜单并处理选择
    while true; do
        show_menu
        read choice
        
        case $choice in
            1)
                install_base_requirements
                ;;
            2)
                install_infllm_v2
                ;;
            3)
                install_vllm_standard
                ;;
            4)
                install_vllm_eagle3
                ;;
            5)
                install_sglang_standard
                ;;
            6)
                install_sglang_eagle3
                ;;
            7)
                install_cpm_cu
                ;;
            8)
                install_finetune_requirements
                ;;
            9)
                download_minicpm_model
                ;;
            10)
                log_info "开始完整安装..."
                install_base_requirements
                install_infllm_v2
                install_cpm_cu
                log_info "完整安装完成！"
                ;;
            11)
                log_info "开始推理全套安装..."
                install_base_requirements
                install_infllm_v2
                install_vllm_standard
                install_sglang_standard
                install_cpm_cu
                log_info "推理全套安装完成！"
                ;;
            12)
                log_info "开始vLLM专用安装..."
                install_base_requirements
                log_info "跳过InfLLM-V2（vLLM不需要稀疏注意力支持）"
                install_vllm_standard
                log_info "vLLM专用安装完成！"
                ;;
            0)
                log_info "退出安装"
                break
                ;;
            *)
                log_error "无效选项，请重新选择"
                ;;
        esac
        
        echo ""
        log_info "按回车键继续..."
        read
    done
    
    echo ""
    log_info "======================================================"
    log_info "部署完成！"
    log_info "======================================================"
    log_info "环境名称: minicpm"
    log_info "激活命令: conda activate minicpm"
    log_info ""
    log_info "快速开始："
    log_info "  1. 下载模型: 运行脚本选择选项9（推荐MiniCPM4-0.5B）"
    log_info "  2. HuggingFace推理: python demo/minicpm/hf_based_demo.py"
    log_info "  3. vLLM推理: vllm serve openbmb/MiniCPM4-0.5B --trust-remote-code"
    log_info "  4. CPM.cu推理: 参考CPM.cu目录下的文档"
    log_info ""
    log_info "注意事项："
    log_info "  - 运行demo前需要先下载模型（选项9，推荐0.5B轻量级版本）"
    log_info "  - vLLM推理建议选择选项12（vLLM专用安装）"
    log_info "  - 0.5B模型适合快速测试和资源受限环境"
    log_info "  - 处理长文本(>64K)需要安装InfLLM-V2（选项2）"
    log_info "======================================================"
}

# 运行主函数
main "$@"
