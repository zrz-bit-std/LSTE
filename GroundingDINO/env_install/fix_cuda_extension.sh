#!/bin/bash

echo "=========================================="
echo "修复 CUDA 扩展问题"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GROUNDINGDINO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 激活环境
source ~/anaconda3/etc/profile.d/conda.sh
conda activate dino

echo "当前 PyTorch 版本："
python -c "import torch; print(torch.__version__)"
echo ""

echo "系统 CUDA 版本："
nvcc --version | grep "release"
echo ""

echo "步骤 1: 卸载当前的 PyTorch"
pip uninstall -y torch torchvision torchaudio
echo ""

echo "步骤 2: 安装与 CUDA 10.1 匹配的 PyTorch 1.10.0"
pip install torch==1.10.0+cu101 torchvision==0.11.0+cu101 -f https://download.pytorch.org/whl/torch_stable.html
echo ""

echo "步骤 3: 验证 PyTorch 安装"
python -c "import torch; print('PyTorch 版本:', torch.__version__); print('CUDA 可用:', torch.cuda.is_available())"
echo ""

echo "步骤 4: 设置 CUDA_HOME"
export CUDA_HOME=/usr
echo "CUDA_HOME=$CUDA_HOME"
echo ""

echo "步骤 5: 清理之前的编译文件"
cd "${GROUNDINGDINO_ROOT}"
rm -rf build/ dist/ *.egg-info
find . -name "*.so" -delete
echo "清理完成"
echo ""

echo "步骤 6: 重新安装 GroundingDINO（这会重新编译 CUDA 扩展）"
echo "这可能需要几分钟，请耐心等待..."
pip install -e .
echo ""

echo "步骤 7: 验证 CUDA 扩展是否成功加载"
python -c "
try:
    from groundingdino.models.GroundingDINO import ms_deform_attn
    print('✅ CUDA 扩展加载成功！')
    import groundingdino._C as _C
    print('✅ _C 模块可用')
except Exception as e:
    print('❌ CUDA 扩展加载失败:', e)
"
echo ""

echo "=========================================="
echo "修复完成！"
echo "=========================================="
echo ""
echo "现在可以重新运行 test.py"
