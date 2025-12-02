#!/bin/bash

echo "=========================================="
echo "快速修复 CUDA 扩展"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GROUNDINGDINO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 激活环境
source ~/anaconda3/etc/profile.d/conda.sh
conda activate dino
export CUDA_HOME=/usr

echo "步骤 1: 卸载错误的 PyTorch 版本"
pip uninstall -y torch torchvision torchaudio
echo ""

echo "步骤 2: 安装正确的 PyTorch 版本 (1.8.1+cu101)"
echo "使用 PyTorch 官方源..."
pip install torch==1.8.1+cu101 torchvision==0.9.1+cu101 -f https://download.pytorch.org/whl/torch_stable.html

if [ $? -ne 0 ]; then
    echo "尝试备用版本 1.7.1+cu101..."
    pip install torch==1.7.1+cu101 torchvision==0.8.2+cu101 -f https://download.pytorch.org/whl/torch_stable.html
fi
echo ""

echo "步骤 3: 验证 PyTorch 安装"
python -c "import torch; print('PyTorch:', torch.__version__); print('CUDA 可用:', torch.cuda.is_available())"
echo ""

echo "步骤 4: 清理旧的编译文件"
cd "${GROUNDINGDINO_ROOT}"
rm -rf build/ dist/ *.egg-info groundingdino.egg-info/
find . -type f -name "*.so" -delete 2>/dev/null
echo "清理完成"
echo ""

echo "步骤 5: 重新编译 CUDA 扩展"
echo "这需要几分钟，请耐心等待..."
pip install -e . --no-build-isolation --no-deps
echo ""

echo "步骤 6: 安装其他依赖"
pip install transformers addict yapf timm "numpy<2.0" opencv-python pycocotools supervision
echo ""

echo "步骤 7: 验证 CUDA 扩展"
python -c "
import sys
try:
    import groundingdino._C as _C
    print('✅ CUDA 扩展加载成功！')
    print('可用函数:', [x for x in dir(_C) if not x.startswith('_')][:5])
    sys.exit(0)
except Exception as e:
    print('❌ CUDA 扩展加载失败:', e)
    sys.exit(1)
"

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "🎉 修复成功！可以运行 python test.py 了"
    echo "=========================================="
else
    echo ""
    echo "=========================================="
    echo "修复失败，请查看上方错误信息"
    echo "=========================================="
    echo ""
    echo "可能的原因："
    echo "1. GCC 版本过低 (需要 >= 7)"
    echo "2. CUDA_HOME 未正确设置"
    echo "3. 编译过程中出现错误"
    echo ""
    echo "请检查编译输出中的错误信息"
fi
