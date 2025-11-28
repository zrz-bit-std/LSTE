#!/bin/bash
# Grounding DINO 环境设置脚本
# 使用方法: source setup_env.sh

# 激活 conda 环境
source ~/anaconda3/etc/profile.d/conda.sh
conda activate dino

# 设置 CUDA_HOME
export CUDA_HOME=/usr

# 设置 LD_LIBRARY_PATH（关键！防止 libc10.so 找不到）
export LD_LIBRARY_PATH=/home/zrz/anaconda3/envs/dino/lib/python3.9/site-packages/torch/lib:$LD_LIBRARY_PATH

echo "✅ 环境已配置"
echo "   - Conda 环境: dino"
echo "   - CUDA_HOME: $CUDA_HOME"
echo "   - PyTorch 库路径已添加到 LD_LIBRARY_PATH"
echo ""
echo "现在可以运行: python test.py"
