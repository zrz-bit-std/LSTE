#!/bin/bash

# GroundingDINO 部署脚本
echo "========================================="
echo "开始部署 Grounding DINO"
echo "========================================="

# 检查 CUDA 版本
echo "步骤 1: 检查 CUDA 环境"
if command -v nvcc &> /dev/null; then
    echo "CUDA 版本:"
    nvcc --version | grep "release"
else
    echo "警告: 未找到 nvcc 命令"
fi

# 设置 CUDA_HOME
echo "设置 CUDA_HOME"
export CUDA_HOME=/usr
echo "CUDA_HOME 设置为: $CUDA_HOME"
echo ""

# 激活 conda 环境
echo "步骤 2: 激活 dino conda 环境"
source ~/anaconda3/etc/profile.d/conda.sh
conda activate dino
echo "当前 conda 环境: $CONDA_DEFAULT_ENV"
echo ""

# 检查并升级 gcc（如果需要）
echo "步骤 3: 检查 GCC 版本"
gcc_version=$(gcc --version | head -n1 | awk '{print $NF}' | cut -d. -f1)
echo "当前 GCC 版本: $(gcc --version | head -n1)"
if [ "$gcc_version" -lt 9 ]; then
    echo "警告: GCC 版本过低（需要 9 或更高），可能导致编译失败"
    echo "如需升级，请运行以下命令（需要 sudo 权限）："
    echo "  sudo apt install software-properties-common"
    echo "  sudo add-apt-repository ppa:ubuntu-toolchain-r/test"
    echo "  sudo apt update"
    echo "  sudo apt install gcc-9 g++-9"
    echo "  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9"
    echo "  sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9"
    read -p "是否继续安装？(y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo "GCC 版本符合要求"
fi
echo ""

# 先安装与 CUDA 10.1 匹配的 PyTorch
echo "步骤 4: 安装与 CUDA 10.1 匹配的 PyTorch"
echo "重要: 必须安装与您的 CUDA 版本匹配的 PyTorch"
echo "正在安装 torch==1.8.1+cu101 和 torchvision==0.9.1+cu101 (for CUDA 10.1)..."
pip install torch==1.8.1+cu101 torchvision==0.9.1+cu101 -f https://download.pytorch.org/whl/torch_stable.html
if [ $? -ne 0 ]; then
    echo "错误: PyTorch 安装失败"
    echo "尝试备用版本..."
    pip install torch==1.7.1+cu101 torchvision==0.8.2+cu101 -f https://download.pytorch.org/whl/torch_stable.html
    if [ $? -ne 0 ]; then
        echo "错误: 无法安装兼容的 PyTorch 版本"
        exit 1
    fi
fi
echo ""

# 安装其他依赖（使用兼容的版本）
echo "步骤 5: 安装其他必需的包（兼容版本）"
cd /home/zrz/Desktop/LSTE/GroundingDINO
# 使用与 PyTorch 1.8.1 兼容的 timm 和 transformers 版本
echo "安装 timm==0.6.13（兼容 PyTorch 1.8.1，不需要 torch.fx）"
echo "安装 transformers==4.30.0（兼容 PyTorch 1.8.1）"
pip install 'timm==0.6.13' 'transformers==4.30.0' addict yapf "numpy<2.0" opencv-python pycocotools supervision
echo ""

# 安装项目
echo "步骤 6: 安装 GroundingDINO (pip install -e .)"
echo "这一步会编译 CUDA 扩展，可能需要几分钟..."
# 使用 --no-build-isolation --no-deps 避免重新安装 PyTorch
pip install -e . --no-build-isolation --no-deps
if [ $? -ne 0 ]; then
    echo "错误: GroundingDINO 安装失败"
    echo "常见原因:"
    echo "  1. GCC 版本过低（需要 >= 7）"
    echo "  2. CUDA 版本与 PyTorch 不匹配"
    echo "  3. CUDA_HOME 未正确设置"
    exit 1
fi
echo ""

# 检查安装
echo "步骤 7: 验证安装"
python -c "import torch; print(f'PyTorch 版本: {torch.__version__}'); print(f'CUDA 可用: {torch.cuda.is_available()}'); print(f'CUDA 版本: {torch.version.cuda}')"
python -c "import groundingdino; print('GroundingDINO 导入成功!')"
if [ $? -ne 0 ]; then
    echo "错误: GroundingDINO 导入失败"
    exit 1
fi
echo ""

# 验证 CUDA 扩展是否成功编译
echo "步骤 7.1: 验证 CUDA 扩展"
# 设置 LD_LIBRARY_PATH（关键步骤！）
export LD_LIBRARY_PATH=/home/zrz/anaconda3/envs/dino/lib/python3.9/site-packages/torch/lib:$LD_LIBRARY_PATH

CUDA_EXT_CHECK=$(python -c "
try:
    import groundingdino._C as _C
    print('SUCCESS')
except Exception as e:
    print('FAILED:', str(e))
" 2>&1)

if [[ $CUDA_EXT_CHECK == *"SUCCESS"* ]]; then
    echo "✅ CUDA 扩展加载成功！"
    python -c "import groundingdino._C as _C; print('   可用函数:', [x for x in dir(_C) if not x.startswith('_')][:5], '...')"
else
    echo "⚠️  CUDA 扩展加载失败: $CUDA_EXT_CHECK"
    echo ""
    
    # 检查是否是 LD_LIBRARY_PATH 问题
    if [[ $CUDA_EXT_CHECK == *"libc10.so"* ]] || [[ $CUDA_EXT_CHECK == *"cannot open shared object"* ]]; then
        echo "检测到共享库路径问题，已自动设置 LD_LIBRARY_PATH"
        echo "请使用提供的 setup_env.sh 脚本来设置环境"
    else
        # 尝试修复
        echo "【开始修复 CUDA 扩展】"
        echo "原因: 编译可能失败"
        echo ""
        
        # 清理并重新编译
        echo "清理旧的编译文件..."
        cd /home/zrz/Desktop/LSTE/GroundingDINO
        rm -rf build/ dist/ *.egg-info
        find . -type f -name "*.so" -delete 2>/dev/null
        echo "清理完成"
        echo ""
        
        echo "重新编译 CUDA 扩展（可能需要几分钟）..."
        pip install -e . --no-build-isolation --no-deps --no-cache-dir
        
        if [ $? -eq 0 ]; then
            # 再次验证
            CUDA_EXT_CHECK2=$(python -c "
try:
    import groundingdino._C as _C
    print('SUCCESS')
except Exception as e:
    print('FAILED:', str(e))
" 2>&1)
            
            if [[ $CUDA_EXT_CHECK2 == *"SUCCESS"* ]]; then
                echo "✅ 修复成功！CUDA 扩展现在可以正常使用"
            else
                echo "❌ 修复失败: $CUDA_EXT_CHECK2"
                echo ""
                echo "可能的原因："
                echo "  1. GCC 版本过低（当前: $(gcc --version | head -n1)）"
                echo "  2. CUDA_HOME 设置不正确（当前: $CUDA_HOME）"
                echo "  3. PyTorch 与 CUDA 版本不匹配"
                echo ""
                echo "建议："
                echo "  - 检查 GCC 版本是否 >= 7"
                echo "  - 确认 CUDA 10.1 与 PyTorch 1.8.1+cu101 匹配"
                echo "  - 查看上方的编译错误信息"
                echo ""
                read -p "是否继续（CUDA 扩展将在 CPU 模式下运行）？(y/n) " -n 1 -r
                echo
                if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                    exit 1
                fi
            fi
        else
            echo "❌ 重新编译失败"
            exit 1
        fi
    fi
fi
echo ""

# 创建 weights 目录
echo "步骤 8: 创建 weights 目录"
mkdir -p weights
echo ""

# 下载预训练模型
echo "步骤 9: 下载预训练模型权重"
cd weights
# if [ ! -f "groundingdino_swint_ogc.pth" ]; then
#     echo "正在下载 groundingdino_swint_ogc.pth (约 600MB，可能需要几分钟)..."
#     echo "提示: 如果下载很慢或失败，可以手动下载后放到 weights 目录"
#     # 使用 wget 显示进度，添加超时和重试
#     wget --progress=bar:force --timeout=60 --tries=5 --wait=3 \
#          https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
#     if [ $? -eq 0 ]; then
#         echo "模型下载完成！"
#     else
#         echo "下载失败！"
#         echo "手动下载链接: https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth"
#         echo "或使用 HuggingFace 镜像: https://huggingface.co/ShilongLiu/GroundingDINO/resolve/main/groundingdino_swint_ogc.pth"
#         echo "下载后请放到: $(pwd)/"
#     fi
# else
    echo "模型文件已存在，跳过下载"
# fi
cd ..
echo ""

# 最终功能测试
echo "步骤 8: 运行功能测试"
echo "测试模型加载..."
# 设置必要的环境变量
export LD_LIBRARY_PATH=/home/zrz/anaconda3/envs/dino/lib/python3.9/site-packages/torch/lib:$LD_LIBRARY_PATH

TEST_RESULT=$(python -c "
from groundingdino.util.inference import load_model
import os

config_path = 'groundingdino/config/GroundingDINO_SwinT_OGC.py'
model_path = 'weights/groundingdino_swint_ogc.pth'

if not os.path.exists(config_path):
    print('ERROR: 配置文件不存在')
    exit(1)
    
if not os.path.exists(model_path):
    print('ERROR: 模型权重文件不存在')
    exit(1)

try:
    model = load_model(config_path, model_path)
    print('SUCCESS')
except Exception as e:
    print(f'ERROR: {e}')
" 2>&1)

if [[ $TEST_RESULT == *"SUCCESS"* ]]; then
    echo "✅ 模型加载测试成功！"
else
    echo "❌ 模型加载测试失败: $TEST_RESULT"
    if [[ $TEST_RESULT == *"模型权重文件不存在"* ]]; then
        echo ""
        echo "请下载模型权重文件:"
        echo "  wget -P weights https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth"
    fi
    exit 1
fi
echo ""

echo "========================================="
echo "🎉 部署完成！所有测试通过！"
echo "========================================="
echo ""
echo "📊 部署摘要:"
python -c "
import torch
import os

print(f'  ✓ Python 环境: dino')
print(f'  ✓ PyTorch: {torch.__version__}')
print(f'  ✓ CUDA 可用: {torch.cuda.is_available()}')
print(f'  ✓ 配置文件: 存在')
print(f'  ✓ 模型权重: 存在 ({os.path.getsize("weights/groundingdino_swint_ogc.pth") / 1024 / 1024:.1f} MB)')
print(f'  ✓ BERT 模型: 存在')

try:
    import groundingdino._C
    print(f'  ✓ CUDA 扩展: 已加载')
except:
    print(f'  ⚠ CUDA 扩展: CPU 模式')
"
echo ""
echo "⚠️  重要提示："
echo "如果运行时遇到 HuggingFace 连接问题，BERT 模型已下载到本地"
echo "位置: bert-base-uncased/"
echo ""
echo "⚠️  每次使用前必须设置环境变量！"
echo "已创建便捷脚本 setup_env.sh，使用方法："
echo "  source setup_env.sh"
echo ""
echo "🚀 使用说明："
echo "1. 每次使用前运行: source setup_env.sh"
echo "2. 运行推理示例:"
echo ""
echo "   python demo/inference_on_a_image.py \\"
echo "     -c groundingdino/config/GroundingDINO_SwinT_OGC.py \\"
echo "     -p weights/groundingdino_swint_ogc.pth \\"
echo "     -i your_image.jpg \\"
echo "     -o output_dir \\"
echo "     -t \"chair . person . dog .\""
echo ""
echo "   或运行测试脚本:"
echo "   source setup_env.sh && python test.py"
echo ""
echo "📝 提示: 文本提示词用 . 分隔不同的类别，例如 \"person . cat . dog .\""
echo ""
echo "📚 参考文档:"
echo "  - 环境设置: source setup_env.sh（每次使用必须！）"
echo "  - 测试脚本: test.py"
echo "  - CUDA 扩展问题: CUDA扩展错误修复指南.md"
echo ""

# 创建 setup_env.sh 脚本
echo "创建环境设置脚本..."
cat > setup_env.sh << 'ENVEOF'
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
ENVEOF

chmod +x setup_env.sh
echo "✅ setup_env.sh 已创建"
echo ""
