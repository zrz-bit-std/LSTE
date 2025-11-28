# LSTE - Large-Scale Vision-Language Model Project

## 项目简介

LSTE是一个集成了视觉检测和大语言模型的多模态AI项目，主要包含两大核心模块：

- **GroundingDINO**: 开放词汇目标检测模型，支持基于文本提示的图像目标检测
- **MiniCPM**: 轻量级大语言模型，支持函数调用、代码解释、多轮对话等功能

## 项目结构

```
LSTE/
├── GroundingDINO/          # 视觉检测模块
│   ├── demo/               # 演示脚本
│   ├── env_install/        # 环境安装脚本
│   ├── groundingdino/      # 核心模型代码
│   └── test/               # 测试代码
├── MiniCPM/                # 大语言模型模块
│   ├── demo/               # 演示脚本
│   ├── finetune/           # 微调脚本
│   ├── quantize/           # 量化脚本
│   └── test/               # 测试代码
└── Data_exchange/          # 数据交换目录
    ├── vlm_prompt/         # VLM提示数据
    ├── run_detection.sh    # 检测运行脚本
    └── start_vllm.sh       # vLLM启动脚本
```

## 核心功能

### GroundingDINO
- ✅ 开放词汇目标检测
- ✅ 支持图文联合输入推理
- ✅ 基于Swin Transformer骨干网络
- ✅ COCO数据集评估
- ✅ Gradio演示界面

### MiniCPM
- ✅ 轻量级大语言模型推理
- ✅ 函数调用 (Function Calling)
- ✅ 代码解释器 (Code Interpreter)
- ✅ 多轮对话
- ✅ Survey生成
- ✅ 支持HF、vLLM、MLX多种推理后端
- ✅ 微调支持（SFT、DPO、LoRA）
- ✅ 模型量化（AWQ、GPTQ、BNB）

## 技术栈

- **深度学习框架**: PyTorch
- **模型库**: Hugging Face Transformers
- **加速**: CUDA, vLLM
- **视觉模型**: Swin Transformer, BERT
- **语言模型**: MiniCPM (基于LLaMA架构)
- **前端**: Vue.js (用于Survey生成)

## 快速开始

### 环境配置

#### GroundingDINO环境

```bash
cd GroundingDINO/env_install
bash setup_env.sh
```

#### MiniCPM环境

```bash
cd MiniCPM
pip install -r requirements.txt
```

### 运行示例

#### 1. GroundingDINO图像检测

```bash
cd GroundingDINO/demo
python inference_on_a_image.py
```

#### 2. 启动MiniCPM vLLM服务

```bash
cd Data_exchange
bash start_vllm.sh
```

#### 3. 运行检测流程

```bash
cd Data_exchange
bash run_detection.sh
```

### Docker部署

项目支持Docker容器化部署：

```bash
cd GroundingDINO
docker build -t lste-grounding-dino .
```

## 模型微调

### LoRA微调

```bash
cd MiniCPM/finetune
bash lora_finetune.sh
```

### SFT全量微调

```bash
cd MiniCPM/finetune
bash sft_finetune.sh
```

## 模型量化

支持多种量化方法以减少模型大小和推理时间：

```bash
cd MiniCPM/quantize

# AWQ量化
python awq_quantize.py

# GPTQ量化
python gptq_quantize.py

# BitsAndBytes量化
python bnb_quantize.py
```

## 数据处理

项目提供了VLM提示数据的处理流程，相关数据存储在 `Data_exchange/vlm_prompt/` 目录下。

## 依赖要求

### 基础依赖
- Python >= 3.8
- PyTorch >= 1.12
- CUDA >= 11.3 (推荐使用GPU)

### GroundingDINO依赖
- transformers
- timm
- gradio
- opencv-python

### MiniCPM依赖
- vllm
- deepspeed
- peft
- bitsandbytes

详细依赖请参考各模块的 `requirements.txt` 文件。

## 配置文件

### GroundingDINO配置
- `GroundingDINO/groundingdino/config/GroundingDINO_SwinB_cfg.py`
- `GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py`

### DeepSpeed配置
- `MiniCPM/finetune/configs/ds_config_zero2.json`
- `MiniCPM/finetune/configs/ds_config_zero3.json`

## 常见问题

### CUDA扩展编译问题

如果遇到CUDA扩展编译问题，请运行：

```bash
cd GroundingDINO/env_install
bash fix_cuda_extension.sh
```

### vLLM服务启动问题

确保已安装vLLM并正确配置CUDA环境：

```bash
pip install vllm
```

## 贡献指南

欢迎提交Issue和Pull Request来改进项目。

## 许可证

本项目遵循各子模块原有的开源许可证。

## 致谢

- [GroundingDINO](https://github.com/IDEA-Research/GroundingDINO)
- [MiniCPM](https://github.com/OpenBMB/MiniCPM)
- OpenBMB团队

## 联系方式

如有问题或建议，请通过GitHub Issues联系。

---

**注意**: 本项目用于研究和学习目的，请遵守相关模型的使用协议。
