# 开放词汇检测器调研与选型

更新日期：2026-07-21

## 目标和当前基线

本项目需要在 Gazebo/RViz、SA-PPO 和 ROS 节点同时运行时，按自然语言类别检测目标（当前重点为远距离 `blue mug`），并将结果提供给导航链路。任务 JSON 和类别集合相对固定；MiniCPM 已会将类别 prompt 缓存到 `runtime/prompt_cache/`，不应在每帧调用 MiniCPM。

当前 GroundingDINO 对每张图像分别执行目标和环境两次前向推理。RTX 3060（12 GB）全链路实测：启用 CUDA 扩展、不限速时约 **0.570 FPS**；生产调度为 1.5 秒一次。详见 [20260723_visual_pipeline.md](../perception/20260723_visual_pipeline.md)。它保留为效果基线和回退方案，不适合作为实时主检测器。

选型约束：代码与可本地运行的权重必须公开；无需从零训练；ROS 支持不是前提；优先保证远距离目标召回，其次才是极高 FPS。系统当前已有约 2.43 GB GPU 占用；替换 GroundingDINO 后可释放其对应显存。

## 公共基准候选

下表是论文或官方仓库给出的结果，**不是本机实测**。FPS 的 GPU、后端、分辨率和计时边界不同，不能直接当作 RTX 3060 上的 ROS 端到端 FPS。`Fixed AP` 只有在同一 LVIS protocol 下才可直接比较；普通 `AP` 与 `Fixed AP` 不应混为同一排序指标。

| 模型 | 公开输入/后端 | 公开速度 | 公开效果 | 代码与权重 | 初步判断 |
| --- | --- | ---: | --- | --- | --- |
| GroundingDINO（当前） | 当前 ROS 两次 PyTorch 前向 | 本机 0.570 FPS | 未在当前场景量化 | 已接入 | 只作基线/回退 |
| YOLOE-v8-S | T4 + TensorRT | 305.8 FPS | LVIS Fixed AP 27.9 | 公开 | 极快，但公开精度较低 |
| YOLOE-26N | 官方模型 | 未找到可比 FPS | N 小型版本 | 公开 | 极端速度档 |
| YOLOE-26S | 官方模型 | 未找到可比 FPS | AP 29.9 | 公开，约 31 MB 权重 | 接入最方便的基线，不是效果首选 |
| OV-DEIM-S | 640，T4 + TensorRT | 161 FPS | Fixed AP 29.6 | 公开 | 低显存高帧率 |
| OV-DEIM-M | 640，T4 + TensorRT | 109 FPS | Fixed AP 32.6 | 公开 | 高帧率/效果/显存的优秀平衡 |
| OV-DEIM-L | 640，T4 + TensorRT | 91 FPS | Fixed AP 35.9 | 公开 | OV-DEIM 内的效果优先档 |
| WeDetect-Tiny | 官方报告 | 62.5 FPS | Fixed AP 37.4 | 公开，含 ONNX | 高帧率下效果最强候选之一 |
| WeDetect-Base | 官方报告 | 35.1 FPS | Fixed AP 47.3 | 公开，含 ONNX | 当前已公开数字中效果首选 |
| WeDetect-Large | 官方权重 | 未公开可比数据 | 未公开可比数据 | 公开 | 可测试的效果上限，不应先假定一定优于 Base |
| YOLO-UniOW-S | V100 | 98.3 FPS | AP 26.2 | 公开 | 不如 OV-DEIM-M / WeDetect 有竞争力 |
| OmDet-Turbo-Tiny | PyTorch / TensorRT | 21.5 / 140 FPS | LVIS AP 30.3 | 公开 | TensorRT 才有明显速度优势 |

### 各系列说明

**WeDetect** 有 `Tiny`、`Base`、`Large` 三个直接检测器，没有官方 `WeDetect-Super`。另有 `Base-Uni`/`Large-Uni`，它们生成通用目标 proposal，并非直接根据 `blue mug` 输出类别；`WeDetect-Ref 2B/4B` 是复杂指代表达式的二阶段分类器，延迟和显存不适合当前实时控制环路。WeDetect 使用 XLM-RoBERTa 文本编码器，仓库示例偏中文类别。应在启动时同时编码中英文固定同义词，例如 `blue mug`、`蓝色马克杯`、`蓝色杯子`，将文本特征缓存后再处理相机帧。

**OV-DEIM** 的表中所有 FPS 都是 640 分辨率、T4 TensorRT 的官方数据；S/M/L 参数量分别为 11M、20M、36M。它的 DETR 式推理对大量类别的后处理更友好。代码为 Apache-2.0；使用公开 checkpoint 前应再次核对权重的 CC BY-NC 4.0 条款是否满足最终用途。

**YOLOE** 使用 `model.set_classes([...])`，可以在启动时一次性设置缓存类别，并对每个图像执行一次前向推理，工程接入成本很低。仓库为 AGPL-3.0；若将来分发闭源产品，需要评估许可证或商业许可。

**YOLO-UniOW** 为 GPL-3.0。**OmDet-Turbo** 采用 Apache-2.0。许可证只在实际分发或商用时成为约束；当前研究和本地实验仍应保留这些信息。

## RTX 3060 显存评估

下面是 640、batch=1、单路推理的保守工程估算，不是作者公布的峰值，也不是训练显存。实际 `nvidia-smi` 峰值会受 PyTorch CUDA allocator、输入尺寸和 ROS 并发影响。

| 模型 | 已知规模 | 估计推理显存 | 3060（12 GB）判断 |
| --- | ---: | ---: | --- |
| OV-DEIM-S | 11M 参数 | 0.7-1.2 GB | 很宽裕 |
| OV-DEIM-M | 20M 参数 | 1.0-1.8 GB | 很宽裕 |
| OV-DEIM-L | 36M 参数 | 1.5-2.5 GB | 宽裕 |
| WeDetect-Tiny | checkpoint 约 1.28 GB | 2.5-4.0 GB | 可运行，余量充足 |
| WeDetect-Base | checkpoint 约 1.59 GB | 3.5-5.5 GB | 可运行；应使用 FP16 与文本缓存 |
| WeDetect-Large | checkpoint 约 3.20 GB | 约 6-9 GB（待实测） | 可作为单独实验；与全部节点并发时余量偏紧 |

这些模型均可直接使用官方 zero-shot 权重，**不需要自己训练**。只有实际评测证明 `blue mug` 持续漏检时，才考虑用 Gazebo 或实机少量标注图进行微调；不应在 zero-shot 评测之前启动训练工作。

## 选型结论

综合当前目标，默认方案不是纯粹追求论文峰值 FPS，而是在至少 8-10 FPS 的 ROS 端到端更新率下，尽量提高远距离杯子的召回率：

1. **WeDetect-Base**：首个完整接入和主要候选。其公开 Fixed AP 47.3 明显最高，35.1 FPS 的公开速度也远高于控制闭环所需。运行配置应为 640、batch=1、FP16、启动时缓存所有类别的文本特征。
2. **WeDetect-Tiny**：Base 在本机实际链路无法稳定达到 8-10 FPS 或显存压力过大时的首选降级版。
3. **OV-DEIM-M**：对 GPU 负载最敏感时的平衡方案；相比 Tiny 公开精度较低，但速度和显存表现优秀。
4. **WeDetect-Large**：效果上限实验。它有更大权重，但没有可直接比较的公开速度和 Fixed AP，必须实测后再决定，不应以“大”推断为默认版本。
5. **GroundingDINO / YOLOE-26S**：分别保留作现有效果基线与快速接入基线。

## 推荐实验设计

在相同 Gazebo 场景、相同相机图像、RViz 和 SA-PPO 都运行的条件下，先测试 WeDetect-Base、WeDetect-Tiny、OV-DEIM-M。每个模型使用同一组固定类别，并在启动后缓存文本特征。记录：

| 指标 | 判定方式 |
| --- | --- |
| 端到端 FPS / P95 延迟 | 从 ROS 图像时间戳到检测结果发布，不只测模型 `forward` |
| 显存与 GPU 利用率 | 稳态平均值和峰值，检查是否影响 Gazebo/RViz |
| 远距离 `blue mug` 召回率 | 不同距离、视角、遮挡下是否检测到 |
| 误检率 | 桌椅、显示器、文件夹被误报成目标的比例 |
| 控制稳定性 | 目标框短暂丢失时是否影响 SA-PPO 接近目标 |

通过条件：Base 若稳定达到 8-10 FPS 且 `blue mug` 召回优于其他候选，即作为主检测器；否则按 Tiny，再按 OV-DEIM-M 的顺序降级。不要在未完成同场景验证前直接删除 GroundingDINO。

## WeDetect-Base 接入实测（2026-07-21）

已完成可切换后端接入。当前统一配置键是 `DETECTOR`；将
`scripts/config/pipeline_defaults.yaml` 设为 `wedetect-base`、`wedetect-large` 或
`groundingdino` 即可选择对应独立节点。WeDetect 把 `prompt_a` 和 `prompt_b_terms` 合并为单次视觉前向，再按原始
英文标签拆分回 `target_dets` 和 `env_dets`，因此 Goal Manager、评分、可视化与 SA-PPO
接口均不变。中文文本向量只在类别集合变化时重建。

在 RTX 3060、现有 `dino` 环境、官方 `assets/demo.jpeg`、640、FP16、10 个固定类别下：

| 指标 | 实测 |
| --- | ---: |
| 模型加载 | 5.282 s |
| 首帧（包含文本向量编码） | 0.366 s |
| 稳态模型端单图 | 0.057 s / 17.61 FPS |
| 本进程 PyTorch 峰值显存 | 1.73 GiB |
| 固定文本向量缓存 | 已验证复用 |

该图片本身没有上述类别，得到 0 个框是预期结果。测试时 ROS master 未运行，尚未取得
Gazebo 相机帧；切换到 `wedetect` 后应在完整仿真链路中补测端到端 FPS、远距离 `blue mug`
召回、误检和并发显存，再决定是否替换默认后端。

## 近期但暂不进入接入队列的工作

以下工作在调研时没有满足“可立刻本地验证”的要求：Dynamic-DINO 尚未发布完整代码/权重；VocaDet 仓库接近占位状态；VL-DINO 表示代码仍在整理；DeCo-DETR 报告约 7.4 FPS，但代码/模型计划在论文后发布；HDINO 有代码和权重，但未找到可用于本次排序的可靠公开速度数据。这些可后续跟踪，但不应延误已有权重候选的实测。

## 来源

- GroundingDINO 本机测试：[20260723_visual_pipeline.md](../perception/20260723_visual_pipeline.md)
- YOLOE：<https://github.com/THU-MIG/yoloe>；YOLOE-26 文档：<https://docs.ultralytics.com/models/yoloe/>
- OV-DEIM：<https://github.com/wleilei/OV-DEIM>
- WeDetect：<https://github.com/WeChatCV/WeDetect>；权重：<https://huggingface.co/fushh7/WeDetect>
- YOLO-UniOW：<https://github.com/THU-MIG/YOLO-UniOW>
- OmDet：<https://github.com/om-ai-lab/OmDet>
