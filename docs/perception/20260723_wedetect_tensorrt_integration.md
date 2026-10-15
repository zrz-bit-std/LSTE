# WeDetect TensorRT Integration

## Scope

This document records the WeDetect-Base TensorRT FP16 integration for LSTE.
The active detector remains WeDetect-Base. No fallback to GroundingDINO is used
by the default configuration.

## Current Architecture

The detector keeps the existing LSTE ROS contract unchanged:

```text
/kinect/hd/image_color_rect
  -> lste_det_node.py
  -> /lste/detections
  -> score/state/goal/visualization/controller
```

The WeDetect implementation is split into two towers:

1. The XLM-RoBERTa language tower stays in PyTorch. English LSTE labels are
   translated to Chinese prompts and embeddings are cached until the prompt set
   changes.
2. The ConvNeXt-B vision tower, detection head and DFL decode run through
   ONNX Runtime's TensorRT FP16 provider.
3. Score filtering and class-aware NMS run on CUDA. Only the final detections
   are copied back to CPU and converted to the original LSTE normalized boxes.

This preserves the original English labels in `LsteDetections`, so the Goal
Manager, scoring, RViz overlay, global goal logic and controller interfaces do
not need changes.

## Files

| Path | Responsibility |
| --- | --- |
| `src/lste_core/scripts/utils/wedetect_backend.py` | WeDetect adapter, text cache, TensorRT session, CUDA NMS and ROS-facing output conversion. |
| `src/lste_core/scripts/lste_det_node.py` | Selects the backend and reads WeDetect/TensorRT ROS parameters. |
| `scripts/config/pipeline_defaults.yaml` | Default WeDetect TensorRT configuration. |
| `scripts/lifecycle/run_nodes_tmux.sh` | Activates `dino`, exposes TensorRT shared libraries and forwards configuration. |
| `scripts/lifecycle/run_pipeline_tmux.sh` | Same forwarding for the combined pipeline launcher. |
| `model/WeDetect/deploy/onnx_models/` | Exported dual-tower ONNX artifacts, ignored by Git with the model source. |
| `model/WeDetect/deploy/trt_cache/` | Machine-specific TensorRT engine cache, ignored by Git. |

## Installed Runtime

The packages are installed in the existing `dino` Conda environment:

| Component | Version |
| --- | --- |
| Python | 3.9.25 |
| PyTorch | 2.5.1+cu121 |
| ONNX | 1.16.2 |
| ONNX Runtime GPU | 1.19.2 |
| TensorRT CUDA 12 | 10.16.1.11 |
| NumPy | 1.26.4 |

The host GPU is an RTX 3060 12 GB (`sm86`), driver `560.35.03`. TensorRT
libraries are added to `LD_LIBRARY_PATH` only in the detector tmux command.
The TensorRT runtime installation consumes several GB on disk; at the time of
integration, regenerable pip/ROS/editor caches were cleaned before installation.

## Engine Cache And Prompt Capacity

The exported vision ONNX has a dynamic text-class dimension. Letting TensorRT
build an engine for every prompt count causes a costly build when task prompts
change. The adapter therefore pads text features to a fixed capacity:

```yaml
WDETECT_TRT_MAX_CLASSES: 16
```

Only the first `K` score columns, where `K` is the actual prompt count, are
kept for NMS. Padded columns never become LSTE labels. The current `blue_cup`
task uses 10 classes and fits this engine.

The resulting RTX 3060 FP16 engine is approximately 227 MB. It is reused for
all tasks with at most 16 prompt classes. A rebuild is required only when one
of these changes:

- the cache is absent or deleted;
- `WDETECT_TRT_MAX_CLASSES` is increased;
- the ONNX graph, WeDetect checkpoint, input resolution or FP16 setting changes;
- the GPU architecture changes;
- CUDA, TensorRT, ONNX Runtime or NVIDIA driver changes incompatibly.

Changing prompt text alone does not require a rebuild when the class count is
within the fixed capacity.

The first 16-class build took roughly ten minutes while the system was running.
After nonessential tmux windows were stopped to free CPU/GPU, the engine was
written successfully. Subsequent detector restarts loaded the cached engine in
about five seconds without modifying the cache timestamp.

## Default Configuration

`scripts/config/pipeline_defaults.yaml` contains:

```yaml
DETECTOR: wedetect-large
WDETECT_TRT_MAX_CLASSES: 16
WDETECT_SPIN_HZ: 10.0
WDETECT_MIN_INTERVAL: 0.10
```

`DETECTOR` is the only model-selection key: use `wedetect-base`,
`wedetect-large`, or `groundingdino`. The launcher selects the matching
checkpoint, language model, ONNX file, and TensorRT cache automatically.

The normal startup command remains `runall` (or
`./scripts/bin/runall`). It starts detached and keeps keyboard teleoperation as
the initial controller. No extra TensorRT command-line arguments are required.

`WDETECT_SPIN_HZ` was added because the detector loop had a hard-coded 10 Hz
limit. Its default remains 10 Hz for normal operation. It is separate from the
state-specific inference intervals.

## Measurements

All figures below use WeDetect-Base, the 640x640 vision input, 10 active LSTE
prompt classes padded to 16, and the RTX 3060.

| Measurement | Result | Notes |
| --- | ---: | --- |
| ONNX Runtime CUDA vision tower | 17.26 FPS | Before TensorRT, cached text embeddings. |
| TensorRT FP16 vision tower | 49.51 FPS | With the active LSTE system running. |
| TensorRT full `detect()` before CUDA NMS | 8.00 FPS | CPU score filtering/NMS was the bottleneck. |
| TensorRT full `detect()` after CUDA NMS | 18.28 FPS | Includes preprocessing, NMS and normalized LSTE output. |
| `/lste/detections` maximum, wall-clock | 17.17 FPS | `spin_hz=60`, all inference intervals set to zero for the test. |
| Normal `/lste/detections` rate | about 10 FPS | Intentional default: `0.10 s` interval and 10 Hz spin. |
| `/lste/det_vis_image` rate | about 6 FPS | Bound by visualization and CSRT tracker updates. |

`rostopic hz` under Gazebo simulation reported roughly 32 Hz during the maximum
test. That uses simulated ROS time, which ran faster than wall-clock time. The
17.17 FPS wall-clock subscriber measurement is the valid real-time limit.

## Live Verification

The full pipeline was started with `runall` after integration. Verification
confirmed:

- `lste-env`, `lste` and `lste-teleop` sessions were running;
- TensorRT cache loaded without recompilation;
- `/lste/detections` published environment detections for chair, desk and
  computer monitor labels;
- `/lste/det_vis_image` carried a non-black annotated camera image with boxes;
- keyboard/teleop remained the initial controller.

The current scene did not point the camera at the blue mug during the test, so
the absence of a target box in that view is expected and is not a TensorRT
failure.

## Operational Notes

- First engine builds are expensive. Do not interrupt a build merely because
  it does not expose a percentage: TensorRT/ONNX Runtime provides no reliable
  build-progress API. CPU/GPU activity and the creation of the `.engine` file
  are the available health signals.
- When a build must be done on a busy machine, temporarily stopping Gazebo,
  RViz and nonessential LSTE windows gives the builder more CPU/GPU headroom.
- Keep normal operation at the configured 10 Hz. Raising `WDETECT_SPIN_HZ` and
  setting intervals to zero is useful for benchmarking, but it increases GPU
  load and can starve visualization or simulation.
