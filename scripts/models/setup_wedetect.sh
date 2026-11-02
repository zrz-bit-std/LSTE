#!/usr/bin/env bash
set -euo pipefail

# Install the optional WeDetect source and Base zero-shot checkpoint outside Git.
# The checkpoint is 1.59 GB and requires roughly 2 GB free disk while downloading.
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
SOURCE_DIR="$WS/model/WeDetect"
REPO_URL="https://github.com/WeChatCV/WeDetect.git"
REPO_COMMIT="dd302dba0069ace1b05816bafbc3fa1dbd6aa68c"
MODEL_SIZE="${WDETECT_MODEL_SIZE:-base}"
case "$MODEL_SIZE" in
  tiny)
    CHECKPOINT="$SOURCE_DIR/checkpoints/wedetect_tiny.pth"
    WEIGHT_URL="https://huggingface.co/fushh7/WeDetect/resolve/main/wedetect_tiny.pth"
    WEIGHT_MIRROR_URL="https://hf-mirror.com/fushh7/WeDetect/resolve/main/wedetect_tiny.pth"
    CHECKPOINT_BYTES=1277582698
    ;;
  base)
    CHECKPOINT="$SOURCE_DIR/checkpoints/wedetect_base.pth"
    WEIGHT_URL="https://huggingface.co/fushh7/WeDetect/resolve/main/wedetect_base.pth?download=true"
    WEIGHT_MIRROR_URL="https://hf-mirror.com/fushh7/WeDetect/resolve/main/wedetect_base.pth"
    CHECKPOINT_BYTES=1588423888
    ;;
  *)
    echo "[error] WDETECT_MODEL_SIZE must be tiny or base (got '$MODEL_SIZE')" >&2
    exit 2
    ;;
esac

if ! command -v conda >/dev/null 2>&1 && [[ -x "$HOME/anaconda3/bin/conda" ]]; then
  export PATH="$HOME/anaconda3/bin:$PATH"
fi
if ! command -v conda >/dev/null 2>&1; then
  echo "[error] conda is required to install the WeDetect checkpoint reader in the dino environment." >&2
  exit 1
fi

# The official checkpoint serializes mmengine metadata. The runtime only needs
# this lightweight reader, not MMCV/MMDetection. --no-deps preserves the known
# good NumPy 1.26 ABI used by ROS/OpenCV in the dino environment.
if ! conda run --no-capture-output -n dino python -c 'import mmengine; assert mmengine.__version__ == "0.10.7"' >/dev/null 2>&1; then
  conda run --no-capture-output -n dino python -m pip install --no-deps \
    'mmengine==0.10.7' 'rich==15.0.0' 'termcolor==3.1.0' \
    'pygments==2.20.0' 'markdown-it-py==3.0.0' 'mdurl==0.1.2'
fi
conda run --no-capture-output -n dino python -c 'import mmengine, numpy; assert mmengine.__version__ == "0.10.7"; assert numpy.__version__ == "1.26.4"'

if [[ -d "$SOURCE_DIR/.git" ]]; then
  git -C "$SOURCE_DIR" fetch --depth 1 origin "$REPO_COMMIT"
  git -C "$SOURCE_DIR" checkout --detach "$REPO_COMMIT"
elif [[ ! -e "$SOURCE_DIR" ]]; then
  git clone "$REPO_URL" "$SOURCE_DIR"
  git -C "$SOURCE_DIR" fetch --depth 1 origin "$REPO_COMMIT"
  git -C "$SOURCE_DIR" checkout --detach "$REPO_COMMIT"
elif [[ -f "$SOURCE_DIR/deploy/test_coco_pytorch.py" && -f "$SOURCE_DIR/config/wedetect_base.py" ]]; then
  echo "[info] reusing existing WeDetect source tree at $SOURCE_DIR"
else
  echo "[error] existing WeDetect source tree is incomplete: $SOURCE_DIR" >&2
  exit 1
fi
mkdir -p "$(dirname "$CHECKPOINT")"

CHECKPOINT_SIZE=0
if [[ -f "$CHECKPOINT" ]]; then
  CHECKPOINT_SIZE="$(stat --format='%s' "$CHECKPOINT")"
fi
if (( CHECKPOINT_SIZE < CHECKPOINT_BYTES )); then
  if ! curl --connect-timeout 15 --max-time 3600 --fail --location --continue-at - \
      --output "$CHECKPOINT" "$WEIGHT_URL"; then
    curl --connect-timeout 15 --max-time 3600 --fail --location --continue-at - \
      --output "$CHECKPOINT" "$WEIGHT_MIRROR_URL"
  fi
fi

CHECKPOINT_SIZE="$(stat --format='%s' "$CHECKPOINT")"
if (( CHECKPOINT_SIZE != CHECKPOINT_BYTES )); then
  echo "[error] WeDetect checkpoint is incomplete: $CHECKPOINT_SIZE / $CHECKPOINT_BYTES bytes" >&2
  exit 1
fi

echo "WeDetect-$MODEL_SIZE is ready: $CHECKPOINT"
