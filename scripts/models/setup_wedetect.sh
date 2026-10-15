#!/usr/bin/env bash
set -euo pipefail

# Install the optional WeDetect source and Base zero-shot checkpoint outside Git.
# The checkpoint is 1.59 GB and requires roughly 2 GB free disk while downloading.
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
SOURCE_DIR="$WS/model/WeDetect"
CHECKPOINT="$SOURCE_DIR/checkpoints/wedetect_base.pth"
REPO_URL="https://github.com/WeChatCV/WeDetect.git"
REPO_COMMIT="dd302dba0069ace1b05816bafbc3fa1dbd6aa68c"
WEIGHT_URL="https://huggingface.co/fushh7/WeDetect/resolve/main/wedetect_base.pth?download=true"
CHECKPOINT_BYTES=1588423888

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

if [[ ! -d "$SOURCE_DIR/.git" ]]; then
  git clone "$REPO_URL" "$SOURCE_DIR"
fi
git -C "$SOURCE_DIR" fetch --depth 1 origin "$REPO_COMMIT"
git -C "$SOURCE_DIR" checkout --detach "$REPO_COMMIT"
mkdir -p "$(dirname "$CHECKPOINT")"

CHECKPOINT_SIZE=0
if [[ -f "$CHECKPOINT" ]]; then
  CHECKPOINT_SIZE="$(stat --format='%s' "$CHECKPOINT")"
fi
if (( CHECKPOINT_SIZE < CHECKPOINT_BYTES )); then
  curl --fail --location --continue-at - --output "$CHECKPOINT" "$WEIGHT_URL"
fi

CHECKPOINT_SIZE="$(stat --format='%s' "$CHECKPOINT")"
if (( CHECKPOINT_SIZE != CHECKPOINT_BYTES )); then
  echo "[error] WeDetect checkpoint is incomplete: $CHECKPOINT_SIZE / $CHECKPOINT_BYTES bytes" >&2
  exit 1
fi

echo "WeDetect-Base is ready: $CHECKPOINT"
