#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GROUNDINGDINO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ENV_NAME="${GROUNDINGDINO_ENV_NAME:-dino}"

if ! command -v conda >/dev/null 2>&1; then
    if [[ -x "$HOME/miniconda3/bin/conda" ]]; then
        export PATH="$HOME/miniconda3/bin:$PATH"
    else
        echo "[error] conda was not found" >&2
        exit 1
    fi
fi

eval "$(conda shell.bash hook)"
conda activate "$ENV_NAME"

TORCH_CUDA="$(python -c 'import torch; print(torch.version.cuda or "")')"
if [[ ! "$TORCH_CUDA" =~ ^[0-9]+\.[0-9]+$ ]]; then
    echo "[error] The PyTorch installation does not provide a CUDA version" >&2
    exit 1
fi

CUDA_CHANNEL="nvidia/label/cuda-${TORCH_CUDA}.0"
echo "[dino-ops] environment: $ENV_NAME"
echo "[dino-ops] PyTorch CUDA: $TORCH_CUDA"
echo "[dino-ops] installing matching compiler and development headers"
conda install -y -c "$CUDA_CHANNEL" \
    "cuda-nvcc=$TORCH_CUDA" \
    "cuda-cudart-dev=$TORCH_CUDA"

export TORCH_CUDA_ARCH_LIST="$(
    python -c 'import torch; major, minor = torch.cuda.get_device_capability(); print(f"{major}.{minor}")'
)"
export MAX_JOBS="${MAX_JOBS:-2}"

echo "[dino-ops] GPU architecture: $TORCH_CUDA_ARCH_LIST"
echo "[dino-ops] building groundingdino._C"
cd "$GROUNDINGDINO_ROOT"
python setup.py build_ext --inplace --force

python - <<'PY'
import torch

from groundingdino import _C
from groundingdino.models.GroundingDINO import ms_deform_attn

assert ms_deform_attn._C_AVAILABLE

value = torch.randn(1, 4, 1, 2, device="cuda")
spatial_shapes = torch.tensor([[2, 2]], dtype=torch.long, device="cuda")
level_start_index = torch.tensor([0], dtype=torch.long, device="cuda")
sampling_locations = torch.rand(1, 3, 1, 1, 1, 2, device="cuda")
attention_weights = torch.ones(1, 3, 1, 1, 1, device="cuda")

output = _C.ms_deform_attn_forward(
    value,
    spatial_shapes,
    level_start_index,
    sampling_locations,
    attention_weights,
    1,
)
torch.cuda.synchronize()
assert output.is_cuda and torch.isfinite(output).all()

print(f"[dino-ops] loaded: {_C.__file__}")
print(f"[dino-ops] CUDA kernel verified on {torch.cuda.get_device_name(0)}")
PY
