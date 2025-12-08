#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MINICPM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODEL_PATH="${MINICPM_ROOT}/OpenBMB/MiniCPM4-0___5B"

VLLM_USE_V1=0 vllm serve "$MODEL_PATH" \
    --trust-remote-code \
    --max-model-len 2048 \
    --gpu-memory-utilization 0.6 \
    --enforce-eager

#之后再开一个终端
# conda activate minicpm
# python test/test-vllm-client.py
