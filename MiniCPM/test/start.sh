VLLM_USE_V1=0 vllm serve /home/zrz/Desktop/LSTE/MiniCPM/OpenBMB/MiniCPM4-0___5B     --trust-remote-code     --max-model-len 2048     --gpu-memory-utilization 0.6     --enforce-eager

#之后再开一个终端
conda activate minicpm
python test/test-vllm-client.py
