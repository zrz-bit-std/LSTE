#!/usr/bin/env python3
"""
MiniCPM4-0.5B vLLM 测试客户端
用于测试vLLM服务是否正常运行
"""

import openai
import sys
from pathlib import Path

MINICPM_MODEL_PATH = str(Path(__file__).resolve().parents[1] / "OpenBMB" / "MiniCPM4-0___5B")

def test_vllm_service():
    """测试vLLM服务"""
    print("=" * 60)
    print("MiniCPM4-0.5B vLLM 服务测试")
    print("=" * 60)
    print()
    
    # 创建客户端
    client = openai.Client(
        base_url="http://localhost:8000/v1",
        api_key="EMPTY"
    )
    
    # 测试问题列表
    test_cases = [
        {"role": "user", "content": "你好，请介绍一下你自己"},
        {"role": "user", "content": "请用Python写一个快速排序算法"},
        {"role": "user", "content": "解释一下什么是人工智能"},
    ]
    
    print("开始测试...\n")
    
    for i, messages in enumerate([test_cases[0]], 1):
        print(f"[测试 {i}/1] 问题: {messages['content']}")
        print("-" * 60)
        
        try:
            response = client.chat.completions.create(
                model=MINICPM_MODEL_PATH,
                messages=[messages],
                temperature=0.6,
                max_tokens=512,
                extra_body=dict(add_special_tokens=True),
            )
            
            answer = response.choices[0].message.content
            print(f"回答: {answer}")
            print()
            print("✓ 测试成功!")
            print()
            
        except Exception as e:
            print(f"✗ 测试失败: {e}")
            print()
            print("请确保:")
            print("  1. vLLM服务已启动 (运行 ./quick-start-0.5b.sh)")
            print("  2. 服务监听在 http://localhost:8000")
            print()
            return False
    
    print("=" * 60)
    print("所有测试通过! vLLM服务运行正常 ✓")
    print("=" * 60)
    return True


def interactive_chat():
    """交互式对话"""
    print()
    print("=" * 60)
    print("交互式对话模式 (输入 'quit' 或 'exit' 退出)")
    print("=" * 60)
    print()
    
    client = openai.Client(
        base_url="http://localhost:8000/v1",
        api_key="EMPTY"
    )
    
    conversation_history = []
    
    while True:
        try:
            # 获取用户输入
            user_input = input("你: ").strip()
            
            if user_input.lower() in ['quit', 'exit', '退出']:
                print("再见!")
                break
            
            if not user_input:
                continue
            
            # 添加到对话历史
            conversation_history.append({"role": "user", "content": user_input})
            
            # 调用API
            response = client.chat.completions.create(
                model=MINICPM_MODEL_PATH,
                messages=conversation_history,
                temperature=0.6,
                max_tokens=512,
                extra_body=dict(add_special_tokens=True),
            )
            
            # 获取回复
            assistant_reply = response.choices[0].message.content
            conversation_history.append({"role": "assistant", "content": assistant_reply})
            
            print(f"MiniCPM: {assistant_reply}")
            print()
            
        except KeyboardInterrupt:
            print("\n再见!")
            break
        except Exception as e:
            print(f"错误: {e}")
            print()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="MiniCPM4-0.5B vLLM 测试客户端")
    parser.add_argument("--test", action="store_true", help="运行测试")
    parser.add_argument("--chat", action="store_true", help="交互式对话")
    
    args = parser.parse_args()
    
    if args.test:
        # 运行测试
        success = test_vllm_service()
        sys.exit(0 if success else 1)
    elif args.chat:
        # 交互式对话
        interactive_chat()
    else:
        # 默认：先测试，然后进入对话
        print("正在测试vLLM服务连接...")
        print()
        if test_vllm_service():
            interactive_chat()
        else:
            sys.exit(1)
