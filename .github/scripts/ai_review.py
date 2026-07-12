import sys, os
from openai import OpenAI

# DeepSeek 兼容 OpenAI SDK，只需配置 base_url 和 api_key
client = OpenAI(
    api_key=os.environ["DEEPSEEK_API_KEY"],
    base_url="https://api.deepseek.com"  # http://192.168.28.57:8045/v1  https://api.deepseek.com
)

diff_file = sys.argv[1]
with open(diff_file, "r") as f:
    diff_content = f.read()

# 建议使用 deepseek-chat 模型，性价比极高
response = client.chat.completions.create(
    model="deepseek-v4-pro",          # 或 deepseek-v4-pro 用于深度推理
    messages=[
        {
            "role": "system",
            "content": "你是一位资深代码审查专家，请针对提供的 PR diff 找出潜在 bug、安全隐患、性能问题和违反最佳实践的地方。只针对变更部分输出简洁的 Markdown 列表，若未发现问题则回复“未发现明显问题”。"
        },
        {
            "role": "user",
            "content": f"请审查以下代码差异：\n\n{diff_content}"
        }
    ],
    temperature=0.0,                # 严谨审查建议用 0
    max_tokens=2048,
)

print(response.choices[0].message.content)