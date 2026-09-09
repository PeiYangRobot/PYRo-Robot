import sys
import os
from anthropic import Anthropic

client = Anthropic(
    api_key=os.environ["CLAUDE_API_KEY"],
    base_url="https://api.hanhegufei.online"
)

diff_file = sys.argv[1]
with open(diff_file, "r", encoding="utf-8") as f:
    diff_content = f.read()

system_prompt = """你是一位经验丰富的嵌入式系统代码审查专家。该仓库是一个基于 STM32H723 的机器人控制项目，使用 C/C++ 和 CMake 构建。请针对变更部分进行代码审查，重点关注：

1. **代码正确性与安全性**：
   - 指针使用是否安全（空指针检查、越界访问）
   - 数组索引是否有边界检查
   - 内存操作是否正确（malloc/free 配对、缓冲区溢出）
   - 中断服务函数是否简洁高效
   - 是否有竞态条件或数据竞争风险

2. **嵌入式特定问题**：
   - 寄存器操作是否正确
   - 硬件资源访问是否线程安全
   - 是否有阻塞操作在中断或实时任务中
   - DMA、定时器等外设配置是否合理
   - 栈使用是否可能溢出

3. **代码质量**：
   - 函数是否过长或过于复杂
   - 变量命名是否清晰
   - 魔术数字是否应该定义为常量
   - 是否有未使用的变量或代码

4. **性能考虑**：
   - 是否有不必要的计算或内存分配
   - 循环是否可以优化
   - 是否适合在资源受限的嵌入式环境中运行

5. **构建配置**：
   - CMakeLists.txt 修改是否合理
   - 编译选项是否正确

**输出要求**：
- 用简洁的 Markdown 列表输出
- 每条建议包含：文件路径、受影响行号、问题类型、具体建议和修改建议
- 只指出真正需要修复的问题，避免过度吹毛求疵
- 如果代码质量良好，直接回复"未发现明显问题"
- 优先关注安全性和正确性问题，其次是性能和代码风格"""

try:
    response = client.messages.create(
        model="claude-sonnet-4-6",
        max_tokens=8192,
        system=system_prompt,
        messages=[
            {
                "role": "user",
                "content": "请审查以下代码变更：\n\n" + diff_content
            }
        ]
    )

    result = response.content[0].text
    if not result or not result.strip():
        result = "AI 审查完成：未发现明显问题。"
    print(result)

except Exception as e:
    print("⚠️ AI 审查服务暂时不可用")
    print("错误类型: " + type(e).__name__)
    print("错误信息: " + str(e))
    print("\n建议检查:")
    print("1. API Key 是否有效")
    print("2. API 服务是否允许来自 GitHub Actions 的请求")
    print("3. 网络连接是否正常")
    sys.exit(0)
