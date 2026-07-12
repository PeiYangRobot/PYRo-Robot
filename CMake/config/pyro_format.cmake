# ANSI转义码颜色和格式定义

# 获取ESC字符
string(ASCII 27 ESC)

# ===========================================
# 重置所有属性
# ===========================================
set(RESET_ALL "${ESC}[0m")      # 重置所有属性（颜色、样式等）

# ===========================================
# 文本样式（字体格式）
# ===========================================

# 基本样式
set(STYLE_BOLD "1")       # 粗体/高亮
set(STYLE_DIM "2")        # 暗淡/弱化
set(STYLE_ITALIC "3")     # 斜体
set(STYLE_UNDERLINE "4")  # 下划线
set(STYLE_BLINK_SLOW "5") # 慢闪烁
set(STYLE_BLINK_FAST "6") # 快闪烁（不常用）
set(STYLE_REVERSE "7")    # 反色（前景背景互换）
set(STYLE_HIDDEN "8")     # 隐藏/不可见
set(STYLE_STRIKETHROUGH "9") # 删除线

# 关闭样式
set(STYLE_BOLD_OFF "22")         # 关闭粗体/暗淡
set(STYLE_ITALIC_OFF "23")       # 关闭斜体
set(STYLE_UNDERLINE_OFF "24")    # 关闭下划线
set(STYLE_BLINK_OFF "25")        # 关闭闪烁
set(STYLE_REVERSE_OFF "27")      # 关闭反色
set(STYLE_HIDDEN_OFF "28")       # 关闭隐藏
set(STYLE_STRIKETHROUGH_OFF "29")# 关闭删除线

# ===========================================
# 标准16色 - 前景色
# ===========================================

# 普通颜色
set(FG_BLACK "30")    # 黑色
set(FG_RED "31")      # 红色
set(FG_GREEN "32")    # 绿色
set(FG_YELLOW "33")   # 黄色
set(FG_BLUE "34")     # 蓝色
set(FG_MAGENTA "35")  # 洋红/紫色
set(FG_CYAN "36")     # 青色/蓝绿
set(FG_WHITE "37")    # 白色

# 亮色/高亮色
set(FG_BRIGHT_BLACK "90")    # 亮黑色/灰色
set(FG_BRIGHT_RED "91")      # 亮红色
set(FG_BRIGHT_GREEN "92")    # 亮绿色
set(FG_BRIGHT_YELLOW "93")   # 亮黄色
set(FG_BRIGHT_BLUE "94")     # 亮蓝色
set(FG_BRIGHT_MAGENTA "95")  # 亮洋红
set(FG_BRIGHT_CYAN "96")     # 亮青色
set(FG_BRIGHT_WHITE "97")    # 亮白色

# 关闭前景色
set(FG_DEFAULT "39")  # 默认前景色

# ===========================================
# 标准16色 - 背景色
# ===========================================

# 普通背景色
set(BG_BLACK "40")    # 黑色背景
set(BG_RED "41")      # 红色背景
set(BG_GREEN "42")    # 绿色背景
set(BG_YELLOW "43")   # 黄色背景
set(BG_BLUE "44")     # 蓝色背景
set(BG_MAGENTA "45")  # 洋红背景
set(BG_CYAN "46")     # 青色背景
set(BG_WHITE "47")    # 白色背景

# 亮背景色
set(BG_BRIGHT_BLACK "100")   # 亮黑背景/灰背景
set(BG_BRIGHT_RED "101")     # 亮红背景
set(BG_BRIGHT_GREEN "102")   # 亮绿背景
set(BG_BRIGHT_YELLOW "103")  # 亮黄背景
set(BG_BRIGHT_BLUE "104")    # 亮蓝背景
set(BG_BRIGHT_MAGENTA "105") # 亮洋红背景
set(BG_BRIGHT_CYAN "106")    # 亮青背景
set(BG_BRIGHT_WHITE "107")   # 亮白背景

# 关闭背景色
set(BG_DEFAULT "49")  # 默认背景色

# ===========================================
# 256色模式
# ===========================================

# 语法：${ESC}[38;5;{n}m 设置前景色
# 语法：${ESC}[48;5;{n}m 设置背景色
# 其中n是0-255的颜色索引

# 常用256色前景色
set(FG_256_ORANGE "38;5;214")   # 橙色
set(FG_256_PINK "38;5;205")     # 粉色
set(FG_256_PURPLE "38;5;129")   # 紫色
set(FG_256_BROWN "38;5;130")    # 棕色
set(FG_256_GRAY_LIGHT "38;5;250") # 浅灰
set(FG_256_GRAY_DARK "38;5;238")  # 深灰
set(FG_256_LIME "38;5;118")     # 柠檬绿
set(FG_256_TEAL "38;5;30")      # 凫蓝
set(FG_256_GOLD "38;5;178")     # 金色
set(FG_256_SILVER "38;5;250")   # 银色

# 常用256色背景色
set(BG_256_ORANGE "48;5;214")
set(BG_256_PINK "48;5;205")
set(BG_256_PURPLE "48;5;129")

# 256色模式关闭
set(FG_256_DEFAULT "39")
set(BG_256_DEFAULT "49")

# ===========================================
# RGB真彩色模式 (24位色)
# ===========================================

# 语法：${ESC}[38;2;{r};{g};{b}m 设置前景RGB色
# 语法：${ESC}[48;2;{r};{g};{b}m 设置背景RGB色
# 其中r,g,b是0-255的RGB值

# 常用RGB前景色
set(FG_RGB_RED "38;2;255;0;0")         # 纯红
set(FG_RGB_GREEN "38;2;0;255;0")       # 纯绿
set(FG_RGB_BLUE "38;2;0;0;255")        # 纯蓝
set(FG_RGB_YELLOW "38;2;255;255;0")    # 纯黄
set(FG_RGB_CYAN "38;2;0;255;255")      # 纯青
set(FG_RGB_MAGENTA "38;2;255;0;255")   # 纯洋红
set(FG_RGB_WHITE "38;2;255;255;255")   # 纯白
set(FG_RGB_BLACK "38;2;0;0;0")         # 纯黑
set(FG_RGB_ORANGE "38;2;255;165;0")    # 橙色
set(FG_RGB_PINK "38;2;255;192;203")    # 粉色
set(FG_RGB_PURPLE "38;2;128;0;128")    # 紫色
set(FG_RGB_GOLD "38;2;255;215;0")      # 金色

# 常用RGB背景色
set(BG_RGB_RED "48;2;255;0;0")
set(BG_RGB_GREEN "48;2;0;255;0")
set(BG_RGB_BLUE "48;2;0;0;255")
set(BG_RGB_DARK_BLUE "48;2;0;0;128")   # 深蓝背景

# RGB模式关闭
set(FG_RGB_DEFAULT "39")
set(BG_RGB_DEFAULT "49")

# ===========================================
# 预设组合格式（使用常量定义，方便阅读和修改）
# ===========================================

# =============================================================================
#  Modern UI Palette (Based on Tailwind CSS & Ant Design)
# =============================================================================

# 核心功能色 (RGB)
set(RGB_EMERALD_500 "48;2;16;185;129")   # 成功 (Success) - 清新翠绿
set(RGB_AMBER_500   "48;2;245;158;11")   # 警告 (Warning) - 温暖琥珀
set(RGB_ROSE_500    "48;2;244;63;94")    # 错误 (Error)   - 柔和玫瑰红
set(RGB_SKY_500     "48;2;14;165;233")   # 信息 (Info)    - 天空蓝
set(RGB_VIOLET_500  "48;2;139;92;246")   # 标题 (Title)   - 罗兰紫
set(RGB_SLATE_800   "48;2;30;41;59")     # 深色背景 (Dark Bg)

# 辅助文本色 (RGB 前景)
set(F_RGB_WHITE     "38;2;255;255;255")  # 纯白文本
set(F_RGB_SLATE_900 "38;2;15;23;42")     # 深色文本 (配合浅背景)
set(F_RGB_CYAN_400  "38;2;34;211;238")   # 高亮青色文本
set(F_RGB_INDIGO_400 "38;2;129;140;248") # 边框蓝紫文本

# =============================================================================
#  预设组合格式 (Modern UI Style)
# =============================================================================

# 成功状态 [Success]: 翠绿背景 + 纯白文字 + 粗体
# 视觉心理：通过降低绿色的饱和度，使其不刺眼但充满活力
set(SUCCESS_FORMAT "${ESC}[${STYLE_BOLD};${F_RGB_WHITE};${RGB_EMERALD_500}m")

# 警告状态 [Warning]: 琥珀色背景 + 深色文字(增加对比度) + 粗体
# 前端原则：黄色背景上通常使用深色文字以保证可读性 (A11y)
set(WARNING_FORMAT "${ESC}[${STYLE_BOLD};${F_RGB_SLATE_900};${RGB_AMBER_500}m")

# 错误状态 [Error]: 玫瑰红背景 + 纯白文字 + 粗体
# 视觉心理：比纯红(#FF0000)更现代，具有警示性但不产生视觉疲劳
set(ERROR_FORMAT "${ESC}[${STYLE_BOLD};${F_RGB_WHITE};${RGB_ROSE_500}m")

# 信息状态 [Info]: 天空蓝背景 + 纯白文字 + 粗体
set(INFO_FORMAT "${ESC}[${STYLE_BOLD};${F_RGB_WHITE};${RGB_SKY_500}m")

# 标题格式 [Title]: 罗兰紫背景 + 纯白文字 + 粗体
# 视觉心理：紫色代表高级感和统领性，适合作为 Header
set(TITLE_FORMAT "${ESC}[${STYLE_BOLD};${F_RGB_WHITE};${RGB_VIOLET_500}m")

# 强调格式 [Highlight]: 亮白文字 + 深蓝灰背景 (类似代码块效果)
# 设计风格：模仿 Markdown 代码块的渲染效果
set(HIGHLIGHT_FORMAT "${ESC}[${STYLE_BOLD};${F_RGB_CYAN_400};${RGB_SLATE_800}m")