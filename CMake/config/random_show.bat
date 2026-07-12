@echo off
setlocal enabledelayedexpansion

:: 1. 获取参数（目录路径）
set "TARGET_DIR=%~1"

:: 2. 检查目录是否存在
if not exist "%TARGET_DIR%" (
    echo [Batch] Warning: Directory not found: %TARGET_DIR%
    exit /b 0
)

:: 3. 收集文件列表并计数
set count=0
for %%f in ("%TARGET_DIR%\*.txt") do (
    set /a count+=1
    set "file[!count!]=%%f"
)

:: 4. 如果没文件，直接退出
if %count%==0 (
@REM     echo [Batch] No .txt files found in %TARGET_DIR%
    exit /b 0
)

:: 5. 生成随机数 (1 到 count)
set /a idx=%random% %% %count% + 1
set "selected_file=!file[%idx%]!"

:: 6. 核心：防止乱码并输出
:: chcp 65001 强制终端使用 UTF-8 解析接下来的 output
chcp 65001 >nul
type "!selected_file!"

endlocal