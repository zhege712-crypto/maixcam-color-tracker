@echo off
chcp 936 >nul 2>nul
setlocal EnableExtensions EnableDelayedExpansion
title 智能搬运 · 上位机（协议 v4）
cd /d "%~dp0"

echo ================================================================
echo    智能搬运 · 上位机
echo    协议 v4：相机上行 20 字节 / 下行 4 字节
echo ================================================================
echo.

set "PY="
set "CAND="
set "STORE="

rem ---------- 1. py 启动器：python.org 官方安装包自带，最可靠 ----------
where py >nul 2>nul
if not errorlevel 1 (
    py -3 -c "import sys" >nul 2>nul
    if not errorlevel 1 set "PY=py -3"
)
if defined PY goto :got_py

rem ---------- 2. PATH 里的 python.exe：优先非商店版，实在没有再用商店版 ----------
where python >nul 2>nul
if not errorlevel 1 (
    for /f "delims=" %%p in ('where python 2^>nul') do (
        echo %%p | find /i "\WindowsApps\" >nul && set "STORE=%%p" || set "CAND=%%p"
    )
)
if defined CAND (
    "!CAND!" -c "import sys" >nul 2>nul
    if not errorlevel 1 set "PY=!CAND!"
)
if defined PY goto :got_py
if defined STORE (
    "!STORE!" -c "import sys" >nul 2>nul
    if not errorlevel 1 set "PY=!STORE!"
)
if defined PY goto :got_py

rem ---------- 3. 装了但没勾 Add to PATH 的常见位置 ----------
for %%b in ("%LOCALAPPDATA%\Programs\Python" "%ProgramFiles%\Python") do (
    if exist "%%~b" (
        for /d %%d in ("%%~b\Python3*") do (
            if not defined PY if exist "%%~d\python.exe" (
                "%%~d\python.exe" -c "import sys" >nul 2>nul
                if not errorlevel 1 set "PY=%%~d\python.exe"
            )
        )
    )
)
if defined PY goto :got_py

for /d %%d in ("C:\Python3*") do (
    if not defined PY if exist "%%~d\python.exe" (
        "%%~d\python.exe" -c "import sys" >nul 2>nul
        if not errorlevel 1 set "PY=%%~d\python.exe"
    )
)

:got_py
if not defined PY (
    echo [错误] 这台电脑上没找到 Python。
    echo.
    echo   本工具需要 Python 3.8 以上，下载地址：
    echo     https://www.python.org/downloads/
    echo   安装时请勾选 “Add python.exe to PATH”，装完重新双击本文件。
    echo.
    pause
    exit /b 1
)

%PY% -c "import sys;raise SystemExit(0 if sys.version_info>=(3,8) else 3)" >nul 2>nul
if errorlevel 1 (
    echo [错误] Python 版本太旧，需要 3.8 或更新，当前版本：
    %PY% -V
    echo   请到 https://www.python.org/downloads/ 装新版。
    echo.
    pause
    exit /b 1
)

%PY% -c "import tkinter" >nul 2>nul
if errorlevel 1 (
    echo [错误] 这个 Python 没带 tkinter 图形库，界面起不来。
    echo   请用 python.org 的官方安装包重装，安装时保留 tcl/tk 选项。
    echo.
    pause
    exit /b 1
)

if not exist "app\ground_station.py" (
    echo [错误] 文件不全：找不到 app\ground_station.py
    echo   请把整个“智能搬运上位机”文件夹一起拷贝，不要只拿这个 bat。
    echo.
    pause
    exit /b 1
)

echo 正在启动界面……
echo 提示：这个黑窗口是启动器，关掉它等于退出上位机。
echo.

%PY% "app\ground_station.py" %*
set "RC=%ERRORLEVEL%"

if "%RC%"=="0" goto :done
echo.
echo [异常退出] 返回码 %RC%
echo 把上面的报错截图发出来即可定位。
echo.
pause

:done
exit /b 0
