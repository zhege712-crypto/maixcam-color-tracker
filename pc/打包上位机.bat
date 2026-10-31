@echo off
chcp 936 >nul 2>nul
title 打包上位机
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_release.ps1" %*
echo.
pause
