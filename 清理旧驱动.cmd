@echo off
rem 双击运行：清理已被 bsp/ 取代的第三方驱动文件
rem 实际逻辑在 tools\清理旧驱动.ps1
pwsh -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\清理旧驱动.ps1"
echo.
pause
