@echo off
rem Страница проверки XIAO на ПК: xiao\start.cmd [--lan] [--port 9001]
chcp 65001 >nul
cd /d "%~dp0.."
python xiao\ui\serve.py %*
