@echo off
rem Запуск портала EDOG на ПК: start.cmd [--lan] [--port 9000]
chcp 65001 >nul
cd /d "%~dp0"
python ui\serve.py %*
