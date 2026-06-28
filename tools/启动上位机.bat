@echo off
chcp 65001 >nul
title STM32 PID 温控器上位机

"D:\Program Files\Python 3.14\python.exe" "%~dp0pid_upper_monitor.py"
pause
