@echo off
cd /d "%~dp0"
title Pendulum GUI - Wildan Oke
echo ========================================================
echo   Menjalankan Pendulum GUI (wildan-oke)
echo ========================================================
python pendulum-monitoring.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Program keluar dengan kode error %ERRORLEVEL%.
    pause
)
