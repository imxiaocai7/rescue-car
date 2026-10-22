@echo off
setlocal
cd /d "%~dp0.."
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" "%~dp0drive.py" --host 192.168.4.1 --allow-hardware --allow-motion --keyboard --duty 100
) else (
    where py >nul 2>nul
    if not errorlevel 1 (
        py -3 "%~dp0drive.py" --host 192.168.4.1 --allow-hardware --allow-motion --keyboard --duty 100
    ) else (
        python "%~dp0drive.py" --host 192.168.4.1 --allow-hardware --allow-motion --keyboard --duty 100
    )
)
if errorlevel 1 pause
