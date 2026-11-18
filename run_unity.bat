@echo off
rem Double-click to start the drone core (press Play in Unity first).
cd /d "%~dp0core"
if exist "%~dp0.venv\Scripts\python.exe" (set PY="%~dp0.venv\Scripts\python.exe") else (set PY=python)
%PY% main.py --mode unity --tracker color %*
echo.
pause
