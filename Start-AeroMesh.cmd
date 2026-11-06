@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    py -3.10 -m venv .venv
    if errorlevel 1 goto failed
)
".venv\Scripts\python.exe" -c "import pygame,numpy,networkx,panda3d,PIL" >nul 2>&1
if errorlevel 1 (
    ".venv\Scripts\python.exe" -m pip install -r requirements-repro.txt
    if errorlevel 1 goto failed
)
".venv\Scripts\python.exe" scripts\start_aeromesh.py %*
if errorlevel 1 goto failed
exit /b 0
:failed
echo AeroMesh could not start. Review the error above.
pause
exit /b 1
