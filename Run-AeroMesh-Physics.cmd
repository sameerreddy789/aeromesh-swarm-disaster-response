@echo off
cd /d "%~dp0"
for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd-HHmmss"') do set "RUN_ID=%%i"
".venv\Scripts\python.exe" run_physics.py --gui --output "evidence\physics-live-%RUN_ID%"
pause
