@echo off
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
  py simulator\mock_server.py
) else (
  python simulator\mock_server.py
)
pause
