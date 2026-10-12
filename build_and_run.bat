@echo off
REM ============================================================
REM  build_and_run.bat — SmartRoute AI Root Build & Launch Wrapper
REM ============================================================
cd /d "%~dp0\MazeAI"
call build_and_run.bat
exit /b %ERRORLEVEL%
