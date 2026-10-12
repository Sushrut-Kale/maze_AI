@echo off
REM ============================================================
REM  compile.bat — SmartRoute AI Root Build Wrapper
REM ============================================================
cd /d "%~dp0\MazeAI"
call compile.bat
exit /b %ERRORLEVEL%
