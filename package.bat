@echo off
REM ============================================================
REM  package.bat — SmartRoute AI Root Package Wrapper
REM ============================================================
cd /d "%~dp0\MazeAI"
call package.bat
exit /b %ERRORLEVEL%
