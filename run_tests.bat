@echo off
REM ============================================================
REM  run_tests.bat — SmartRoute AI Root Test Runner Wrapper
REM ============================================================
cd /d "%~dp0\MazeAI"
call run_tests.bat
exit /b %ERRORLEVEL%
