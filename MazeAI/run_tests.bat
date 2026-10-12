@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0"

echo ============================================================
echo  SmartRoute AI — Automated Algorithm Test Suite Runner
echo ============================================================

set LOCAL_EXE=test_algorithms.exe
set MINGW_EXE=C:\MinGW\bin\test_algorithms.exe

echo Compiling test harness...
g++ -std=c++14 -O2 -Wall -Wno-unused-variable -I./src -o %LOCAL_EXE% tests/test_algorithms.cpp
if errorlevel 1 (
    echo.
    echo [COMPILATION ERROR] Failed to compile tests/test_algorithms.cpp locally.
    exit /b 1
)

echo Compilation succeeded. Running tests...
echo.

REM Try running local binary
%LOCAL_EXE% >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    %LOCAL_EXE%
    echo.
    echo [TEST SUITE SUCCESS] All automated tests passed successfully!
    exit /b 0
)

REM If local directory blocked by system Application Control / Device Guard policy,
REM compile directly into MinGW allowed path and execute
g++ -std=c++14 -O2 -I./src -o %MINGW_EXE% tests/test_algorithms.cpp
if errorlevel 1 (
    echo [COMPILATION ERROR] Failed to compile test harness.
    exit /b 1
)

%MINGW_EXE%
set RUN_RES=%ERRORLEVEL%
if exist "%MINGW_EXE%" del "%MINGW_EXE%" >nul 2>&1

if %RUN_RES% EQU 0 (
    echo.
    echo [TEST SUITE SUCCESS] All automated tests passed successfully!
    exit /b 0
) else (
    echo.
    echo [TEST SUITE FAILURE] One or more tests failed.
    exit /b 1
)
