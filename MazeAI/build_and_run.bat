@echo off
cd /d "%~dp0"
echo ============================================================
echo  SmartRoute AI — Build and Launch
echo ============================================================
echo  Compiling SmartRoute AI...

if not exist frames mkdir frames

g++ -std=c++14 -O2 -Wall -Wno-unused-variable ^
    -I./src -I. -L./lib ^
    -o SmartRouteAI.exe src/main.cpp ^
    -lfreeglut -lopengl32 -lglu32 -mwindows

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo  [ERROR] Build failed. Check g++ is on PATH and GL/lib folders exist.
    pause
    exit /b %ERRORLEVEL%
)

echo  [SUCCESS] SmartRouteAI.exe compiled successfully!
echo  Launching SmartRoute AI...
echo.
start "" SmartRouteAI.exe
