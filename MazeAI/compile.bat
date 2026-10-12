@echo off
REM ============================================================
REM  compile.bat — SmartRoute AI Build Script for Windows MinGW
REM ============================================================
REM  Prerequisites:
REM    1. MinGW/g++ 6.3+ on PATH  (run: g++ --version)
REM    2. GL/  folder with glut.h, freeglut headers (already present)
REM    3. lib/ folder with libfreeglut.a (already present)
REM    4. freeglut.dll in this directory (already present)
REM ============================================================

cd /d "%~dp0"

if not exist frames mkdir frames

echo.
echo ============================================================
echo  SmartRoute AI — Build Script
echo ============================================================
echo  Compiling...

g++ -std=c++14 ^
    -O2 ^
    -Wall ^
    -Wno-unused-variable ^
    -I./src ^
    -I. ^
    -L./lib ^
    -o SmartRouteAI.exe ^
    src/main.cpp ^
    -lfreeglut -lopengl32 -lglu32 ^
    -mwindows

if %ERRORLEVEL% EQU 0 (
    echo.
    echo  *** BUILD SUCCESS ***
    echo.
    echo  Run with:   SmartRouteAI.exe
    echo  Or double-click build_and_run.bat to build + launch.
    echo.
    echo  Keyboard shortcuts:
    echo    G       = Generate road network
    echo    1/2/3   = Select BFS / DFS / A*
    echo    C       = Compare all algorithms
    echo    A       = Animate delivery van route
    echo    SPACE   = Pause / Resume
    echo    N       = Step one node expansion
    echo    L       = Legend screen
    echo    H       = Help / How It Works
    echo    ESC     = Return / Back
    echo.
    exit /b 0
) else (
    echo.
    echo  *** BUILD FAILED — check errors above ***
    echo.
    echo  Common fixes:
    echo    - Ensure GL/ and lib/ folders exist with freeglut headers
    echo    - Ensure g++ 6.3+ is on PATH   (run: g++ --version)
    echo    - Ensure freeglut.dll is in this directory
    echo.
    exit /b 1
)
