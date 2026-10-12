@echo off
REM ============================================================
REM  setup_deps.bat  —  MAZEAI Dependency Setup
REM  Run this ONCE before compiling for the first time.
REM ============================================================
REM  What it does:
REM    1. Downloads freeglut headers from GitHub into GL\
REM    2. Copies freeglut.dll from the existing project
REM    3. Generates a freeglut .a import library from the DLL
REM ============================================================

setlocal

echo ============================================================
echo MAZEAI Dependency Setup
echo ============================================================

REM -- Step 1: freeglut headers ---------------------------------
echo.
echo [1/3] Downloading freeglut headers from GitHub...
if not exist GL mkdir GL

powershell -Command "Invoke-WebRequest -Uri 'https://raw.githubusercontent.com/freeglut/freeglut/master/include/GL/freeglut.h' -OutFile 'GL\freeglut.h' -UseBasicParsing" 2>&1
powershell -Command "Invoke-WebRequest -Uri 'https://raw.githubusercontent.com/freeglut/freeglut/master/include/GL/freeglut_std.h' -OutFile 'GL\freeglut_std.h' -UseBasicParsing" 2>&1
powershell -Command "Invoke-WebRequest -Uri 'https://raw.githubusercontent.com/freeglut/freeglut/master/include/GL/freeglut_ext.h' -OutFile 'GL\freeglut_ext.h' -UseBasicParsing" 2>&1
powershell -Command "Invoke-WebRequest -Uri 'https://raw.githubusercontent.com/freeglut/freeglut/master/include/GL/freeglut_ucall.h' -OutFile 'GL\freeglut_ucall.h' -UseBasicParsing" 2>&1

if exist GL\freeglut.h (
    echo    -> freeglut.h        OK
) else (
    echo    -> FAILED to download GL\freeglut.h
    echo    Manually copy headers into GL\ from freeglut.sourceforge.net
    goto :end
)

REM -- Step 2: Copy freeglut.dll --------------------------------
echo.
echo [2/3] Copying freeglut.dll from sister project...
if not exist freeglut.dll (
    if exist "..\Project_maze\main\freeglut.dll" (
        copy "..\Project_maze\main\freeglut.dll" freeglut.dll >nul
        echo    -> freeglut.dll      OK (copied from Project_maze)
    ) else (
        echo    -> freeglut.dll NOT FOUND in ..\Project_maze\main\
        echo    Copy freeglut.dll manually next to MazeAI.exe
    )
) else (
    echo    -> freeglut.dll      already present
)

REM -- Step 3: Generate import library --------------------------
echo.
echo [3/3] Generating libfreeglut.a import library...
if not exist lib mkdir lib

REM Check if dlltool is available
where dlltool >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo    -> dlltool not found in PATH.
    echo    Trying to find it in MinGW...
    set DLLTOOL=C:\MinGW\bin\dlltool.exe
) else (
    set DLLTOOL=dlltool
)

if exist "%DLLTOOL%" (
    "%DLLTOOL%" --dllname freeglut.dll --output-lib lib\libfreeglut.a --kill-at 2>&1
    if exist lib\libfreeglut.a (
        echo    -> libfreeglut.a     OK
    ) else (
        echo    -> WARNING: libfreeglut.a was NOT created.
        echo    Try: dlltool --dllname freeglut.dll --output-lib lib\libfreeglut.a --kill-at
    )
) else (
    echo    -> dlltool not available. Trying pexports + dlltool alternative...
    echo.
    echo    Manual alternative:
    echo      1) Run: pexports freeglut.dll ^> freeglut.def
    echo      2) Run: dlltool -D freeglut.dll -d freeglut.def -l lib\libfreeglut.a
)

:end
echo.
echo ============================================================
echo Setup complete. Now run compile.bat to build MazeAI.
echo ============================================================
pause
