@echo off
setlocal
cd /d "%~dp0"
echo Packaging SmartRouteAI.zip...

set SRC_DIR=%~dp0
if "%SRC_DIR:~-1%"=="\" set SRC_DIR=%SRC_DIR:~0,-1%
set STAGING=%SRC_DIR%\zip_staging\SmartRouteAI
set ZIP_FILE=%SRC_DIR%\SmartRouteAI.zip

if exist "%SRC_DIR%\zip_staging" rmdir /s /q "%SRC_DIR%\zip_staging"
mkdir "%STAGING%"

xcopy /E /I /Q /Y "%SRC_DIR%\src" "%STAGING%\src"
xcopy /E /I /Q /Y "%SRC_DIR%\tests" "%STAGING%\tests"
xcopy /E /I /Q /Y "%SRC_DIR%\GL" "%STAGING%\GL"
xcopy /E /I /Q /Y "%SRC_DIR%\lib" "%STAGING%\lib"
copy /Y "%SRC_DIR%\compile.bat" "%STAGING%\"
copy /Y "%SRC_DIR%\run_tests.bat" "%STAGING%\"
copy /Y "%SRC_DIR%\build_and_run.bat" "%STAGING%\"
copy /Y "%SRC_DIR%\freeglut.dll" "%STAGING%\"
copy /Y "%SRC_DIR%\SmartRouteAI.exe" "%STAGING%\"
copy /Y "%SRC_DIR%\README.md" "%STAGING%\"

if exist "%ZIP_FILE%" del /f /q "%ZIP_FILE%"

powershell -NoProfile -Command "Compress-Archive -Path '%SRC_DIR%\zip_staging\SmartRouteAI' -DestinationPath '%ZIP_FILE%' -Force"

if exist "%SRC_DIR%\..\SmartRouteAI.zip" copy /Y "%ZIP_FILE%" "%SRC_DIR%\..\SmartRouteAI.zip" >nul 2>&1

rmdir /s /q "%SRC_DIR%\zip_staging"

echo Packaging complete.
dir "%ZIP_FILE%"
