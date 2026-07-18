@echo off
setlocal

set SCRIPT_DIR=%~dp0
powershell -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%init_new_project.ps1"

if errorlevel 1 (
  echo.
  echo Initialization failed.
  exit /b 1
)

echo.
echo Project initialization finished.
echo Created or updated project files in repository root.
echo Use RESUME_PROJECT.bat in the project root for future refresh runs.

exit /b 0
