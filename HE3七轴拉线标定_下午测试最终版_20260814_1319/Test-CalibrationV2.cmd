@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Test-CalibrationV2.ps1"
exit /b %errorlevel%
