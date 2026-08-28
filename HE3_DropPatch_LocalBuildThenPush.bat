@echo off
setlocal EnableExtensions
title HE3 Drop Patch Local Build Then Push
cd /d "%~dp0"
if "%~1"=="" (
  echo Drag a ChatGPT .patch file onto this BAT.
  echo.
  echo Flow: pull main ^> apply patch ^> local VS2022 Release x86 build ^> verify ^> commit ^> push main
  echo Build failure: no push; source is restored; logs stay under _local_build\failures.
  pause
  exit /b 2
)
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\HE3_LocalPatchBuildPush.ps1" -PatchPath "%~f1" -RepoPath "%~dp0"
set "RC=%ERRORLEVEL%"
echo.
echo PowerShell exit code: %RC%
if not "%RC%"=="0" (
  echo FAILED. Check _local_build\logs or _local_build\failures.
  pause
  exit /b %RC%
)
echo PASS. EXE: _local_build\latest\HE3_RopeCalibration_V3_Tool.exe
pause
