@echo off
setlocal EnableExtensions
title HE3 R7.4 First Local Build And Push
cd /d "%~dp0"
echo HE3 R7.4 first local build.
echo This builds the current uncommitted R7.4 changes locally.
echo Commit and push happen only after Release x86 build passes.
echo.
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\HE3_LocalPatchBuildPush.ps1" -RepoPath "%~dp0" -CommitMessage "R7.4: diagnostic matching, metric labels and local build sync"
set "RC=%ERRORLEVEL%"
echo.
echo PowerShell exit code: %RC%
if not "%RC%"=="0" (
  echo FAILED. Check _local_build\logs or _local_build\failures.
  pause
  exit /b %RC%
)
echo PASS. R7.4 was built locally and pushed to main.
pause
