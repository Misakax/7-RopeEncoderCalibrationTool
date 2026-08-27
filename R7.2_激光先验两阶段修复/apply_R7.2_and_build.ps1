param(
    [string]$BuildRoot = "",
    [switch]$NoBuild
)

$ErrorActionPreference = "Stop"

function Require-Path([string]$Path, [string]$Label) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Missing $Label : $Path"
    }
}

Write-Host "=== R7.2 apply and build ===" -ForegroundColor Cyan

$FixRoot = $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($BuildRoot)) {
    $WorkspaceRoot = Split-Path -Parent $FixRoot
    $BuildRoot = Join-Path $WorkspaceRoot "R7.1-Build"
}

Write-Host "R7.2 package : $FixRoot"
Write-Host "Build tree   : $BuildRoot"

Require-Path $BuildRoot "R7.1-Build directory"

$srcDllCpp = Join-Path $FixRoot "CalibrationV3Analytic\CalibrationV3Analytic.cpp"
$srcDllH   = Join-Path $FixRoot "CalibrationV3Analytic\CalibrationV3Analytic.h"
$srcGuiCpp = Join-Path $FixRoot "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp"
$srcQkmCpp = Join-Path $FixRoot "RopeEncoderCalibrationV3\QKMLinkComm.cpp"

Require-Path $srcDllCpp "R7.2 CalibrationV3Analytic.cpp"
Require-Path $srcDllH   "R7.2 CalibrationV3Analytic.h"
Require-Path $srcGuiCpp "R7.2 RopeEncoderCalibrationDlg.cpp"
Require-Path $srcQkmCpp "R7.2 QKMLinkComm.cpp"

$dstDllDir = Join-Path $BuildRoot "CalibrationV3Analytic"
$dstGuiDir = Join-Path $BuildRoot "RopeEncoderCalibrationV3"

Require-Path $dstDllDir "target CalibrationV3Analytic directory"
Require-Path $dstGuiDir "target RopeEncoderCalibrationV3 directory"

$dstDllCpp = Join-Path $dstDllDir "CalibrationV3Analytic.cpp"
$dstDllH   = Join-Path $dstDllDir "CalibrationV3Analytic.h"
$dstGuiCpp = Join-Path $dstGuiDir "RopeEncoderCalibrationDlg.cpp"
$dstQkmCpp = Join-Path $dstGuiDir "QKMLinkComm.cpp"

Require-Path $dstDllCpp "target CalibrationV3Analytic.cpp"
Require-Path $dstDllH   "target CalibrationV3Analytic.h"
Require-Path $dstGuiCpp "target RopeEncoderCalibrationDlg.cpp"
Require-Path $dstQkmCpp "target QKMLinkComm.cpp"

# Verify package identity before touching the build tree.
$dllText = Get-Content -LiteralPath $srcDllCpp -Raw
$guiText = Get-Content -LiteralPath $srcGuiCpp -Raw

if ($dllText -notmatch "R7\.2") {
    throw "Package check failed: CalibrationV3Analytic.cpp does not contain R7.2."
}
if ($guiText -notmatch "R7\.2") {
    throw "Package check failed: RopeEncoderCalibrationDlg.cpp does not contain R7.2."
}

# Backup the current R7.1/R7.2 build-tree source before overwrite.
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$backupRoot = Join-Path $BuildRoot ("backup_before_R7.2_" + $stamp)
New-Item -ItemType Directory -Path (Join-Path $backupRoot "CalibrationV3Analytic") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $backupRoot "RopeEncoderCalibrationV3") -Force | Out-Null

Copy-Item -LiteralPath $dstDllCpp -Destination (Join-Path $backupRoot "CalibrationV3Analytic\CalibrationV3Analytic.cpp") -Force
Copy-Item -LiteralPath $dstDllH   -Destination (Join-Path $backupRoot "CalibrationV3Analytic\CalibrationV3Analytic.h") -Force
Copy-Item -LiteralPath $dstGuiCpp -Destination (Join-Path $backupRoot "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp") -Force
Copy-Item -LiteralPath $dstQkmCpp -Destination (Join-Path $backupRoot "RopeEncoderCalibrationV3\QKMLinkComm.cpp") -Force

Write-Host "Backup created: $backupRoot" -ForegroundColor DarkYellow

# Apply R7.2 source.
Copy-Item -LiteralPath $srcDllCpp -Destination $dstDllCpp -Force
Copy-Item -LiteralPath $srcDllH   -Destination $dstDllH -Force
Copy-Item -LiteralPath $srcGuiCpp -Destination $dstGuiCpp -Force
Copy-Item -LiteralPath $srcQkmCpp -Destination $dstQkmCpp -Force

Write-Host "R7.2 source applied." -ForegroundColor Green

# Verify target after copy.
$targetDllText = Get-Content -LiteralPath $dstDllCpp -Raw
$targetGuiText = Get-Content -LiteralPath $dstGuiCpp -Raw
if ($targetDllText -notmatch "R7\.2") {
    throw "Target verification failed for CalibrationV3Analytic.cpp."
}
if ($targetGuiText -notmatch "R7\.2") {
    throw "Target verification failed for RopeEncoderCalibrationDlg.cpp."
}

if ($NoBuild) {
    Write-Host "NoBuild specified. Open the solution and build Release|x86 manually." -ForegroundColor Yellow
    exit 0
}

$sln = Join-Path $BuildRoot "HE3_RopeCalibration_R7.1_Minimal.sln"
Require-Path $sln "solution file"

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
Require-Path $vswhere "vswhere.exe"

$msbuild = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find "MSBuild\**\Bin\MSBuild.exe" | Select-Object -First 1
if ([string]::IsNullOrWhiteSpace($msbuild)) {
    throw "MSBuild.exe was not found. Install VS2022 C++ desktop development tools."
}

Write-Host "MSBuild: $msbuild"
Write-Host "Rebuilding Release|x86 ..." -ForegroundColor Cyan

& $msbuild $sln /m /t:Rebuild /p:Configuration=Release /p:Platform=x86

if ($LASTEXITCODE -ne 0) {
    throw "MSBuild failed. Exit code: $LASTEXITCODE"
}

$exe = Join-Path $BuildRoot "bin\Win32\Release\HE3_RopeCalibration_V3_Tool.exe"
$dll = Join-Path $BuildRoot "bin\Win32\Release\CalibrationV3Analytic.dll"

Require-Path $exe "R7.2 EXE"
Require-Path $dll "R7.2 DLL"

Write-Host ""
Write-Host "BUILD PASS" -ForegroundColor Green
Write-Host "EXE: $exe"
Write-Host "DLL: $dll"
Write-Host ""
Write-Host "First run: acquire + calculate only. Do NOT save to the robot yet." -ForegroundColor Yellow
