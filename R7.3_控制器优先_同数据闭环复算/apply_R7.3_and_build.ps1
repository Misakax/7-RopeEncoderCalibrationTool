param(
    [string]$BuildRoot = ""
)

$ErrorActionPreference = "Stop"

function Require-Path([string]$Path, [string]$Label) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Missing $Label : $Path"
    }
}

Write-Host "=== R7.3 controller-first replay apply and build ===" -ForegroundColor Cyan

$PkgRoot = $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($BuildRoot)) {
    $WorkspaceRoot = Split-Path -Parent $PkgRoot
    $BuildRoot = Join-Path $WorkspaceRoot "R7.1-Build"
}

$srcDll = Join-Path $PkgRoot "CalibrationV3Analytic\CalibrationV3Analytic.cpp"
$srcGui = Join-Path $PkgRoot "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp"
$dstDll = Join-Path $BuildRoot "CalibrationV3Analytic\CalibrationV3Analytic.cpp"
$dstGui = Join-Path $BuildRoot "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp"

Require-Path $BuildRoot "build tree"
Require-Path $srcDll "R7.3 DLL source"
Require-Path $srcGui "R7.3 GUI source"
Require-Path $dstDll "target DLL source"
Require-Path $dstGui "target GUI source"

$guiText = Get-Content -LiteralPath $srcGui -Raw
$dllText = Get-Content -LiteralPath $srcDll -Raw
if ($guiText -notmatch "R7\.3 Controller-First") { throw "GUI identity check failed." }
if ($dllText -notmatch "R7\.3") { throw "DLL identity check failed." }

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$backup = Join-Path $BuildRoot ("backup_before_R7.3_" + $stamp)
New-Item -ItemType Directory -Path (Join-Path $backup "CalibrationV3Analytic") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $backup "RopeEncoderCalibrationV3") -Force | Out-Null

Copy-Item -LiteralPath $dstDll -Destination (Join-Path $backup "CalibrationV3Analytic\CalibrationV3Analytic.cpp") -Force
Copy-Item -LiteralPath $dstGui -Destination (Join-Path $backup "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp") -Force
Copy-Item -LiteralPath $srcDll -Destination $dstDll -Force
Copy-Item -LiteralPath $srcGui -Destination $dstGui -Force

Write-Host "Backup: $backup" -ForegroundColor DarkYellow
Write-Host "R7.3 source applied." -ForegroundColor Green

$sln = Join-Path $BuildRoot "HE3_RopeCalibration_R7.1_Minimal.sln"
Require-Path $sln "solution"

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
Require-Path $vswhere "vswhere.exe"
$msbuild = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find "MSBuild\**\Bin\MSBuild.exe" | Select-Object -First 1
if ([string]::IsNullOrWhiteSpace($msbuild)) { throw "MSBuild.exe not found." }

Write-Host "Rebuilding Release|x86 ..."
& $msbuild $sln /m /t:Rebuild /p:Configuration=Release /p:Platform=x86
if ($LASTEXITCODE -ne 0) { throw "MSBuild failed. Exit code: $LASTEXITCODE" }

$exe = Join-Path $BuildRoot "bin\Win32\Release\HE3_RopeCalibration_V3_Tool.exe"
$dll = Join-Path $BuildRoot "bin\Win32\Release\CalibrationV3Analytic.dll"
Require-Path $exe "EXE"
Require-Path $dll "DLL"

Write-Host ""
Write-Host "BUILD PASS" -ForegroundColor Green
Write-Host "EXE: $exe"
Write-Host "DLL: $dll"
Write-Host ""
Write-Host "For same-data stability check: connect robot, DO NOT click Start, click the controller replay button." -ForegroundColor Yellow
