param(
    [string]$BuildRoot = ""
)

$ErrorActionPreference = "Stop"

function Require-Path([string]$Path, [string]$Label) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Missing $Label : $Path"
    }
}

Write-Host "=== R7.2.2 final patch apply and build ===" -ForegroundColor Cyan

$FixRoot = $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($BuildRoot)) {
    $WorkspaceRoot = Split-Path -Parent $FixRoot
    $BuildRoot = Join-Path $WorkspaceRoot "R7.1-Build"
}

$srcDll = Join-Path $FixRoot "CalibrationV3Analytic\CalibrationV3Analytic.cpp"
$srcGui = Join-Path $FixRoot "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp"
$dstDll = Join-Path $BuildRoot "CalibrationV3Analytic\CalibrationV3Analytic.cpp"
$dstGui = Join-Path $BuildRoot "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp"

Require-Path $BuildRoot "build tree"
Require-Path $srcDll "R7.2.2 DLL source"
Require-Path $srcGui "R7.2.2 GUI source"
Require-Path $dstDll "target DLL source"
Require-Path $dstGui "target GUI source"

$dllText = Get-Content -LiteralPath $srcDll -Raw
$guiText = Get-Content -LiteralPath $srcGui -Raw
if ($dllText -notmatch "R7\.2\.2") { throw "DLL identity check failed." }
if ($guiText -notmatch "R7\.2\.2") { throw "GUI identity check failed." }

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$backup = Join-Path $BuildRoot ("backup_before_R7.2.2_" + $stamp)
New-Item -ItemType Directory -Path (Join-Path $backup "CalibrationV3Analytic") -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $backup "RopeEncoderCalibrationV3") -Force | Out-Null

Copy-Item -LiteralPath $dstDll -Destination (Join-Path $backup "CalibrationV3Analytic\CalibrationV3Analytic.cpp") -Force
Copy-Item -LiteralPath $dstGui -Destination (Join-Path $backup "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp") -Force

Copy-Item -LiteralPath $srcDll -Destination $dstDll -Force
Copy-Item -LiteralPath $srcGui -Destination $dstGui -Force

Write-Host "Backup: $backup" -ForegroundColor DarkYellow
Write-Host "R7.2.2 source applied." -ForegroundColor Green

# Contract preflight on the applied target.
$targetGui = Get-Content -LiteralPath $dstGui -Raw
if ($targetGui -match "candidateD\[5\]\s*=\s*0") {
    throw "Contract check failed: D6 still appears as calibration candidate."
}
if ($targetGui -notmatch "d6_is_rope_calibration_parameter,0") {
    throw "Contract check failed: D6 role marker missing."
}
if ($targetGui -notmatch "d7_written,0") {
    throw "Contract check failed: D7 no-write marker missing."
}

$sln = Join-Path $BuildRoot "HE3_RopeCalibration_R7.1_Minimal.sln"
Require-Path $sln "solution"

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
Require-Path $vswhere "vswhere.exe"

$msbuild = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find "MSBuild\**\Bin\MSBuild.exe" | Select-Object -First 1
if ([string]::IsNullOrWhiteSpace($msbuild)) {
    throw "MSBuild.exe not found."
}

Write-Host "Rebuilding Release|x86 ..."
& $msbuild $sln /m /t:Rebuild /p:Configuration=Release /p:Platform=x86
if ($LASTEXITCODE -ne 0) {
    throw "MSBuild failed. Exit code: $LASTEXITCODE"
}

$exe = Join-Path $BuildRoot "bin\Win32\Release\HE3_RopeCalibration_V3_Tool.exe"
$dll = Join-Path $BuildRoot "bin\Win32\Release\CalibrationV3Analytic.dll"
Require-Path $exe "EXE"
Require-Path $dll "DLL"

Write-Host ""
Write-Host "BUILD PASS" -ForegroundColor Green
Write-Host "EXE: $exe"
Write-Host "DLL: $dll"
Write-Host ""
Write-Host "Final contract: rope calibration writes D2-D5/Alpha1-6/q2-6 only; D6 structural repair is separate; D7 is never written." -ForegroundColor Yellow
