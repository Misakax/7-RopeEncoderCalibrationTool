param(
    [string]$PatchRoot = "C:\Users\1282\Desktop\拉线编码\R7.2_激光先验两阶段修复",
    [string]$BuildRoot = "C:\Users\1282\Desktop\拉线编码\R7.1-Build"
)

$ErrorActionPreference = "Stop"

function Require-Path([string]$Path, [string]$Name) {
    if (-not (Test-Path $Path)) { throw "缺少 $Name : $Path" }
}

Write-Host "=== 应用 R7.2 两阶段 + 激光统计先验修复 ===" -ForegroundColor Cyan
Write-Host "Patch : $PatchRoot"
Write-Host "Build : $BuildRoot"

Require-Path $PatchRoot "R7.2补丁目录"
Require-Path $BuildRoot "现有R7.1-Build目录"
Require-Path (Join-Path $BuildRoot "HE3_RopeCalibration_R7.1_Minimal.sln") "最小解决方案"

$running = Get-Process "HE3_RopeCalibration_V3_Tool" -ErrorAction SilentlyContinue
if ($running) {
    throw "HE3_RopeCalibration_V3_Tool.exe 仍在运行。请先安全停止机器人并关闭上位机。"
}

$files = @(
    @{ Src = "CalibrationV3Analytic\CalibrationV3Analytic.cpp"; Dst = "CalibrationV3Analytic\CalibrationV3Analytic.cpp" },
    @{ Src = "CalibrationV3Analytic\CalibrationV3Analytic.h";   Dst = "CalibrationV3Analytic\CalibrationV3Analytic.h" },
    @{ Src = "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp"; Dst = "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp" },
    @{ Src = "RopeEncoderCalibrationV3\QKMLinkComm.cpp"; Dst = "RopeEncoderCalibrationV3\QKMLinkComm.cpp" }
)

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$backupRoot = Join-Path $BuildRoot ("backup_before_R7.2_" + $stamp)
New-Item -ItemType Directory -Path $backupRoot -Force | Out-Null

foreach ($item in $files) {
    $src = Join-Path $PatchRoot $item.Src
    $dst = Join-Path $BuildRoot $item.Dst
    Require-Path $src "补丁文件 $($item.Src)"
    Require-Path $dst "目标源码 $($item.Dst)"

    $backup = Join-Path $backupRoot $item.Dst
    New-Item -ItemType Directory -Path (Split-Path $backup -Parent) -Force | Out-Null
    Copy-Item $dst $backup -Force
    Copy-Item $src $dst -Force
    Write-Host "覆盖: $($item.Dst)" -ForegroundColor Yellow
}

$dllCpp = Get-Content (Join-Path $BuildRoot "CalibrationV3Analytic\CalibrationV3Analytic.cpp") -Raw
$guiCpp = Get-Content (Join-Path $BuildRoot "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp") -Raw
if ($dllCpp -notmatch "TWOSTAGE-LASER-PRIOR-20260826-R7.2") {
    throw "DLL源码身份检查失败：不是R7.2。"
}
if ($guiCpp -notmatch "R7.2 Two-Stage \+ Laser Prior") {
    throw "GUI源码身份检查失败：不是R7.2。"
}

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
Require-Path $vswhere "Visual Studio vswhere.exe"
$msbuild = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe | Select-Object -First 1
if (-not $msbuild) { throw "未找到 MSBuild.exe。请确认安装VS2022 C++桌面开发组件。" }

$sln = Join-Path $BuildRoot "HE3_RopeCalibration_R7.1_Minimal.sln"
Write-Host "`n开始重新生成 Release | x86 ..." -ForegroundColor Cyan
& $msbuild $sln /m /t:Rebuild /p:Configuration=Release /p:Platform=x86
if ($LASTEXITCODE -ne 0) {
    throw "MSBuild失败，退出码=$LASTEXITCODE。源码备份在：$backupRoot"
}

$exe = Join-Path $BuildRoot "bin\Win32\Release\HE3_RopeCalibration_V3_Tool.exe"
$dll = Join-Path $BuildRoot "bin\Win32\Release\CalibrationV3Analytic.dll"
Require-Path $exe "R7.2 EXE"
Require-Path $dll "R7.2 DLL"

Write-Host "`n=== R7.2 编译成功 ===" -ForegroundColor Green
Write-Host "EXE: $exe"
Write-Host "DLL: $dll"
Write-Host "备份: $backupRoot"
Write-Host "首次运行请只采集+计算，确认Build ID、候选参数和安全门；确认无误后再保存。" -ForegroundColor Yellow
