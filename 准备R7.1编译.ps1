param(
    [string]$OriRoot = "C:\Users\1282\Desktop\拉线编码\ORI-RopeEncoderCalibrationTool",
    [string]$R71Root = "C:\Users\1282\Desktop\拉线编码\V3-R7.1-交付",
    [string]$Dest = "C:\Users\1282\Desktop\拉线编码\R7.1-Build",
    [switch]$Build
)

$ErrorActionPreference = "Stop"

function Require-Path($p, $desc) {
    if (-not (Test-Path $p)) {
        throw "缺少 $desc : $p"
    }
}

Write-Host "=== R7.1 build tree preparation ===" -ForegroundColor Cyan
Write-Host "ORI  : $OriRoot"
Write-Host "R7.1 : $R71Root"
Write-Host "Dest : $Dest"

Require-Path $OriRoot "ORI 根目录"
Require-Path (Join-Path $OriRoot "RopeEncoderCalibration") "ORI 主程序源码目录"
Require-Path (Join-Path $R71Root "source") "R7.1 source 目录"
Require-Path (Join-Path $R71Root "Config\RobotType\HE3_GY-cfg.txt") "R7.1 公共 HE3_GY-cfg.txt"

if (Test-Path $Dest) {
    Write-Host "清理旧目标目录..." -ForegroundColor Yellow
    Remove-Item $Dest -Recurse -Force
}
New-Item -ItemType Directory -Path $Dest | Out-Null

# 1) Copy the R7.1 source tree first. These files must win over ORI.
Copy-Item (Join-Path $R71Root "source\CalibrationV3Analytic") $Dest -Recurse
Copy-Item (Join-Path $R71Root "source\RopeEncoderCalibrationV3") $Dest -Recurse

# 2) Fill only the GUI files missing from the reduced R7/R7.1 handoff using ORI.
$oriGui = Join-Path $OriRoot "RopeEncoderCalibration"
$dstGui = Join-Path $Dest "RopeEncoderCalibrationV3"

$fillFiles = @(
    "RopeEncoderCalibration.cpp",
    "RopeEncoderCalibration.h",
    "RoughCalibrationDlg.cpp",
    "RoughCalibrationDlg.h",
    "stdafx.cpp",
    "stdafx.h",
    "targetver.h",
    "qkmlinklib_i.c",
    "qkmlinklib_i.h",
    "websocket_endpoint.cpp",
    "websocket_endpoint.h",
    "ReadMe.txt",
    "ClassDiagram.cd"
)

foreach ($f in $fillFiles) {
    $src = Join-Path $oriGui $f
    $dst = Join-Path $dstGui $f
    if (Test-Path $src) {
        Copy-Item $src $dst -Force
        Write-Host "补齐: $f"
    } elseif ($f -in @("ReadMe.txt","ClassDiagram.cd")) {
        Write-Host "可选文件未找到: $f" -ForegroundColor DarkYellow
    } else {
        throw "ORI 中缺少必须文件: $src"
    }
}

# Resource files referenced by the R7.1 .rc
Require-Path (Join-Path $oriGui "res") "ORI res 资源目录"
Copy-Item (Join-Path $oriGui "res") $dstGui -Recurse -Force

# 3) Put the public R7.1 Config where the GUI post-build step expects it.
$dstConfig = Join-Path $dstGui "Config"
New-Item -ItemType Directory -Path $dstConfig -Force | Out-Null
Copy-Item (Join-Path $R71Root "Config\*") $dstConfig -Recurse -Force

# 4) Third-party dependencies.
foreach ($dep in @("eigen3","asio-1.12.2")) {
    $src = Join-Path $OriRoot $dep
    Require-Path $src "第三方依赖 $dep"
    Copy-Item $src (Join-Path $Dest $dep) -Recurse -Force
}

# websocketpp may be in ORI root or elsewhere under 拉线编码.
$wsCandidates = @()
$direct = Join-Path $OriRoot "websocketpp-master"
if (Test-Path $direct) { $wsCandidates += Get-Item $direct }

$searchRoot = Split-Path $OriRoot -Parent
if ($wsCandidates.Count -eq 0) {
    $wsCandidates = @(Get-ChildItem $searchRoot -Directory -Recurse -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -like "websocketpp*" })
}
if ($wsCandidates.Count -eq 0) {
    throw "未找到 websocketpp 目录。请把原工程实际使用的 websocketpp-master 放到: $Dest\websocketpp-master"
}
$ws = $wsCandidates[0].FullName
Copy-Item $ws (Join-Path $Dest "websocketpp-master") -Recurse -Force
Write-Host "websocketpp: $ws"

# 5) Create a minimal solution with only the two projects that actually exist.
$sln = @'
Microsoft Visual Studio Solution File, Format Version 12.00
# Visual Studio Version 17
VisualStudioVersion = 17.0.31903.59
MinimumVisualStudioVersion = 10.0.40219.1
Project("{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942}") = "CalibrationV3Analytic", "CalibrationV3Analytic\CalibrationV3Analytic.vcxproj", "{A13E38E0-45A7-4CF5-A2E4-BAA2A257A301}"
EndProject
Project("{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942}") = "HE3_RopeCalibration_V3_Tool", "RopeEncoderCalibrationV3\RopeEncoderCalibration.vcxproj", "{E5675644-8789-4150-821E-4912F1C4DBE8}"
EndProject
Global
    GlobalSection(SolutionConfigurationPlatforms) = preSolution
        Debug|x64 = Debug|x64
        Debug|x86 = Debug|x86
        Release|x64 = Release|x64
        Release|x86 = Release|x86
    EndGlobalSection
    GlobalSection(ProjectConfigurationPlatforms) = postSolution
        {A13E38E0-45A7-4CF5-A2E4-BAA2A257A301}.Debug|x64.ActiveCfg = Debug|x64
        {A13E38E0-45A7-4CF5-A2E4-BAA2A257A301}.Debug|x64.Build.0 = Debug|x64
        {A13E38E0-45A7-4CF5-A2E4-BAA2A257A301}.Debug|x86.ActiveCfg = Debug|Win32
        {A13E38E0-45A7-4CF5-A2E4-BAA2A257A301}.Debug|x86.Build.0 = Debug|Win32
        {A13E38E0-45A7-4CF5-A2E4-BAA2A257A301}.Release|x64.ActiveCfg = Release|x64
        {A13E38E0-45A7-4CF5-A2E4-BAA2A257A301}.Release|x64.Build.0 = Release|x64
        {A13E38E0-45A7-4CF5-A2E4-BAA2A257A301}.Release|x86.ActiveCfg = Release|Win32
        {A13E38E0-45A7-4CF5-A2E4-BAA2A257A301}.Release|x86.Build.0 = Release|Win32
        {E5675644-8789-4150-821E-4912F1C4DBE8}.Debug|x64.ActiveCfg = Debug|x64
        {E5675644-8789-4150-821E-4912F1C4DBE8}.Debug|x64.Build.0 = Debug|x64
        {E5675644-8789-4150-821E-4912F1C4DBE8}.Debug|x86.ActiveCfg = Debug|Win32
        {E5675644-8789-4150-821E-4912F1C4DBE8}.Debug|x86.Build.0 = Debug|Win32
        {E5675644-8789-4150-821E-4912F1C4DBE8}.Release|x64.ActiveCfg = Release|x64
        {E5675644-8789-4150-821E-4912F1C4DBE8}.Release|x64.Build.0 = Release|x64
        {E5675644-8789-4150-821E-4912F1C4DBE8}.Release|x86.ActiveCfg = Release|Win32
        {E5675644-8789-4150-821E-4912F1C4DBE8}.Release|x86.Build.0 = Release|Win32
    EndGlobalSection
    GlobalSection(SolutionProperties) = preSolution
        HideSolutionNode = FALSE
    EndGlobalSection
EndGlobal
'@
$slnPath = Join-Path $Dest "HE3_RopeCalibration_R7.1_Minimal.sln"
Set-Content -Path $slnPath -Value $sln -Encoding UTF8

# 6) Preflight checks: all files referenced by the GUI vcxproj that are required for build.
$required = @(
    "RopeEncoderCalibrationV3\FileOperation.h",
    "RopeEncoderCalibrationV3\QKMLinkComm.h",
    "RopeEncoderCalibrationV3\qkmlinklib_i.h",
    "RopeEncoderCalibrationV3\resource.h",
    "RopeEncoderCalibrationV3\RobotConfig.h",
    "RopeEncoderCalibrationV3\RopeEncoderCalibration.h",
    "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.h",
    "RopeEncoderCalibrationV3\RoughCalibrationDlg.h",
    "RopeEncoderCalibrationV3\stdafx.h",
    "RopeEncoderCalibrationV3\targetver.h",
    "RopeEncoderCalibrationV3\websocket_endpoint.h",
    "RopeEncoderCalibrationV3\FileOperation.cpp",
    "RopeEncoderCalibrationV3\QKMLinkComm.cpp",
    "RopeEncoderCalibrationV3\qkmlinklib_i.c",
    "RopeEncoderCalibrationV3\RopeEncoderCalibration.cpp",
    "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp",
    "RopeEncoderCalibrationV3\RoughCalibrationDlg.cpp",
    "RopeEncoderCalibrationV3\stdafx.cpp",
    "RopeEncoderCalibrationV3\websocket_endpoint.cpp",
    "RopeEncoderCalibrationV3\RopeEncoderCalibration.rc",
    "RopeEncoderCalibrationV3\res\RopeEncoderCalibration.rc2",
    "RopeEncoderCalibrationV3\res\RopeEncoderCalibration.ico",
    "CalibrationV3Analytic\CalibrationV3Analytic.cpp",
    "CalibrationV3Analytic\CalibrationV3Analytic.h",
    "eigen3\Eigen\Dense",
    "asio-1.12.2\include\asio.hpp",
    "websocketpp-master\websocketpp\config\asio_no_tls_client.hpp",
    "RopeEncoderCalibrationV3\Config\RobotType\HE3_GY-cfg.txt"
)

$missing = @()
foreach ($r in $required) {
    if (-not (Test-Path (Join-Path $Dest $r))) { $missing += $r }
}

if ($missing.Count -gt 0) {
    Write-Host "`n预检失败，仍缺:" -ForegroundColor Red
    $missing | ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
    throw "R7.1 编译树未补齐。"
}

# Confirm R7.1 identity so an R7 file did not overwrite it.
$dllCpp = Get-Content (Join-Path $Dest "CalibrationV3Analytic\CalibrationV3Analytic.cpp") -Raw
$guiCpp = Get-Content (Join-Path $Dest "RopeEncoderCalibrationV3\RopeEncoderCalibrationDlg.cpp") -Raw
if ($dllCpp -notmatch "R7\.1-PUBLIC-MDH-ROPE-BIAS") {
    throw "CalibrationV3Analytic.cpp 不是 R7.1 版本。"
}
if ($guiCpp -notmatch "R7\.1 Public MDH \+ Rope Bias") {
    throw "RopeEncoderCalibrationDlg.cpp 不是 R7.1 版本。"
}

Write-Host "`n预检 PASS：R7.1 源码、公共配置和依赖已组装完成。" -ForegroundColor Green
Write-Host "解决方案: $slnPath"
Write-Host "目标配置: Release | x86"

if ($Build) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    Require-Path $vswhere "Visual Studio vswhere"
    $msbuild = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe | Select-Object -First 1
    if (-not $msbuild) { throw "未找到 MSBuild.exe" }

    Write-Host "`n开始编译 Release|x86 ..." -ForegroundColor Cyan
    & $msbuild $slnPath /m /t:Rebuild /p:Configuration=Release /p:Platform=x86
    if ($LASTEXITCODE -ne 0) {
        throw "MSBuild 失败，退出码 $LASTEXITCODE"
    }

    $exe = Join-Path $Dest "bin\Win32\Release\HE3_RopeCalibration_V3_Tool.exe"
    $dll = Join-Path $Dest "bin\Win32\Release\CalibrationV3Analytic.dll"
    Require-Path $exe "R7.1 EXE"
    Require-Path $dll "R7.1 DLL"
    Write-Host "`n编译成功:" -ForegroundColor Green
    Write-Host "  $exe"
    Write-Host "  $dll"
} else {
    Write-Host "`n下一步：双击上面的 .sln，在 VS2022 选 Release | x86 后 Rebuild Solution。" -ForegroundColor Yellow
    Write-Host "也可直接重跑本脚本并加 -Build 自动调用 MSBuild。"
}
