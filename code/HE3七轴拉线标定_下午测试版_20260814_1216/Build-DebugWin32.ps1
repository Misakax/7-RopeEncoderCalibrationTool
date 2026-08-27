[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$solutionRoot = $PSScriptRoot
$solutionPath = Join-Path $solutionRoot 'RopeEncoderCalibrationTool.sln'
$dllPath = Join-Path $solutionRoot 'Debug\MathematicalDLL.dll'
$exePath = Join-Path $solutionRoot 'Debug\RopeEncoderCalibrationTool.exe'
$testPath = Join-Path $solutionRoot 'Debug\MathematicalDLLTests.exe'

if (-not (Test-Path -LiteralPath $solutionPath -PathType Leaf)) {
    throw "Solution not found: $solutionPath"
}

$running = Get-Process -Name 'RopeEncoderCalibrationTool' -ErrorAction SilentlyContinue
if ($null -ne $running) {
    $ids = ($running | ForEach-Object { $_.Id }) -join ', '
    throw "RopeEncoderCalibrationTool is running (PID: $ids). Stop debugging/close it before rebuilding; the process locks MathematicalDLL.dll."
}

$vsRootCandidates = @(
    'D:\Program Files\Microsoft Visual Studio\2022\Community',
    'C:\Program Files\Microsoft Visual Studio\2022\Community',
    'C:\Program Files\Microsoft Visual Studio\2022\Professional',
    'C:\Program Files\Microsoft Visual Studio\2022\Enterprise'
)
$vsRoot = $vsRootCandidates | Where-Object { Test-Path -LiteralPath $_ -PathType Container } | Select-Object -First 1
if ([string]::IsNullOrWhiteSpace($vsRoot)) {
    throw 'Visual Studio 2022 installation was not found.'
}

$msbuildPath = Join-Path $vsRoot 'MSBuild\Current\Bin\MSBuild.exe'
if (-not (Test-Path -LiteralPath $msbuildPath -PathType Leaf)) {
    throw "MSBuild was not found: $msbuildPath"
}

$mfcHeader = Get-ChildItem -LiteralPath (Join-Path $vsRoot 'VC\Tools\MSVC') -Filter 'afxwin.h' -Recurse -File -ErrorAction SilentlyContinue |
    Where-Object { $_.FullName -like '*\atlmfc\include\afxwin.h' } |
    Select-Object -First 1
if ($null -eq $mfcHeader) {
    throw 'MFC is not installed. Open Visual Studio Installer -> Modify -> Individual components, then install "C++ MFC for latest v143 build tools (x86 & x64)".'
}

$buildStarted = Get-Date
Write-Host "Building Debug|x86 from: $solutionPath"
& $msbuildPath $solutionPath '/t:Rebuild' '/p:Configuration=Debug' '/p:Platform=x86' '/m'
if ($LASTEXITCODE -ne 0) {
    throw "MSBuild failed with exit code $LASTEXITCODE. Do not choose 'run last successful build' in Visual Studio."
}

foreach ($artifactPath in @($dllPath, $exePath, $testPath)) {
    if (-not (Test-Path -LiteralPath $artifactPath -PathType Leaf)) {
        throw "Expected build artifact was not generated: $artifactPath"
    }
    $artifact = Get-Item -LiteralPath $artifactPath
    if ($artifact.LastWriteTime -lt $buildStarted.AddSeconds(-2)) {
        throw "Artifact timestamp was not refreshed: $artifactPath"
    }
    Write-Host ("Updated: {0}  {1:yyyy-MM-dd HH:mm:ss}  {2} bytes" -f $artifact.FullName, $artifact.LastWriteTime, $artifact.Length)
}

Push-Location $solutionRoot
try {
    & $testPath
    if ($LASTEXITCODE -ne 0) {
        throw "CalibrationV2 offline tests failed with exit code $LASTEXITCODE."
    }
}
finally {
    Pop-Location
}

Write-Host 'Build chain verified. Start Debugging (F5), inspect Output -> Debug, and set a breakpoint in CalibrationV2.'
