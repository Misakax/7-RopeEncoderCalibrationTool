[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$project = Join-Path $root 'MathematicalDLLTests\MathematicalDLLTests.vcxproj'
$testExe = Join-Path $root 'Debug\MathematicalDLLTests.exe'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'

if (-not (Test-Path -LiteralPath $vswhere -PathType Leaf)) {
    throw "vswhere was not found: $vswhere"
}

$msbuild = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find 'MSBuild\**\Bin\MSBuild.exe' |
    Select-Object -First 1
if ([string]::IsNullOrWhiteSpace($msbuild)) {
    throw 'MSBuild was not found.'
}

& $msbuild $project '/t:Rebuild' '/p:Configuration=Debug' '/p:Platform=Win32' '/m:1'
if ($LASTEXITCODE -ne 0) {
    throw "CalibrationV2 test build failed with exit code $LASTEXITCODE."
}

Push-Location $root
try {
    & $testExe
    if ($LASTEXITCODE -ne 0) {
        throw "CalibrationV2 tests failed with exit code $LASTEXITCODE."
    }
}
finally {
    Pop-Location
}
