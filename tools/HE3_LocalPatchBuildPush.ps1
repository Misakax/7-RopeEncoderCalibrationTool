[CmdletBinding()]
param(
    [string]$PatchPath = "",
    [string]$RepoPath = "",
    [string]$CommitMessage = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version 2.0

function Invoke-Git {
    param([Parameter(Mandatory=$true)][string[]]$Args)
    & git -C $script:RepoRoot @Args
    if ($LASTEXITCODE -ne 0) {
        throw "git $($Args -join ' ') failed with exit code $LASTEXITCODE"
    }
}

function Add-LocalExclude {
    param([string]$Entry)
    $excludePath = Join-Path $script:RepoRoot ".git\info\exclude"
    $excludeDir = Split-Path -Parent $excludePath
    New-Item -ItemType Directory -Path $excludeDir -Force | Out-Null
    if (-not (Test-Path -LiteralPath $excludePath)) {
        New-Item -ItemType File -Path $excludePath -Force | Out-Null
    }
    $existing = @(Get-Content -LiteralPath $excludePath -ErrorAction SilentlyContinue)
    if ($existing -notcontains $Entry) {
        Add-Content -LiteralPath $excludePath -Value $Entry -Encoding UTF8
    }
}

function Save-FailureAndRestore {
    param(
        [string]$Phase,
        [string]$Reason,
        [string]$LogDir
    )
    $failureDir = Join-Path $script:OutputRoot ("failures\" + (Get-Date -Format "yyyyMMdd_HHmmss") + "_" + $Phase)
    New-Item -ItemType Directory -Path $failureDir -Force | Out-Null
    $diffPath = Join-Path $failureDir "working-tree.patch"
    $diffLines = @(& git -C $script:RepoRoot diff --binary)
    [System.IO.File]::WriteAllLines($diffPath, $diffLines, [System.Text.UTF8Encoding]::new($false))
    if ($PatchPath -and (Test-Path -LiteralPath $PatchPath)) {
        Copy-Item -LiteralPath $PatchPath -Destination $failureDir -Force
    }
    if ($LogDir -and (Test-Path -LiteralPath $LogDir)) {
        Copy-Item -LiteralPath $LogDir -Destination (Join-Path $failureDir "logs") -Recurse -Force
    }
    Set-Content -LiteralPath (Join-Path $failureDir "FAILURE.txt") -Encoding UTF8 -Value @(
        "phase=$Phase",
        "reason=$Reason",
        "base_sha=$script:BaseSha",
        "time=$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
    )

    Write-Host ""
    Write-Host "BUILD FAILED - no commit/push was performed." -ForegroundColor Red
    Write-Host "Failure package: $failureDir" -ForegroundColor Yellow
    Write-Host "Restoring working tree to $script:BaseSha ..."
    & git -C $script:RepoRoot reset --hard HEAD | Out-Host
    & git -C $script:RepoRoot clean -fd | Out-Host
    throw $Reason
}

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    throw "git.exe not found in PATH."
}

if ([string]::IsNullOrWhiteSpace($RepoPath)) {
    $RepoPath = Split-Path -Parent $PSScriptRoot
}
$script:RepoRoot = (Resolve-Path -LiteralPath $RepoPath).Path
if (-not (Test-Path -LiteralPath (Join-Path $script:RepoRoot ".git"))) {
    throw "Not a Git repository: $script:RepoRoot"
}
$originUrl = (& git -C $script:RepoRoot remote get-url origin).Trim()
if ($LASTEXITCODE -ne 0 -or
    ($originUrl -notmatch 'Misakax/7-RopeEncoderCalibrationTool(\.git)?$' -and
     $originUrl -notmatch 'Misakax:7-RopeEncoderCalibrationTool(\.git)?$')) {
    throw "Unexpected origin remote: '$originUrl'. Expected Misakax/7-RopeEncoderCalibrationTool."
}

$script:OutputRoot = Join-Path $script:RepoRoot "_local_build"
Add-LocalExclude "_local_build/"

if ($PatchPath) {
    $PatchPath = (Resolve-Path -LiteralPath $PatchPath).Path
    if ($PatchPath.StartsWith($script:RepoRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        $relativePatch = $PatchPath.Substring($script:RepoRoot.Length).TrimStart([char[]]@('\','/')).Replace('\','/')
        if ($relativePatch) { Add-LocalExclude $relativePatch }
    }
}

$branch = (& git -C $script:RepoRoot branch --show-current).Trim()
if ($branch -ne "main") {
    throw "R7 local build tool requires branch 'main'. Current branch: '$branch'"
}

if ($PatchPath) {
    $dirty = @(& git -C $script:RepoRoot status --porcelain --untracked-files=normal)
    if ($dirty.Count -gt 0) {
        Write-Host ($dirty -join [Environment]::NewLine)
        throw "Working tree is not clean. Commit/stash/revert local changes before running the patch tool."
    }

    Invoke-Git @("fetch", "origin", "main")
    Invoke-Git @("pull", "--ff-only", "origin", "main")
    $script:BaseSha = (& git -C $script:RepoRoot rev-parse HEAD).Trim()
    Write-Host "Base SHA: $script:BaseSha"

    Write-Host "Checking patch: $PatchPath"
    & git -C $script:RepoRoot apply --check --whitespace=nowarn $PatchPath
    if ($LASTEXITCODE -ne 0) {
        throw "git apply --check failed. Patch was not applied."
    }
    & git -C $script:RepoRoot apply --whitespace=nowarn $PatchPath
    if ($LASTEXITCODE -ne 0) {
        throw "git apply failed."
    }
} else {
    # First-use/manual mode: the patch is already applied, so the tree is expected
    # to be dirty. Fetch is safe; refuse to build if HEAD is not the current main.
    Invoke-Git @("fetch", "origin", "main")
    $script:BaseSha = (& git -C $script:RepoRoot rev-parse HEAD).Trim()
    $remoteSha = (& git -C $script:RepoRoot rev-parse origin/main).Trim()
    if ($script:BaseSha -ne $remoteSha) {
        throw "Local HEAD ($script:BaseSha) is not origin/main ($remoteSha). Update main before applying the patch."
    }
    Write-Host "Base SHA: $script:BaseSha (patch already applied locally)"
}

$patchChangedPaths = @()
$patchChangedPaths += @(& git -C $script:RepoRoot diff --name-only)
$patchChangedPaths += @(& git -C $script:RepoRoot ls-files --others --exclude-standard)
$patchChangedPaths = @($patchChangedPaths | Where-Object { $_ -and $_ -ne "HE3_LOCAL_BUILD_STATUS.txt" } | Sort-Object -Unique)
if ($patchChangedPaths.Count -eq 0) {
    throw "No source changes found after patch application."
}

& git -C $script:RepoRoot diff --check
if ($LASTEXITCODE -ne 0) {
    Save-FailureAndRestore -Phase "diffcheck" -Reason "git diff --check failed" -LogDir ""
}

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$logDir = Join-Path $script:OutputRoot ("logs\" + $stamp)
New-Item -ItemType Directory -Path $logDir -Force | Out-Null

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vswhere)) {
    Save-FailureAndRestore -Phase "toolchain" -Reason "vswhere.exe not found; Visual Studio 2022 Build Tools/IDE is required." -LogDir $logDir
}
$msbuild = @(& $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find "MSBuild\**\Bin\MSBuild.exe") | Select-Object -First 1
if (-not $msbuild) {
    Save-FailureAndRestore -Phase "toolchain" -Reason "MSBuild.exe not found via vswhere." -LogDir $logDir
}
Write-Host "MSBuild: $msbuild"

$testProject = Join-Path $script:RepoRoot "tools\tests\R74DiagnosticMatchTest.vcxproj"
if (Test-Path -LiteralPath $testProject) {
    $testOut = Join-Path $script:OutputRoot "test\bin"
    $testInt = Join-Path $script:OutputRoot "test\obj"
    New-Item -ItemType Directory -Path $testOut, $testInt -Force | Out-Null
    $testLog = Join-Path $logDir "R74DiagnosticMatchTest.msbuild.log"
    $testBinlog = Join-Path $logDir "R74DiagnosticMatchTest.binlog"
    & $msbuild $testProject /m /t:Rebuild /p:Configuration=Release /p:Platform=Win32 "/p:OutDir=$testOut\" "/p:IntDir=$testInt\" /nologo /verbosity:minimal /fl "/flp:logfile=$testLog;verbosity=diagnostic" "/bl:$testBinlog"
    if ($LASTEXITCODE -ne 0) {
        Save-FailureAndRestore -Phase "r74-test-build" -Reason "R74DiagnosticMatchTest build failed." -LogDir $logDir
    }
    $testExe = Join-Path $testOut "R74DiagnosticMatchTest.exe"
    if (-not (Test-Path -LiteralPath $testExe)) {
        Save-FailureAndRestore -Phase "r74-test-build" -Reason "R74DiagnosticMatchTest.exe was not produced." -LogDir $logDir
    }
    $testRunLog = Join-Path $logDir "R74DiagnosticMatchTest.run.log"
    & $testExe *>&1 | Tee-Object -FilePath $testRunLog
    if ($LASTEXITCODE -ne 0) {
        Save-FailureAndRestore -Phase "r74-test-run" -Reason "R74DiagnosticMatchTest failed." -LogDir $logDir
    }
}

$solution = Join-Path $script:RepoRoot "R7.1-Build\HE3_RopeCalibration_R7.1_Minimal.sln"
if (-not (Test-Path -LiteralPath $solution)) {
    Save-FailureAndRestore -Phase "main-build" -Reason "Solution not found: $solution" -LogDir $logDir
}
$mainLog = Join-Path $logDir "msbuild.log"
$mainBinlog = Join-Path $logDir "msbuild.binlog"
& $msbuild $solution /m /t:Rebuild /p:Configuration=Release /p:Platform=x86 /nologo /verbosity:minimal /fl "/flp:logfile=$mainLog;verbosity=diagnostic" "/bl:$mainBinlog"
if ($LASTEXITCODE -ne 0) {
    Save-FailureAndRestore -Phase "main-build" -Reason "HE3 Release|x86 MSBuild failed." -LogDir $logDir
}

$exe = Join-Path $script:RepoRoot "R7.1-Build\bin\Win32\Release\HE3_RopeCalibration_V3_Tool.exe"
$dll = Join-Path $script:RepoRoot "R7.1-Build\bin\Win32\Release\CalibrationV3Analytic.dll"
$config = Join-Path $script:RepoRoot "R7.1-Build\RopeEncoderCalibrationV3\Config"
$robotCfg = Join-Path $config "RobotType\HE3_GY-cfg.txt"
$location = Join-Path $config "RobotLocation\HE3_GY.txt"
foreach ($required in @($exe, $dll, $config, $robotCfg, $location)) {
    if (-not (Test-Path -LiteralPath $required)) {
        Save-FailureAndRestore -Phase "runtime-verify" -Reason "Missing runtime input: $required" -LogDir $logDir
    }
}
if (-not (Select-String -LiteralPath $robotCfg -SimpleMatch "[Robot Axis] = 7" -Quiet)) {
    Save-FailureAndRestore -Phase "runtime-verify" -Reason "HE3_GY-cfg.txt is not 7-axis." -LogDir $logDir
}
if (-not (Select-String -LiteralPath $robotCfg -SimpleMatch "[Calbration location number] = 82" -Quiet)) {
    Save-FailureAndRestore -Phase "runtime-verify" -Reason "HE3_GY-cfg.txt does not declare 82 poses." -LogDir $logDir
}
$poseLines = @(Get-Content -LiteralPath $location | Where-Object {
    $line = $_.Trim()
    $line -and -not $line.StartsWith("'ConfigVersion=")
})
if ($poseLines.Count -ne 82) {
    Save-FailureAndRestore -Phase "runtime-verify" -Reason "HE3_GY.txt has $($poseLines.Count) poses; expected 82." -LogDir $logDir
}

$latest = Join-Path $script:OutputRoot "latest"
Remove-Item -LiteralPath $latest -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Path $latest -Force | Out-Null
Copy-Item -LiteralPath $exe, $dll -Destination $latest -Force
Copy-Item -LiteralPath $config -Destination (Join-Path $latest "Config") -Recurse -Force
foreach ($dir in @("Result", "data", "log", "Joint")) {
    New-Item -ItemType Directory -Path (Join-Path $latest $dir) -Force | Out-Null
}
@"
@echo off
cd /d "%~dp0"
start "" "HE3_RopeCalibration_V3_Tool.exe"
"@ | Set-Content -LiteralPath (Join-Path $latest "Run_HE3_R7.4.bat") -Encoding ASCII

$buildTime = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
$statusFile = Join-Path $script:RepoRoot "HE3_LOCAL_BUILD_STATUS.txt"
@"
status=PASS
version=R7.4
built_at=$buildTime
source_base_sha=$script:BaseSha
configuration=Release|x86
diagnostic_match_test=PASS
robot_axis=7
calibration_poses=82
exe=R7.1-Build/bin/Win32/Release/HE3_RopeCalibration_V3_Tool.exe
logs=_local_build/logs/$stamp
note=This source tree was compiled locally before commit/push.
"@ | Set-Content -LiteralPath $statusFile -Encoding UTF8

$stagePaths = @($patchChangedPaths + @("HE3_LOCAL_BUILD_STATUS.txt"))
$stagePaths = @($stagePaths | Sort-Object -Unique)
& git -C $script:RepoRoot add -- @stagePaths
if ($LASTEXITCODE -ne 0) {
    Save-FailureAndRestore -Phase "git-stage" -Reason "git add failed." -LogDir $logDir
}
& git -C $script:RepoRoot diff --cached --check
if ($LASTEXITCODE -ne 0) {
    Save-FailureAndRestore -Phase "git-stage" -Reason "git diff --cached --check failed." -LogDir $logDir
}

if ([string]::IsNullOrWhiteSpace($CommitMessage)) {
    $patchLabel = if ($PatchPath) { [System.IO.Path]::GetFileNameWithoutExtension($PatchPath) } else { "local-source-update" }
    $CommitMessage = "HE3: $patchLabel (local Release x86 PASS)"
}
& git -C $script:RepoRoot commit -m $CommitMessage
if ($LASTEXITCODE -ne 0) {
    Save-FailureAndRestore -Phase "git-commit" -Reason "git commit failed." -LogDir $logDir
}
$commitSha = (& git -C $script:RepoRoot rev-parse HEAD).Trim()

@"
Version=R7.4
Commit=$commitSha
BuiltAt=$buildTime
Build=Release|x86
DiagnosticMatchTest=PASS
HE3_RobotAxis=7
HE3_CalibrationPoses=82
SourceBase=$script:BaseSha
"@ | Set-Content -LiteralPath (Join-Path $latest "BUILD_INFO.txt") -Encoding UTF8

& git -C $script:RepoRoot push origin main
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "LOCAL BUILD PASSED and commit exists locally, but push failed." -ForegroundColor Yellow
    Write-Host "Commit: $commitSha"
    Write-Host "Run: git push origin main"
    throw "git push failed; successful local build commit was preserved."
}

Write-Host ""
Write-Host "R7.4 LOCAL BUILD PASS" -ForegroundColor Green
Write-Host "Commit pushed: $commitSha"
Write-Host "EXE: $(Join-Path $latest 'HE3_RopeCalibration_V3_Tool.exe')"
Write-Host "Logs: $logDir"
Write-Host "GitHub is now the source/version truth for the next ChatGPT turn."
