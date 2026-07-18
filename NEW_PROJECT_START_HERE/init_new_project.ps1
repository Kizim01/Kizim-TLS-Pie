param(
    [string]$TargetDir = "",
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$templateDir = Join-Path $scriptDir "Templates"

if (-not (Test-Path -LiteralPath $templateDir)) {
    throw "Missing templates folder: $templateDir"
}

if ([string]::IsNullOrWhiteSpace($TargetDir)) {
    $TargetDir = Split-Path -Parent $scriptDir
}

$TargetDir = (Resolve-Path -LiteralPath $TargetDir).Path

Write-Output "Initializing project context files in: $TargetDir"

$filesToCopy = @(
    @{ Source = "AI_PROJECT_RUNBOOK_TEMPLATE.md"; Dest = "AI_PROJECT_RUNBOOK.md" },
    @{ Source = "COPILOT_START_PROMPT_TEMPLATE.txt"; Dest = "COPILOT_START_PROMPT.txt" },
    @{ Source = "PROJECT_CONTEXT_TEMPLATE.md"; Dest = "PROJECT_CONTEXT.md" },
    @{ Source = "AI_HANDOFF_CHANGELOG_TEMPLATE.md"; Dest = "AI_HANDOFF_CHANGELOG.md" },
    @{ Source = "AI_HANDOFF_CHECKLIST_TEMPLATE.md"; Dest = "AI_HANDOFF_CHECKLIST.md" }
)

foreach ($item in $filesToCopy) {
    $src = Join-Path $templateDir $item.Source
    $dst = Join-Path $TargetDir $item.Dest

    if (-not (Test-Path -LiteralPath $src)) {
        throw "Missing source file: $src"
    }

    if ((Test-Path -LiteralPath $dst) -and (-not $Force)) {
        Write-Output "Skip existing: $dst"
        continue
    }

    Copy-Item -LiteralPath $src -Destination $dst -Force
    Write-Output "Wrote: $dst"
}

$startHerePath = Join-Path $TargetDir "START_HERE.md"
if ((-not (Test-Path -LiteralPath $startHerePath)) -or $Force) {
    @"
# Start Here

This repository was initialized with the NEW_PROJECT_START_HERE kit.

## Read first
1. AI_HANDOFF_CHANGELOG.md
2. PROJECT_CONTEXT.md
3. AI_HANDOFF_CHECKLIST.md
4. AI_PROJECT_RUNBOOK.md

## First chat prompt
Use COPILOT_START_PROMPT.txt as the first message in a new chat.

## Operating habit
At the end of each session:
- update AI_HANDOFF_CHANGELOG.md
- update PROJECT_CONTEXT.md
- update AI_HANDOFF_CHECKLIST.md
"@ | Set-Content -LiteralPath $startHerePath -Encoding utf8
    Write-Output "Wrote: $startHerePath"
} else {
    Write-Output "Skip existing: $startHerePath"
}

$resumeBatPath = Join-Path $TargetDir "RESUME_PROJECT.bat"
if ((-not (Test-Path -LiteralPath $resumeBatPath)) -or $Force) {
    @"
@echo off
setlocal

set SCRIPT_DIR=%~dp0
powershell -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%NEW_PROJECT_START_HERE\init_new_project.ps1"

if errorlevel 1 (
  echo.
  echo Resume refresh failed.
  exit /b 1
)

echo.
echo Resume refresh finished.
exit /b 0
"@ | Set-Content -LiteralPath $resumeBatPath -Encoding ascii
    Write-Output "Wrote: $resumeBatPath"
} else {
    Write-Output "Skip existing: $resumeBatPath"
}

Write-Output "Initialization complete."
