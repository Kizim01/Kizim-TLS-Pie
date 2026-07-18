# Build and verify a clean Pi setup package zip from the current repo state.
# Run from the repo root:
#   powershell -ExecutionPolicy Bypass -File .\Pi_Setup_Package\build_package.ps1

$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$packageRoot = Join-Path $repoRoot 'Pi_Setup_Package'
$bundleRoot = Join-Path $packageRoot 'TLS-Pie'

Write-Output "Repo: $repoRoot"
Write-Output "Package: $packageRoot"

# 1) Sync key docs and runtime scripts into the package root.
$rootFiles = @(
    'Raspberry Pie4/TLS_Pie_Pi_Setup_Checklist.pdf',
    'Raspberry Pie4/TLS_Pie_Pi_Setup_Guide.pdf',
    'Raspberry Pie4/TLS_Pie_Pi_Setup_Checklist.md',
    'Raspberry Pie4/TLS_Pie_Pi_Setup_Guide.md',
    'Pi_Setup_Package/setup_tls_pie_pi.sh',
    'Pi_Setup_Package/README.txt'
)

foreach ($f in $rootFiles) {
    $src = Join-Path $repoRoot $f
    if (-not (Test-Path $src)) { continue }
    $dest = if ($f -like 'Pi_Setup_Package/*') {
        Join-Path $packageRoot ([IO.Path]::GetFileName($f))
    } else {
        Join-Path $packageRoot ([IO.Path]::GetFileName($f))
    }
    if ((Resolve-Path $src).Path -eq (Resolve-Path $dest -ErrorAction SilentlyContinue).Path) {
        continue
    }
    Copy-Item $src $dest -Force
}

# 2) Build bundled project folder expected by installer.
if (Test-Path $bundleRoot) { Remove-Item $bundleRoot -Recurse -Force }
New-Item -ItemType Directory -Path $bundleRoot | Out-Null

$bundleItems = @(
    'Raspberry Pie4',
    'Arduino Microview',
    'README.md',
    'PROJECT_CONTEXT.md',
    'AI_HANDOFF_CHANGELOG.md',
    'AI_HANDOFF_CHECKLIST.md',
    'BENCH_TEST_README.md',
    'CHANGELOG_AND_TEST_GUIDE.md'
)

foreach ($item in $bundleItems) {
    $src = Join-Path $repoRoot $item
    if (Test-Path $src) {
        Copy-Item $src (Join-Path $bundleRoot $item) -Recurse -Force
    }
}

# 3) Sync latest runtime scripts and self-check into bundle.
$scriptPairs = @(
    @('Raspberry Pie4/TLS-Pie/VLPrecord.sh','Pi_Setup_Package/TLS-Pie/Raspberry Pie4/TLS-Pie/VLPrecord.sh'),
    @('Raspberry Pie4/TLS-Pie/VLPbuttons.py','Pi_Setup_Package/TLS-Pie/Raspberry Pie4/TLS-Pie/VLPbuttons.py'),
    @('Raspberry Pie4/TLS-Pie/VLPwaitbutton.py','Pi_Setup_Package/TLS-Pie/Raspberry Pie4/TLS-Pie/VLPwaitbutton.py'),
    @('Raspberry Pie4/TLS-Pie/VLPselfcheck.sh','Pi_Setup_Package/TLS-Pie/Raspberry Pie4/TLS-Pie/VLPselfcheck.sh'),
    @('Raspberry Pie4/TLS-Pie/VLPstatussignal.py','Pi_Setup_Package/TLS-Pie/Raspberry Pie4/TLS-Pie/VLPstatussignal.py')
)

foreach ($pair in $scriptPairs) {
    Copy-Item (Join-Path $repoRoot $pair[0]) (Join-Path $repoRoot $pair[1]) -Force
}

# 4) Verify sync lock by hash equality.
foreach ($pair in $scriptPairs) {
    $h1 = (Get-FileHash (Join-Path $repoRoot $pair[0]) -Algorithm SHA256).Hash
    $h2 = (Get-FileHash (Join-Path $repoRoot $pair[1]) -Algorithm SHA256).Hash
    if ($h1 -ne $h2) {
        throw "Sync mismatch: $($pair[0]) != $($pair[1])"
    }
}
Write-Output 'Sync lock check passed.'

# 5) Create distributable zip.
$zipPath = Join-Path $repoRoot 'Pi_Setup_Package.zip'
if (Test-Path $zipPath) { Remove-Item $zipPath -Force }
Compress-Archive -Path (Join-Path $packageRoot '*') -DestinationPath $zipPath -Force
Write-Output "Created $zipPath"
