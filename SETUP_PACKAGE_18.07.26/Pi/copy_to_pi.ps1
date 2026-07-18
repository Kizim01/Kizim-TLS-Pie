# PowerShell helper for copying the TLS_Pie project files into the Pi setup package.
# Run this from the repo root on your PC before copying the package to the Pi.

$repo = Get-Location
$package = Join-Path $repo 'Pi_Setup_Package'
$target = Join-Path $package 'TLS-Pie'

New-Item -ItemType Directory -Force -Path $target | Out-Null

$items = @(
    'Raspberry Pie4',
    'Arduino Microview',
    'README.md',
    'PROJECT_CONTEXT.md',
    'AI_HANDOFF_CHANGELOG.md',
    'AI_HANDOFF_CHECKLIST.md',
    'BENCH_TEST_README.md',
    'CHANGELOG_AND_TEST_GUIDE.md'
)

foreach ($item in $items) {
    $src = Join-Path $repo $item
    $dest = Join-Path $target $item
    if (Test-Path $src) {
        Copy-Item $src $dest -Recurse -Force
    }
}

Write-Output "Prepared $target"
