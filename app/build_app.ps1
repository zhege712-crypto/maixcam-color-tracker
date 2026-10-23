# Regenerate the MaixCAM app package from the single source file.
# Usage:  powershell -ExecutionPolicy Bypass -File app\build_app.ps1
#
# app/color_tracker/main.py is a COPY of camera/maixcam_color_tracker.py.
# The single source of truth is camera\maixcam_color_tracker.py -- edit that,
# then run this script so the app package stays in sync.
#
# NOTE: keep this file ASCII-only. Windows PowerShell 5.1 reads .ps1 as ANSI,
# so non-ASCII text here can break parsing.
#
# Deploy to the device (needs the password, so do it by hand):
#   scp -r app\color_tracker root@10.171.126.1:/maixapp/apps/
#   ssh -n root@10.171.126.1 "printf color_tracker > /maixapp/auto_start.txt"
#   ssh -n root@10.171.126.1 "sync ; sleep 1 ; /sbin/reboot -f"
# Disable auto start again:
#   ssh -n root@10.171.126.1 "rm -f /maixapp/auto_start.txt"

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$src = Join-Path $root 'camera\maixcam_color_tracker.py'
$dstDir = Join-Path $PSScriptRoot 'color_tracker'
$dst = Join-Path $dstDir 'main.py'

if (-not (Test-Path $src)) { throw "source not found: $src" }

New-Item -ItemType Directory -Force -Path $dstDir | Out-Null
Copy-Item -LiteralPath $src -Destination $dst -Force
Write-Host ("wrote {0} ({1} bytes)" -f $dst, (Get-Item $dst).Length)

$py = Join-Path $env:USERPROFILE '.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe'
if (Test-Path $py) {
    & $py (Join-Path $PSScriptRoot 'make_icon.py')
} else {
    Write-Host "skipped icon (python not found; run app\make_icon.py manually)"
}

Write-Host "app package:"
Get-ChildItem $dstDir | ForEach-Object { Write-Host ("  {0,-12} {1,6} B" -f $_.Name, $_.Length) }
