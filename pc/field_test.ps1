# Field-test helper for the MaixCAM color tracker.
# Keep this file ASCII-only: Windows PowerShell 5.1 reads .ps1 as ANSI.
#
# Usage (from E:\maxican):
#   powershell -ExecutionPolicy Bypass -File pc\field_test.ps1 -Deploy
#   powershell -ExecutionPolicy Bypass -File pc\field_test.ps1 -Run "-mission --task-code 132 --duration 10 --no-display"
#   powershell -ExecutionPolicy Bypass -File pc\field_test.ps1 -Run "-mission --task-code 132" -Save
#   powershell -ExecutionPolicy Bypass -File pc\field_test.ps1 -Log
#   powershell -ExecutionPolicy Bypass -File pc\field_test.ps1 -Pull
#   powershell -ExecutionPolicy Bypass -File pc\field_test.ps1 -Stop
#   powershell -ExecutionPolicy Bypass -File pc\field_test.ps1 -Reboot
#   powershell -ExecutionPolicy Bypass -File pc\field_test.ps1 -Cfg
#   powershell -ExecutionPolicy Bypass -File pc\field_test.ps1 -SetCfg '{"mission":true,"task_code":"132"}'
#
# Requires a working "ssh maxican" (key already installed).

param(
    [switch]$Deploy,
    [string]$Run,
    [switch]$Save,
    [switch]$Log,
    [switch]$Pull,
    [switch]$Stop,
    [switch]$Reboot,
    [switch]$Cfg,
    [switch]$CfgPull,
    [string]$CfgFile,
    [string]$H = "maxican"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$src = Join-Path $root "camera\maixcam_color_tracker.py"

function Invoke-Ssh([string]$cmd) {
    & ssh -o BatchMode=yes $H $cmd
    if ($LASTEXITCODE -ne 0) { Write-Host ("[ssh] exit " + $LASTEXITCODE) -ForegroundColor Yellow }
}

function Stop-App {
    Write-Host "[stop] killing launcher + app (frees camera and ttyS1)"
    # NOTE: this image has no `pkill` (only /usr/bin/killall and /usr/bin/kill).
    # The old command used `pkill -f color_tracker`, which silently did nothing,
    # so the app kept the camera open and the next -Run died with SIGSEGV.
    Invoke-Ssh "killall -q launcher_daemon launcher maixvision_server 2>/dev/null; killall -q python3 2>/dev/null; sleep 1; killall -q -9 python3 2>/dev/null; sleep 1; ps -ef | grep -c '[c]olor_tracker'"
}

if ($Deploy) {
    Write-Host "[deploy] rebuild app package"
    & powershell -ExecutionPolicy Bypass -File (Join-Path $root "app\build_app.ps1") | Out-Null
    Write-Host "[deploy] upload source + app"
    & scp -o BatchMode=yes $src "${H}:/root/maixcam_color_tracker.py"
    & scp -o BatchMode=yes (Join-Path $root "app\color_tracker\main.py") "${H}:/maixapp/apps/color_tracker/main.py"
    & scp -o BatchMode=yes (Join-Path $root "camera\vision_selftest.py") "${H}:/root/vision_selftest.py"
    Invoke-Ssh "md5sum /root/maixcam_color_tracker.py /maixapp/apps/color_tracker/main.py"
}

if ($Stop) { Stop-App }

if ($Run -ne "") {
    Stop-App
    $extra = ""
    if ($Save) { $extra = " --save-image /root/snap.jpg --save-every 2" }
    $cmd = "cd /root && python3 maixcam_color_tracker.py " + $Run + $extra + " 2>&1"
    Write-Host ("[run] " + $cmd)
    Invoke-Ssh $cmd
    if ($Save) {
        New-Item -ItemType Directory -Force -Path (Join-Path $root "captures") | Out-Null
        $stamp = Get-Date -Format "MMdd-HHmmss"
        & scp -o BatchMode=yes "${H}:/root/snap.jpg" (Join-Path $root ("captures\snap-" + $stamp + ".jpg"))
        Write-Host "[run] snapshot saved to captures\snap-$stamp.jpg"
    }
}

if ($Log) {
    Invoke-Ssh "tail -n 40 /maixapp/tmp/last_run.log"
}

if ($Pull) {
    New-Item -ItemType Directory -Force -Path (Join-Path $root "captures") | Out-Null
    & scp -o BatchMode=yes "${H}:/root/*.jpg" (Join-Path $root "captures\")
    Get-ChildItem (Join-Path $root "captures") | Select-Object Name, Length, LastWriteTime
}

if ($Cfg) {
    Invoke-Ssh "cat /root/color_tracker_cfg.json; echo; cat /maixapp/auto_start.txt 2>/dev/null; echo"
}

if ($CfgPull) {
    # Download the config so it can be edited in a real editor (no quoting traps).
    $local = Join-Path $root "color_tracker_cfg.json"
    & scp -o BatchMode=yes "${H}:/root/color_tracker_cfg.json" $local
    Write-Host "[cfg] saved to $local"
}

if ($CfgFile -ne "") {
    # Upload a config file. This avoids PowerShell eating the quotes inside JSON:
    # do NOT try to pass JSON on the command line.
    if (-not (Test-Path $CfgFile)) { throw "config file not found: $CfgFile" }
    & scp -o BatchMode=yes $CfgFile "${H}:/root/color_tracker_cfg.json"
    Invoke-Ssh "sync; cat /root/color_tracker_cfg.json; echo"
    Write-Host "[cfg] uploaded (reboot or restart the app for it to take effect)"
}

if ($Reboot) {
    Write-Host "[reboot] device will be back in ~45 s"
    # ssh gets its connection reset by the reboot; that is expected, not an error.
    $oldEap = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    & ssh -o BatchMode=yes $H "sync; sleep 1; /sbin/reboot -f" 2>$null | Out-Null
    Start-Sleep -Seconds 45
    for ($i = 0; $i -lt 20; $i++) {
        & ssh -o BatchMode=yes -o ConnectTimeout=3 $H "echo up" 2>$null | Out-Null
        if ($LASTEXITCODE -eq 0) { Write-Host "[reboot] back online"; break }
        Start-Sleep -Seconds 3
    }
    $ErrorActionPreference = $oldEap
}

if (-not ($Deploy -or $Run -ne "" -or $Log -or $Pull -or $Stop -or $Reboot -or $Cfg -or $CfgPull -or $CfgFile -ne "")) {
    Write-Host "field_test.ps1 - MaixCAM color tracker helper"
    Write-Host ""
    Write-Host "  -Deploy                 rebuild app package and upload to device"
    Write-Host "  -Run ""<args>""           stop the app and run one headless test"
    Write-Host "  -Save                   with -Run: save a snapshot and pull it back"
    Write-Host "  -Log                    tail the app log"
    Write-Host "  -Pull                   pull all /root/*.jpg into .\captures"
    Write-Host "  -Stop                   stop launcher + app (frees the camera)"
    Write-Host "  -Reboot                 reboot and wait until back online"
    Write-Host "  -Cfg                    show the current config on the device"
    Write-Host "  -CfgPull                download the config to .\color_tracker_cfg.json"
    Write-Host "  -CfgFile <path>         upload a config file (edit it locally first)"
    Write-Host ""
    Write-Host "  examples:"
    Write-Host "    -Run ""--mission --task-code 132 --duration 10 --no-display"""
    Write-Host "    -CfgPull      (edit color_tracker_cfg.json)      -CfgFile color_tracker_cfg.json -Reboot"
}
