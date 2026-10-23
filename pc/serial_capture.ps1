# Capture raw bytes from a serial port (for sniffing the camera's UART output).
#
# Why PowerShell: this machine's Python has no pyserial, and .NET ships a
# serial port class. This script only *captures raw bytes*; decoding is done by
# pc\serial_monitor.py --file, which reuses the tested protocol parser.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File pc\serial_capture.ps1 -List
#   powershell -ExecutionPolicy Bypass -File pc\serial_capture.ps1 -Port COM5 -Seconds 5 -Out capture.bin
#   powershell -ExecutionPolicy Bypass -File pc\serial_capture.ps1 -Port COM5 -Send "CC CC DD DD 04 00 04 FF FF EE EE"
#
# Wiring (camera -> CH340):
#   MaixCAM A19 (UART1_TX) -> CH340 RX
#   MaixCAM A18 (UART1_RX) <- CH340 TX      (only needed to send commands back)
#   MaixCAM GND            -> CH340 GND     (REQUIRED - no common ground, no link)
#
# NOTE: keep this file ASCII-only. Windows PowerShell 5.1 reads .ps1 as ANSI,
# so non-ASCII text here can break parsing.

param(
    [switch]$List,
    [string]$Port,
    [int]$Seconds = 5,
    [string]$Out = "capture.bin",
    [string]$Send = ""
)

$ErrorActionPreference = 'Stop'

if ($List) {
    $names = [System.IO.Ports.SerialPort]::GetPortNames()
    if (-not $names -or $names.Count -eq 0) {
        Write-Host "no serial ports found (is the CH340 plugged in and its driver installed?)"
    } else {
        Write-Host "serial ports:"
        foreach ($n in $names) { Write-Host ("  " + $n) }
    }
    exit 0
}

if (-not $Port) {
    Write-Host "need -Port COMx   (run with -List to see available ports)"
    exit 2
}

# NOTE: the enum arguments must be wrapped in parentheses, otherwise
# New-Object passes them as literal strings and the ctor call fails.
$sp = New-Object System.IO.Ports.SerialPort($Port, 115200, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One))
$sp.ReadTimeout = 500
$sp.WriteTimeout = 500
# Default ReadBufferSize is only 4KB. At 510 B/s (30fps x 17B) a stall in the
# PowerShell loop would overflow it and silently drop a burst of frames, which
# shows up as a fake "gap" in the sequence numbers. Give it plenty of room.
$sp.ReadBufferSize = 1048576
$sp.Open()
# Flush whatever the driver buffered BEFORE we opened the port. Otherwise the
# first few frames come from an earlier session and the sequence numbers show a
# bogus "gap" that looks like frame loss. (Seen twice: gap right after frame 4.)
Start-Sleep -Milliseconds 200
$sp.DiscardInBuffer()
Write-Host ("opened {0} @ 115200 8N1" -f $Port)

if ($Send) {
    $parts = $Send -split '[\s,]+' | Where-Object { $_ -ne '' }
    $bytes = New-Object byte[] $parts.Count
    for ($i = 0; $i -lt $parts.Count; $i++) {
        $bytes[$i] = [Convert]::ToByte($parts[$i], 16)
    }
    $sp.Write($bytes, 0, $bytes.Length)
    Write-Host ("sent {0} bytes" -f $bytes.Length)
    Start-Sleep -Milliseconds 300
    # Sending only: nothing to capture, done.
    if ($Seconds -le 0) { $sp.Close(); exit 0 }
}

$fs = [System.IO.File]::Create($Out)
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$gapWatch = [System.Diagnostics.Stopwatch]::StartNew()
$buf = New-Object byte[] 4096
$total = 0
$reads = 0
$maxGapMs = 0.0
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try {
        $n = $sp.Read($buf, 0, $buf.Length)
        if ($n -gt 0) {
            $fs.Write($buf, 0, $n)
            $total += $n
            $reads++
            # how long since the previous successful read? a big gap means the
            # capture loop (or the USB path) stalled and data was lost.
            $gapMs = $gapWatch.Elapsed.TotalMilliseconds
            if ($gapMs -gt $maxGapMs) { $maxGapMs = $gapMs }
            $gapWatch.Restart()
        }
    } catch [System.TimeoutException] {
        # no data within ReadTimeout - normal, keep waiting
    }
}
$fs.Close()
$sp.Close()
Write-Host ("captured {0} bytes in {1} reads -> {2}" -f $total, $reads, $Out)
Write-Host ("max gap between reads: {0:N0} ms" -f $maxGapMs)
if ($total -eq 0) {
    Write-Host "nothing received. check: A19 -> CH340 RX, and GND -> GND (common ground is required)"
}
