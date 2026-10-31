#requires -Version 5.1
<#
    打包「智能搬运 · 上位机」便携版
    ---------------------------------------------------------------
    产物：
        release\智能搬运上位机\        双击 启动上位机.bat 即可运行
        release\智能搬运上位机.zip      整个文件夹的压缩包，直接发给别人

    特点：pyserial 源码随包携带，目标电脑只要有 Python 3.8+ 就能跑，
          不需要 pip install，也不需要联网。

    用法：
        双击 pc\打包上位机.bat
        或者  powershell -ExecutionPolicy Bypass -File pc\build_release.ps1
    注意：本文件含中文，必须保存为 UTF-8 with BOM，否则 Windows
          PowerShell 5.1 会按 ANSI 读，中文全乱。
#>
[CmdletBinding()]
param(
    [string]$Python = "",      # 指定解释器，例如 "py -3" 或 "C:\Python313\python.exe"
    [switch]$NoZip
)

$ErrorActionPreference = "Stop"

$pcDir = $PSScriptRoot
$root = Split-Path -Parent $pcDir
$out = Join-Path $root "release\智能搬运上位机"
$app = Join-Path $out "app"
$tpl = Join-Path $pcDir "release_tpl"
$zip = Join-Path $root "release\智能搬运上位机.zip"

function Find-Python {
    if ($Python) { return @($Python -split ' +') }
    if (Get-Command py.exe -ErrorAction SilentlyContinue) {
        & py -3 -c "import sys" 2>$null
        if ($LASTEXITCODE -eq 0) { return @("py", "-3") }
    }
    foreach ($n in @("python.exe", "python3.exe")) {
        $c = Get-Command $n -ErrorAction SilentlyContinue
        if ($c -and $c.Source -notmatch 'WindowsApps') { return @($c.Source) }
    }
    return @()
}

function Save-TextFile {
    param(
        [Parameter(Mandatory)][string]$Path,
        [Parameter(Mandatory)][AllowEmptyString()][string]$Text,
        [ValidateSet("gbk", "utf8bom")][string]$Enc = "utf8bom"
    )
    # 统一 CRLF（bat 必须是 CRLF，否则 cmd 会解析错行）
    $Text = ($Text -replace "`r`n", "`n") -replace "`n", "`r`n"
    if ($Enc -eq "gbk") {
        $bytes = [System.Text.Encoding]::GetEncoding(936).GetBytes($Text)
    } else {
        $u8 = New-Object System.Text.UTF8Encoding($true)
        $bytes = $u8.GetPreamble() + $u8.GetBytes($Text)
    }
    [System.IO.File]::WriteAllBytes($Path, $bytes)
}

Write-Host ""
Write-Host "== 打包上位机 ==" -ForegroundColor Cyan
Write-Host "   输出: $out"

# ---------------------------------------------------------------- 1. 找解释器
$pyCmd = Find-Python
$pyExe = ""
$pre = @()
if ($pyCmd.Count -gt 0) {
    $pyExe = $pyCmd[0]
    if ($pyCmd.Count -gt 1) { $pre = @($pyCmd[1..($pyCmd.Count - 1)]) }
}

# ---------------------------------------------------------------- 2. 找 pyserial 源码
$serialSrc = ""
if ($pyExe) {
    $a = $pre + @("-c", "import serial,os;print(os.path.dirname(serial.__file__))")
    try {
        $probe = & $pyExe @a 2>$null
        if ($LASTEXITCODE -eq 0 -and $probe) { $serialSrc = ($probe | Select-Object -First 1).Trim() }
    } catch { }
}
if ($serialSrc) {
    Write-Host "   pyserial: $serialSrc"
} else {
    Write-Host "   [警告] 本机没装 pyserial，将沿用 release 里已有的 app\serial" -ForegroundColor Yellow
}

# ---------------------------------------------------------------- 3. 清理 app（旧的 serial 先暂存）
$oldSerial = Join-Path $app "serial"
$stash = $null
if ((Test-Path $oldSerial) -and -not $serialSrc) {
    $stash = Join-Path $env:TEMP ("serial_stash_" + [guid]::NewGuid().ToString("N"))
    Copy-Item $oldSerial $stash -Recurse -Force
}
if (Test-Path $app) { Remove-Item $app -Recurse -Force }
New-Item -ItemType Directory -Path $app -Force | Out-Null

# ---------------------------------------------------------------- 4. 主程序
Copy-Item (Join-Path $pcDir "ground_station.py") $app -Force
Copy-Item (Join-Path $pcDir "protocol.py")       $app -Force
$gsKB = [math]::Round((Get-Item (Join-Path $app "ground_station.py")).Length / 1KB, 1)
Write-Host "   + app\ground_station.py  ($gsKB KB)"
Write-Host "   + app\protocol.py"

# ---------------------------------------------------------------- 5. 携带 pyserial（纯 Python，拷 .py 即可）
$serialDst = Join-Path $app "serial"
if ($serialSrc) {
    Get-ChildItem $serialSrc -Recurse -File -Filter *.py | ForEach-Object {
        $rel = $_.FullName.Substring($serialSrc.Length).TrimStart('\')
        $dstFile = Join-Path $serialDst $rel
        $dir = Split-Path -Parent $dstFile
        if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
        Copy-Item $_.FullName $dstFile -Force
    }
} elseif ($stash -and (Test-Path $stash)) {
    Copy-Item $stash $serialDst -Recurse -Force
    Remove-Item $stash -Recurse -Force
}
if (Test-Path $serialDst) {
    $n = (Get-ChildItem $serialDst -Recurse -File).Count
    Write-Host "   + app\serial\  (pyserial，$n 个文件)"
} else {
    Write-Host "   [错误] 没有 pyserial，真实串口功能会不可用" -ForegroundColor Red
}

# ---------------------------------------------------------------- 6. 启动器和说明（转编码）
$batSrc = Join-Path $tpl "启动上位机.bat"
$txtSrc = Join-Path $tpl "使用说明.txt"
if (-not (Test-Path $batSrc)) { throw "缺少模板：$batSrc" }
if (-not (Test-Path $txtSrc)) { throw "缺少模板：$txtSrc" }

Save-TextFile -Path (Join-Path $out "启动上位机.bat") `
              -Text (Get-Content $batSrc -Raw -Encoding UTF8) -Enc gbk
Save-TextFile -Path (Join-Path $out "使用说明.txt") `
              -Text (Get-Content $txtSrc -Raw -Encoding UTF8) -Enc utf8bom
Write-Host "   + 启动上位机.bat  (GBK / CRLF，cmd 里中文不乱码)"
Write-Host "   + 使用说明.txt    (UTF-8 BOM / CRLF)"

# ---------------------------------------------------------------- 7. 自检
if ($pyExe) {
    $a = $pre + @("-m", "py_compile", (Join-Path $app "ground_station.py"))
    & $pyExe @a
    if ($LASTEXITCODE -ne 0) { throw "ground_station.py 语法检查失败" }

    $code = "import sys,os;sys.path.insert(0,sys.argv[1]);import serial;print(1 if os.path.abspath(serial.__file__).startswith(os.path.abspath(sys.argv[1])) else 0, serial.VERSION)"
    $a = $pre + @("-c", $code, $app)
    $chk = ((& $pyExe @a) -join " ").Trim()
    if ($LASTEXITCODE -ne 0) { throw "便携包内 import serial 失败" }
    if ($chk -like "1*") {
        Write-Host ("   + 自检通过：包内 pyserial {0} 可导入" -f $chk.Substring(1).Trim())
    } else {
        Write-Host "   [警告] import serial 没走在包内：$chk" -ForegroundColor Yellow
    }
    Remove-Item (Join-Path $app "__pycache__") -Recurse -Force -ErrorAction SilentlyContinue
    Remove-Item (Join-Path $serialDst "__pycache__") -Recurse -Force -ErrorAction SilentlyContinue
}

# ---------------------------------------------------------------- 8. 打 zip
if (-not $NoZip) {
    if (Test-Path $zip) { Remove-Item $zip -Force }
    Compress-Archive -Path $out -DestinationPath $zip -CompressionLevel Optimal
    Write-Host "   + 智能搬运上位机.zip  ($([math]::Round((Get-Item $zip).Length / 1KB, 1)) KB)"
}

# ---------------------------------------------------------------- 9. 汇总
$files = @(Get-ChildItem $out -Recurse -File)
$total = ($files | Measure-Object Length -Sum).Sum
Write-Host ""
Write-Host "完成。双击 release\智能搬运上位机\启动上位机.bat 即可运行。" -ForegroundColor Green
Write-Host ("整包 {0} KB，{1} 个文件。" -f [math]::Round($total / 1KB, 1), $files.Count)
Write-Host ""
