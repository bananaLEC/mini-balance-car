# compile.ps1 - 用 armcc 逐文件编译桥接工程（不依赖 UV4，UV4 在当前环境无法运行）
#
# 参数与 template.uvprojx 的 Cads 段保持一致：
#   --c99 --gnu -O1 --cpu Cortex-M3 --no-multibyte-chars
#   STM32F10X_MD + USE_STDPERIPH_DRIVER
#
# 用法： pwsh -File tools\compile.ps1

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$armcc = 'D:\Keil_v5\ARM\ARMCC\Bin\armcc.exe'
$out   = Join-Path $root 'build\obj'

if (Test-Path $out) { Remove-Item -Recurse -Force $out }
New-Item -ItemType Directory -Force -Path $out | Out-Null

# my_lib 必须在搜索路径里：task.h / pid.h / button.h 是作者自己写的模块，靠它才能找到。
# 重构期间 my_lib 下还留着旧的第三方驱动头文件（delay.h / usart.h / i2c.h …）。
# 那些名字**已经没有任何地方引用了**（全部改成了 bsp_ 前缀），所以不会撞车；
# 等旧文件删除后这一点就不需要再惦记了。
$incs = @(
    '-I', 'std_periph_driver\inc',
    '-I', 'user',
    '-I', 'my_lib',
    '-I', 'bsp',
    '-I', 'test'
)

$common = @(
    '--c99', '--gnu', '-O1', '-c',
    '--cpu', 'Cortex-M3',
    '-DSTM32F10X_MD',
    '-DUSE_STDPERIPH_DRIVER',
    '--no-multibyte-chars'
) + $incs

# 本工程自己的代码 + ST 标准外设库
# std_periph_driver / startup 是 ST 的代码，本次重构不修改，但必须编译进去才能链接
$targets = @('my_lib', 'bsp', 'user', 'test', 'std_periph_driver\src') |
           Where-Object { Test-Path (Join-Path $root $_) }

# ⚠️ 临时排除名单
# my_lib 下这些旧驱动已经被 bsp/ 的同名自研实现取代，但**文件本身还留在磁盘上**
# （当前会话的文件沙箱不允许删除 my_lib 里的既有文件，见 docs/驱动重构方案.md）。
# 接口名保持不变，两边一起编译就会出现 "Symbol multiply defined"，
# 所以这里显式排除掉。**等旧文件真正删除后，这一段可以整块去掉。**
$superseded = @(
    'my_lib\delay.c', 'my_lib\usart.c', 'my_lib\i2c.c', 'my_lib\si2c.c',
    'my_lib\oled.c',  'my_lib\qmath.c', 'my_lib\spi.c', 'my_lib\str_cmd.c'
) | ForEach-Object { Join-Path $root $_ }

$files = @()
foreach ($d in $targets) {
    $files += Get-ChildItem -Path $d -Filter *.c -File -Recurse |
              Where-Object { $_.Length -gt 0 }
}
$files = $files | Where-Object { $superseded -notcontains $_.FullName } | Sort-Object FullName

$ok = 0
$failed = @()

foreach ($f in $files) {
    $rel = $f.FullName.Substring($root.Length + 1)
    # 扁平化输出名，避免同名文件互相覆盖
    $objName = ($rel -replace '[\\/]', '_') -replace '\.c$', '.o'
    $obj = Join-Path $out $objName

    $argv = $common + @($rel, '-o', $obj)
    $result = & $armcc @argv 2>&1
    $code = $LASTEXITCODE

    if ($code -eq 0) {
        $ok++
        if ($result) { Write-Host "[warn] $rel`n$($result -join "`n")" -ForegroundColor Yellow }
    } else {
        $failed += [pscustomobject]@{ File = $rel; Output = ($result -join "`n") }
    }
}

Write-Host ''
Write-Host "==== 编译结果： $ok / $($files.Count) 通过 ====" -ForegroundColor Cyan

if ($failed.Count -gt 0) {
    Write-Host ''
    foreach ($f in $failed) {
        Write-Host "---- FAILED: $($f.File)" -ForegroundColor Red
        Write-Host $f.Output
    }
    exit 1
}

exit 0
