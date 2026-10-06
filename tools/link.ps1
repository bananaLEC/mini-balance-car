# link.ps1 - 汇编启动文件并用 armlink 链接出完整固件，输出体积统计
#
# 复现 Keil 的链接方式：
#   armasm  startup_stm32f10x_md.s
#   armlink Objects/*.o --library_type=microlib --strict --scatter template.sct
#
# 基线（重构前，与 Keil 构建日志一致）：
#   Code=23516  RO-data=14444  RW-data=196  ZI-data=9188
#
# 用法： pwsh -File tools\link.ps1   （需先运行 compile.ps1）

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$armasm  = 'D:\Keil_v5\ARM\ARMCC\Bin\armasm.exe'
$armlink = 'D:\Keil_v5\ARM\ARMCC\Bin\armlink.exe'

$objDir = Join-Path $root 'build\obj'
$build  = Join-Path $root 'build'

if (-not (Test-Path $objDir)) {
    Write-Host 'build\obj 不存在，请先运行 tools\compile.ps1' -ForegroundColor Red
    exit 1
}

# --- 1. 汇编启动文件 ---
# 必须定义 __MICROLIB，否则启动文件会走「非 microlib」分支：
#   导出 __user_initial_stackheap、导入 __use_two_region_memory，
#   而链接选项是 --library_type=microlib，就会报 __initial_sp / __heap_base 未定义。
#   Keil 在库类型选 microlib 时会自动加上这个符号，命令行下要手动传。
$startupSrc = Join-Path $root 'startup\startup_stm32f10x_md.s'
$startupObj = Join-Path $objDir 'startup_stm32f10x_md.o'

$asmOut = & $armasm --cpu Cortex-M3 --pd "__MICROLIB SETA 1" $startupSrc -o $startupObj 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host '启动文件汇编失败：' -ForegroundColor Red
    Write-Host ($asmOut -join "`n")
    exit 1
}

# --- 2. 链接 ---
$objs = Get-ChildItem -Path $objDir -Filter *.o -File | ForEach-Object { $_.FullName }

$scatter = Join-Path $root 'Objects\template.sct'
if (-not (Test-Path $scatter)) {
    Write-Host "找不到分散加载文件：$scatter" -ForegroundColor Red
    exit 1
}

$mapFile = Join-Path $build 'template.map'
$axf     = Join-Path $build 'template.axf'

$linkArgs = @($objs) + @(
    '--cpu', 'Cortex-M3',
    '--library_type=microlib',
    '--strict',
    '--scatter', $scatter,
    '--info', 'totals',
    '--map',
    '--list', $mapFile,
    '-o', $axf
)

$linkOut = & $armlink @linkArgs 2>&1
$code = $LASTEXITCODE

Write-Host ($linkOut -join "`n")

if ($code -ne 0) {
    Write-Host '链接失败' -ForegroundColor Red
    exit 1
}

# --- 3. 生成 hex（可选，方便直接下载）---
$fromelf = 'D:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe'
if (Test-Path $fromelf) {
    & $fromelf --i32combined -o (Join-Path $build 'template.hex') $axf 2>&1 | Out-Null
}

Write-Host ''
Write-Host '==== 链接完成 ====' -ForegroundColor Cyan
Write-Host "产物：build\template.axf  build\template.hex"
Write-Host "基线：Code=23516 RO-data=14444 RW-data=196 ZI-data=9188"

exit 0
