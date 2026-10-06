# 清理旧驱动.ps1 —— 删除已被 bsp/ 取代的第三方驱动文件
#
# 为什么要单独给一个脚本：
#   这些文件已经不在 git 里、也不参与编译（已从 template.uvprojx 移出），
#   但会话文件沙箱不允许删除 my_lib 下已存在的文件，所以需要你手动跑一次。
#
# 安全性：
#   · 只删下面显式列出的文件，不做通配匹配
#   · my_lib/font/ 是唯一递归删除的目标（第三方工具 + 商业字体）
#   · 删之前先打清单并要你确认
#   · my_lib/task.* / pid.* / button.* 是自己写的，**不会**被删
#   · 删完代码不需要任何改动：user/ 与 test/ 引用的都是 bsp/ 下的新接口
#
# 用法： pwsh -File tools\清理旧驱动.ps1
#    或 双击根目录的 清理旧驱动.cmd

$ErrorActionPreference = 'Stop'

# 脚本放在 tools/ 下，往上找一层才是仓库根
$root = Split-Path -Parent $PSScriptRoot
Set-Location -Path $root

$oldFiles = @(
    'my_lib\delay.c', 'my_lib\delay.h',
    'my_lib\usart.c', 'my_lib\usart.h',
    'my_lib\i2c.c',   'my_lib\i2c.h',
    'my_lib\si2c.c',  'my_lib\si2c.h',
    'my_lib\oled.c',  'my_lib\oled.h',
    'my_lib\oled_font.h', 'my_lib\oled_default_font.h',
    'my_lib\qmath.c', 'my_lib\qmath.h',
    'my_lib\spi.c',   'my_lib\spi.h',
    'my_lib\str_cmd.c', 'my_lib\str_cmd.h'
)

Write-Host '即将删除以下文件（bsp/ 已提供同名接口的自研实现）：' -ForegroundColor Cyan

$present = @()
foreach ($f in $oldFiles) {
    if (Test-Path -LiteralPath $f) {
        Write-Host "  $f"
        $present += $f
    }
}

$hasFontDir = Test-Path -LiteralPath 'my_lib\font'
if ($hasFontDir) {
    Write-Host '  my_lib\font\            (第三方字体转换工具与商业 TTF 字体)'
}

if (($present.Count -eq 0) -and (-not $hasFontDir)) {
    Write-Host '没有需要清理的文件，可能已经删过了。' -ForegroundColor Green
    exit 0
}

Write-Host ''
Write-Host '保留不动：my_lib\task.*  my_lib\pid.*  my_lib\button.*（本项目自己写的）' -ForegroundColor Green

$ans = Read-Host '确认删除？输入 y 继续'
if ($ans -ne 'y') {
    Write-Host '已取消，未做任何改动。'
    exit 0
}

$failed = @()

foreach ($f in $present) {
    try {
        Remove-Item -LiteralPath $f -Force -ErrorAction Stop
        Write-Host "  已删除 $f"
    } catch {
        $failed += $f
    }
}

if ($hasFontDir) {
    try {
        Remove-Item -LiteralPath 'my_lib\font' -Recurse -Force -ErrorAction Stop
        Write-Host '  已删除 my_lib\font\'
    } catch {
        $failed += 'my_lib\font\'
    }
}

Write-Host ''
if ($failed.Count -gt 0) {
    Write-Host '以下目标删除失败，请关闭 Keil / 编辑器后重试：' -ForegroundColor Yellow
    $failed | ForEach-Object { Write-Host "  $_" }
    exit 1
}

Write-Host '清理完成。' -ForegroundColor Green
Write-Host ''
Write-Host '建议接着验证一次编译链接（不需要 Keil）：'
Write-Host '  pwsh -File tools\compile.ps1'
Write-Host '  pwsh -File tools\link.ps1'
