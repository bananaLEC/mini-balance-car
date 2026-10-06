# update_uvprojx.ps1 - 把重构后的 bsp/ 模块接入 Keil 工程
#
# 做三件事：
#   1. Cads/IncludePath 加入 .\bsp
#   2. 新增 <Group>bsp</Group>，列出 bsp/ 下的源文件
#   3. my_lib 组里移除已被取代的旧驱动文件，只留下 task/pid/button
#
# 说明：旧文件在磁盘上还留着（本会话沙箱不允许删除 my_lib 里的既有文件），
#      但已从工程里移出，因此不会再被编译，也不会和 bsp/ 的同名符号冲突。
#      等旧文件真正删除后，这个脚本不需要再跑。
#
# 用法： pwsh -File tools\update_uvprojx.ps1

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$proj = Join-Path $root 'template.uvprojx'

if (-not (Test-Path $proj)) {
    Write-Host "找不到 $proj" -ForegroundColor Red
    exit 1
}

$text = [System.IO.File]::ReadAllText($proj, [System.Text.Encoding]::UTF8)
$orig = $text

# --- 1. IncludePath 加 .\bsp ---
$oldInc = '.\std_periph_driver\inc;.\user;.\my_lib;.\my_lib\font;.\my_lib\font\.h;.\test'
$newInc = '.\std_periph_driver\inc;.\user;.\my_lib;.\bsp;.\test'
if ($text.Contains($oldInc)) {
    $text = $text.Replace($oldInc, $newInc)
    Write-Host "IncludePath 已更新：$newInc"
} elseif ($text.Contains($newInc)) {
    Write-Host 'IncludePath 已经是新的，跳过'
} else {
    Write-Host 'IncludePath 与预期不符，请手动检查' -ForegroundColor Yellow
}

# --- 2. my_lib 组里移除旧驱动 ---
# 只保留自己写的 task / pid / button
$keepInMyLib = @('task.c', 'task.h', 'pid.c', 'pid.h', 'button.c', 'button.h')

$fileBlock = '(?s)\s*<File>\s*<FileName>([^<]+)</FileName>.*?</File>'
$myLibGroup = [regex]::Match($text, '(?s)<Group>\s*<GroupName>my_lib</GroupName>.*?</Group>')

if (-not $myLibGroup.Success) {
    Write-Host '找不到 my_lib 组' -ForegroundColor Red
    exit 1
}

$groupText = $myLibGroup.Value
$removed = @()
$rebuilt = [regex]::Replace($groupText, $fileBlock, {
    param($m)
    $name = $m.Groups[1].Value
    if ($keepInMyLib -contains $name) {
        $m.Value
    } else {
        $script:removed += $name
        ''
    }
})

$text = $text.Remove($myLibGroup.Index, $myLibGroup.Length).Insert($myLibGroup.Index, $rebuilt)
Write-Host ("my_lib 组移除 {0} 个文件：{1}" -f $removed.Count, ($removed -join ', '))

# --- 3. 新增 bsp 组（放在 test 组之前）---
$bspFiles = @(
    @{ n = 'bsp_delay.h';   t = 5 },
    @{ n = 'bsp_delay.c';   t = 1 },
    @{ n = 'bsp_usart.h';   t = 5 },
    @{ n = 'bsp_usart.c';   t = 1 },
    @{ n = 'bsp_i2c.h';     t = 5 },
    @{ n = 'bsp_i2c.c';     t = 1 },
    @{ n = 'bsp_si2c.h';    t = 5 },
    @{ n = 'bsp_si2c.c';    t = 1 },
    @{ n = 'bsp_font6x8.h'; t = 5 },
    @{ n = 'bsp_font6x8.c'; t = 1 },
    @{ n = 'bsp_oled.h';    t = 5 },
    @{ n = 'bsp_oled.c';    t = 1 }
)

$sb = New-Object System.Text.StringBuilder
[void]$sb.AppendLine('        <Group>')
[void]$sb.AppendLine('          <GroupName>bsp</GroupName>')
[void]$sb.AppendLine('          <Files>')
foreach ($f in $bspFiles) {
    [void]$sb.AppendLine('            <File>')
    [void]$sb.AppendLine("              <FileName>$($f.n)</FileName>")
    [void]$sb.AppendLine("              <FileType>$($f.t)</FileType>")
    [void]$sb.AppendLine("              <FilePath>.\bsp\$($f.n)</FilePath>")
    [void]$sb.AppendLine('            </File>')
}
[void]$sb.AppendLine('          </Files>')
[void]$sb.Append('        </Group>')

$bspGroup = $sb.ToString()

if ($text -match '<GroupName>bsp</GroupName>') {
    Write-Host 'bsp 组已存在，跳过新增'
} else {
    # 插到 test 组之前
    $testGroupMatch = [regex]::Match($text, '(?s)\s*<Group>\s*<GroupName>test</GroupName>')
    if (-not $testGroupMatch.Success) {
        Write-Host '找不到 test 组，无法定位插入点' -ForegroundColor Red
        exit 1
    }
    $text = $text.Insert($testGroupMatch.Index, "`r`n" + $bspGroup)
    Write-Host ("新增 bsp 组，{0} 个文件" -f $bspFiles.Count)
}

if ($text -eq $orig) {
    Write-Host '没有任何改动'
    exit 0
}

# 写回（保持 UTF-8 无 BOM，与原文件一致）
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($proj, $text, $utf8NoBom)

Write-Host 'template.uvprojx 已更新' -ForegroundColor Green
