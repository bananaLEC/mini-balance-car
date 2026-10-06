"""验证 OLED_Pump 每段设置的列/页窗口是否正好覆盖这一段数据。"""

W, PAGES, FRAME, CHUNK = 128, 8, 1024, 8

print('start  chunk  列窗口        页窗口   窗口起点  覆盖数  判定')
ok = True
total = 0
for start in range(0, FRAME, CHUNK):
    chunk = min(CHUNK, FRAME - start)
    col_s = start % W
    col_e = W - 1
    page_s = start // W
    page_e = (start + chunk - 1) // W
    n_cells = (col_e - col_s + 1) + (page_e - page_s) * W
    base = page_s * W + col_s
    good = (base == start) and (n_cells >= chunk)
    ok = ok and good
    total += chunk
    if start < 24 or start > FRAME - 24:
        verdict = 'OK' if good else 'BAD'
        print(f'{start:5d}  {chunk:5d}  0x{col_s:02X}-0x{col_e:02X}    {page_s}-{page_e}     {base:6d}  {n_cells:6d}  {verdict}')

print()
print('每段窗口起点都精确对齐到 start :', ok)
print('段数 =', FRAME // CHUNK, ' 覆盖总字节 =', total, '(应为 1024)')
print('帧缓冲 = 8 页 x 128 列 =', PAGES * W)
