"""验证新的 DelayUs() 对任意 us 都能正常退出（旧写法在 us>1000 时死循环）。"""

M = 1 << 32


def delay_us_iterations(us, start, get_us_at):
    """复现： start=(uint32)GetUs(); while((uint32)((uint32)GetUs()-start) < us);

    返回退出前经历的时间（us）；超过上限返回 None 表示死循环。
    """
    t = 0
    while True:
        t += 1
        if t > 3_000_000:
            return None
        cur = get_us_at(t) % M
        if ((cur - start) % M) >= us:
            return t


print('=== 新 DelayUs：无符号差值比较 ===')
for us in (1, 2, 5, 100, 999, 1000, 1001, 5000, 60000, 1_000_000):
    start = 0xFFFFF000 % M                      # 起点故意贴近 32 位回绕点
    n = delay_us_iterations(us, start, lambda t, s=start: s + t)
    if n is None:
        verdict = '死循环！'
    elif n >= us:
        verdict = f'正常退出（等了 {n} us）'
    else:
        verdict = f'异常：只等了 {n} us'
    print(f'  us={us:<9} -> {verdict}')

print()
print('=== 对照：旧写法按 SysTick 周期数比较 ===')
print('    want_cycles = us * 72')
print('    但 Delay_CyclesBetween 的返回值上限是 1000 * 72 = 72000')
for us in (2, 100, 999, 1000, 1001, 5000, 60000):
    want = us * 72
    if want > 72000:
        verdict = '永远不成立 -> 死循环'
    else:
        verdict = '可以退出'
    print(f'  us={us:<7} want_cycles={want:<9} -> {verdict}')
