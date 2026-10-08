"""分析：修复 GetUs() 之后，各 PID 积分项的实际强度变化了多少倍。

背景：
  旧 GetUs() 有两个缺陷 ——
    (1) 只有 1ms 分辨率；
    (2) 会倒退：SysTick_Handler 与 GetUs() 都会给 ulTicks 加一，同一拍可能 +2。
  而 app_balance.c:221-233 是：
        now = GetUs();
        dt  = (now - balance_last_us) * 1.0e-6f;   // now 倒退 -> dt 为负
        balance_last_us = now;
        if (dt < 1.0e-6f) dt = 1.0e-6f;             // 负 dt 被夹成 1us
  所以只要发生一次倒退，这一拍的 dt 就是 1us 而不是真实的 5000us。

积分项在 pid.c 里是 err_int += (err + err_k_1)*0.5*dt，因此
「单位时间内的积分增量」正比于 dt。dt 被夹成 1us 时，积分强度只有
真实值的 1/5000 —— 也就是说，旧代码下所有 Ki 的实际作用几乎为零。
"""

print('=' * 68)
print('PID 积分强度对比（以 5ms 控制周期为例）')
print('=' * 68)
print()
print('  真实周期            : 5000 us')
print('  旧代码被夹后的 dt    :    1 us  （负 dt 被 if(dt<1e-6f) 夹成 1us）')
print(f'  积分强度比          : 1/5000 = {1/5000:.6f}')
print()

# 各环的 Ki 与积分限幅，取自 app_balance.c / app_motor.c
loops = [
    ('速度环   pid_speed', 'BAL_KI_SPEED', 0.015),
    ('角度环   pid_angle', 'BAL_KP_OUTER 无 Ki', 0.0),
    ('角速度环 pid_rate ', 'BAL_KI_INNER', 10.0),
    ('调速环   pid_motor', 'MOTOR_KI', 6.0),
]

print('=' * 68)
print('各环 Ki 的实际作用（积分项对输出的贡献 / 秒）')
print('=' * 68)
print(f'{"环":<22}{"Ki":>8}{"旧代码":>14}{"新代码":>14}{"倍数":>10}')
for name, sym, ki in loops:
    if ki == 0.0:
        print(f'{name:<22}{"—":>8}{"—":>14}{"—":>14}{"—":>10}')
        continue
    old = ki * 1e-6      # dt = 1us
    new = ki * 5000e-6   # dt = 5000us
    ratio = new / old if old else float('inf')
    print(f'{name:<22}{ki:>8.3f}{old:>14.3e}{new:>14.3e}{ratio:>9.0f}x')

print()
print('=' * 68)
print('结论')
print('=' * 68)
print("""
  修好 GetUs() 之后，所有 Ki 的积分作用一次性放大了约 5000 倍。

  这不是「变小」而是「突然变大」，后果是：
    · 积分项在几十毫秒内就冲到限幅（BAL_INNER_IMAX / MOTOR_INT_LIMIT）
    · 表现为过冲 + 低频振荡：车能立住一小会儿，然后越摆越大倒下去
    · 与「显示屏正常、能立 2~3 秒后倒下」的现象一致

  所以：原来那套 PID 参数是在「积分几乎不起作用」的前提下试出来的，
  换了正确的时间基准之后必须重新整定 —— 尤其是三个 Ki。
""")
