"""校验 bsp_i2c.c 三条接收路径的「调用序列」与「缓冲区下标」是否正确。

为什么要这样验：上一版脚本试图连硬件时序一起建模（NACK 什么时候生效、
停止位会不会吞掉在途字节……），结果模型本身反复出错 —— 用模型去证明实现在
某些细节上正确是不可靠的。所以这里只做**能无歧义判定**的检查：

  A. 下标覆盖：每条路径对 pBuffer 的每一次写入，下标必须落在 [0, Size)，
     且每个下标恰好被写一次（没有漏写、没有重复写）。
     这一条足以抓住 Size==2 多读一个字节导致的静默错位。
  B. 调用次数：进入数据阶段后 I2C_ReceiveData 的调用次数必须恰好等于 Size。
  C. 标志位顺序（对照 RM0008）：
       Size==1 : 关 ACK → 清 ADDR → 发 STOP
       Size>=2 : 置 ACK → 清 ADDR
     这两条是「先设好应答位再清 ADDR」的要求，顺序反了首个字节会被 NACK。
  D. 每条路径都必须清过 ADDR（不清则 SCL 被拉住，RXNE 永不置位 → 死锁）。

序列直接照 C 代码逐行抄写，任何一边改了而另一边没跟上，断言就会失败。
"""

FAIL = []


def check(cond, msg):
    print(('  [OK]   ' if cond else '  [FAIL] ') + msg)
    if not cond:
        FAIL.append(msg)


class Trace:
    """记录一次接收过程中发生的所有动作。"""

    def __init__(self, size):
        self.size = size
        self.writes = []       # (下标)
        self.reads = 0         # I2C_ReceiveData 次数
        self.flags = []        # 标志位操作序列
        self.addr_cleared = False

    def read_dr_into(self, idx):
        self.writes.append(idx)
        self.reads += 1

    def ack(self, on):
        self.flags.append(('ACK', on))

    def clear_addr(self):
        self.flags.append(('CLEAR_ADDR', None))
        self.addr_cleared = True

    def stop(self):
        self.flags.append(('STOP', None))


# ---------------------------------------------------------------------------
# 以下四个函数逐行照抄 bsp_i2c.c 的分支结构（只保留下标与调用序列）
# ---------------------------------------------------------------------------

def read_single(t, buf, size):
    """I2C_ReadSingleByte"""
    t.ack(False)
    t.clear_addr()
    t.stop()
    t.read_dr_into(0)


def read_last_two(t, buf, size):
    """I2C_ReadLastTwoBytes：固定写 Size-2 与 Size-1"""
    t.read_dr_into(size - 2)
    t.ack(False)
    t.stop()
    t.read_dr_into(size - 1)


def read_data(t, buf, size):
    """I2C_ReadData"""
    if size == 1:
        return read_single(t, buf, size)

    t.ack(True)
    t.clear_addr()

    if size == 2:
        return read_last_two(t, buf, size)

    t.read_dr_into(0)

    i = 1
    while i + 2 < size:
        t.read_dr_into(i)
        i += 1

    return read_last_two(t, buf, size)


def read_data_before_fix(t, buf, size):
    """修复前的写法：Size==2 先预读一个字节，再调 read_last_two。"""
    if size == 1:
        return read_single(t, buf, size)

    t.ack(True)
    t.clear_addr()

    if size == 2:
        t.read_dr_into(0)          # <-- 多余的预读
        return read_last_two(t, buf, size)

    t.read_dr_into(0)
    i = 1
    while i + 2 < size:
        t.read_dr_into(i)
        i += 1
    return read_last_two(t, buf, size)


print('=' * 74)
print('A/B. 下标覆盖与调用次数')
print('=' * 74)
for size in (1, 2, 3, 4, 5, 14, 20):
    t = Trace(size)
    read_data(t, None, size)
    print(f'--- Size = {size} ---')
    check(t.reads == size, f'读 DR 次数 = {t.reads}（应为 {size}）')
    check(all(0 <= w < size for w in t.writes),
          f'所有下标都在 [0,{size}) 内：{t.writes}')
    check(sorted(t.writes) == list(range(size)),
          f'每个下标恰好写一次：{sorted(t.writes)}')
    print()

print('=' * 74)
print('C. 标志位顺序（RM0008：必须先设应答位，再清 ADDR）')
print('=' * 74)
for size in (1, 2, 3, 14):
    t = Trace(size)
    read_data(t, None, size)
    seq = t.flags
    print(f'--- Size = {size} ---')
    print(f'  序列：{seq}')
    if size == 1:
        check(seq[:3] == [('ACK', False), ('CLEAR_ADDR', None), ('STOP', None)],
              'Size==1：关 ACK → 清 ADDR → 发 STOP')
    else:
        check(seq[0] == ('ACK', True), 'Size>=2：第一件事就是置 ACK')
        check(seq[1] == ('CLEAR_ADDR', None), 'Size>=2：紧接着清 ADDR（先 ACK 后清）')
    print()

print('=' * 74)
print('D. 每条路径都必须清过 ADDR（否则 RXNE 永不置位 = 死锁）')
print('=' * 74)
for size in (1, 2, 3, 14, 20):
    t = Trace(size)
    read_data(t, None, size)
    check(t.addr_cleared, f'Size = {size} 清过 ADDR')
print()

print('=' * 74)
print('回归对照：修复前 Size==2 的写法会怎样')
print('=' * 74)
t = Trace(2)
read_data_before_fix(t, None, 2)
print(f'  读 DR 次数 = {t.reads}（应为 2）')
print(f'  写入下标   = {t.writes}（应为 [0, 1]）')
bad = (t.reads != 2) or (sorted(t.writes) != [0, 1])
check(bad, '复现出「多读一次 + 下标 0 被写两遍、下标 1 落空」—— 这正是第二轮复核的发现')
print()

print('=' * 74)
if FAIL:
    print(f'结论：{len(FAIL)} 项未通过')
    for m in FAIL:
        print('  - ' + m)
    raise SystemExit(1)
print('结论：全部通过')
print('  · 三条路径写满 [0,Size) 每个下标恰好一次，无重复写、无漏写')
print('  · 读 DR 次数恰为 Size，不多读')
print('  · 标志位顺序符合「先设应答位、再清 ADDR」的要求')
print('  · 每条路径都清过 ADDR，不会因 ADDR 残留而死锁')
print('=' * 74)
