"""K230 -> TI 单向 UART 自检。

接线：K230 GPIO32/UART3_TX -> TI PA9/UART1_RX，GND 必须共地。
TI 端应看到 X 固定为 258，Y 从 0 开始递增，frame_count 每秒约增加 20。
"""

from machine import FPIOA, UART
import time


def make_frame(center_x, center_y):
    return bytes((
        0xAA, 0xAA,
        (center_x >> 8) & 0xFF,
        center_x & 0xFF,
        (center_y >> 8) & 0xFF,
        center_y & 0xFF,
        0xFF, 0xFF,
    ))


fpioa = FPIOA()
fpioa.set_function(32, FPIOA.UART3_TXD)
fpioa.set_function(33, FPIOA.UART3_RXD)
uart3 = UART(UART.UART3, 115200)

sequence = 0
while True:
    frame = make_frame(0x0102, sequence)
    written = uart3.write(frame)

    # 每秒打印一次，避免大量 print 干扰 UART 时序。
    if (sequence % 20) == 0:
        print("uart3 bytes=", written, "x=258 y=", sequence)

    sequence = (sequence + 1) & 0xFFFF
    time.sleep_ms(50)
