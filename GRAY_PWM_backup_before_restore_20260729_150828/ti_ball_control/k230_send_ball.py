"""K230 端只负责把视觉算法得到的小球中心坐标发送给 TI。

不要用 print(frame)，也不要发送十六进制文本；必须 uart.write() 原始 bytes。
UART 的具体构造方式因 CanMV/MicroPython 固件而异，所以这里不绑定 UART 编号。
"""


def make_ball_frame(center_x, center_y):
    center_x = max(0, min(65535, int(center_x)))
    center_y = max(0, min(65535, int(center_y)))
    return bytes((
        0xAA, 0xAA,
        (center_x >> 8) & 0xFF, center_x & 0xFF,
        (center_y >> 8) & 0xFF, center_y & 0xFF,
        0xFF, 0xFF,
    ))


def send_ball_position(uart, center_x, center_y):
    uart.write(make_ball_frame(center_x, center_y))


# 视觉主循环示例：
# while True:
#     center = find_ball_center()  # 你的识别函数，返回 (x, y) 或 None
#     if center is not None:
#         send_ball_position(uart, center[0], center[1])
