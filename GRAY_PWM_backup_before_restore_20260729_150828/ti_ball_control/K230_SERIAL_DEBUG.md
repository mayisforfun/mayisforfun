# K230 → TI 串口分级排查

## 第一步：脱离 YOLO 测试物理链路

在 K230 上单独运行 `k230_uart_link_test.py`，接线：

- K230 GPIO32 / UART3_TX → TI PA9 / UART1_RX
- K230 GND → TI GND
- 暂时不需要连接 TI PA8 → K230 GPIO33
- 两端均为 115200、8 数据位、无校验、1 停止位

TI 烧录当前工程后，在 CCS Expressions 中观察：

```text
g_uart_rx_count
g_uart_irq_count
g_k230_error_irq_count
g_k230_frame_count
g_k230_center_x
g_k230_center_y
g_k230_frame0 ... g_k230_frame7
```

正确结果：

- `g_uart_rx_count` 每秒约增加 160。
- `g_k230_frame_count` 每秒约增加 20。
- `g_k230_center_x` 固定为 258。
- `g_k230_center_y` 持续递增。
- `frame0..7` 为 `170,170,1,2,YH,YL,255,255`。
- `g_k230_error_irq_count` 不应持续增加。

## 第二步：根据变量定位

- `rx_count=0`：物理连接、引脚号、共地或 K230 UART3 没有真正输出。
- `rx_count` 增长但 `frame_count=0`：收到的字节不是约定帧，查看 `g_k230_raw0..7`。
- `error_irq_count` 增长：优先检查波特率、电平、地线和长线干扰。
- 固定帧正常、YOLO 脚本不正常：通信硬件无误，问题在视觉脚本的发送位置或发送内容。

## 第三步：接回 YOLO 坐标

原脚本第 194~201 行的 `send_position()` 当前全部被注释，第 210 行发送的是固定测试帧。固定帧通过后，应恢复为：

```python
tx_position = bytearray([0xAA, 0xAA, 0, 0, 0, 0, 0xFF, 0xFF])

def send_position(center_x, center_y):
    center_x = max(0, min(65535, int(center_x)))
    center_y = max(0, min(65535, int(center_y)))
    tx_position[2] = (center_x >> 8) & 0xFF
    tx_position[3] = center_x & 0xFF
    tx_position[4] = (center_y >> 8) & 0xFF
    tx_position[5] = center_y & 0xFF
    uart3.write(tx_position)
```

在 `draw_result()` 之后：

```python
if yolo_det.last_ball_center is not None:
    send_position(yolo_det.last_ball_center[0],
                  yolo_det.last_ball_center[1])
```

删除固定帧发送，并把 `sleep_ms(500)` 改成约 `sleep_ms(20)`；500 ms 更新一次对位置 PID 太慢。
