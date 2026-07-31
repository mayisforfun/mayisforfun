# K230 + MSPM0G3507 控球接入说明

## 当前已经具备

TI 工程的 `uart.c` 已按以下二进制帧解析 K230 坐标：

```text
AA AA XH XL YH YL FF FF
```

接线为：

- K230 TX -> TI PA9 / UART1 RX
- 如需 TI 回传：K230 RX <- TI PA8 / UART1 TX
- K230 GND 与 TI GND 必须共地
- UART 为 3.3 V TTL、115200、8N1

先观察：

- `g_uart_rx_count` 持续增加：物理 UART 已收到字节。
- `g_k230_frame_count` 持续增加：8 字节协议解析成功。
- `g_k230_center_x/y` 随小球移动：视觉坐标链路成功。

如果第一个计数不增加，优先检查共地、TX/RX、波特率和电平，不要先调 PID。

## PID 接入顺序

在 `main.c` 创建：

```c
#include "ti_ball_control/ti_ball_control.h"

static TIBallControl ball_control;
volatile uint8_t g_ball_task_request = 0U;
```

初始化：

```c
TIBallControlConfig cfg = TIBallControl_defaultConfig();
cfg.pixels_per_cm = 20.0f; /* 必须实测 */
cfg.position_sign = 1;
cfg.servo_sign = 1;
TIBallControl_init(&ball_control, &cfg);
```

每个 10 ms 主循环在 `UART1_poll()` 之后运行：

```c
if (g_k230_frame_event) {
    /* 水管沿画面 X 方向时用 center_x，沿 Y 方向则换成 center_y。 */
    TIBallControl_pushVision(&ball_control, g_k230_center_x, g_sys_ticks);
}

if (g_ball_task_request != (uint8_t) ball_control.task) {
    (void) TIBallControl_setTask(&ball_control,
        (TIBallTask) g_ball_task_request, g_sys_ticks);
}

TIBallControl_update(&ball_control, g_sys_ticks, &ball_servo);
ServoControl_update(&ball_servo);
```

小球放好原点后调用一次：

```c
TIBallControl_requestOriginCapture(&ball_control);
```

下一帧视觉坐标即定义为 0 cm。任务 4 必须先收到有效坐标，再选择任务。

## 四种任务

- `1`：小车静止，小球先到 +5 cm，稳定 1 秒后自动切到 -5 cm。
- `2`：直线运动时保持原点 0。
- `3`：绕圈时保持原点 0。
- `4`：选择任务瞬间捕获当前任意位置，绕圈时保持该位置。

任务 2 和任务 3 的球位置 PID 相同，区别在小车运动状态机；PID 会利用视觉反馈自动抵抗直线加减速和绕圈扰动。

## 当前硬件阻塞点

`board_port.h` 中 `SERVO_PWM_ENABLE` 当前为 0，SysConfig 尚未配置独立的 50 Hz 舵机 PWM。此时 PID 和 `ServoControl` 状态会更新，但引脚不会输出脉冲。

必须先确定一个空闲且支持定时器 CCP 输出的舵机信号脚，再在 SysConfig 中新增：

- 独立 PWM 定时器，频率 50 Hz
- 推荐计数频率 1 MHz、周期 20000
- 舵机通道 compare 对应 1000~2000 us

不能复用 PA12/PA13 的 TIMG0，因为它们正在输出电机 PWM。
