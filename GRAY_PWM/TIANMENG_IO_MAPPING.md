# 天猛星 IO 对应表

来源：`D:\qq\IO口对应.xlsx`。最终以你们实车接线、原理图、万用表测试为准。

## 五路灰度

表格原始 IO：

| 模块输出 | MSPM0 引脚 | 代码原始名字 |
|---|---:|---|
| OUT1 | PA26 | GRAY_L2 |
| OUT2 | PA27 | GRAY_L1 |
| OUT3 | PA24 | GRAY_C |
| OUT4 | PA25 | GRAY_R1 |
| OUT5 | PB24 | GRAY_R2 |

实测结果：

```text
全黑 = 0
全白 = 31
只 L2 = 15
只 L1 = 23
只 C  = 27
只 R1 = 29
只 R2 = 30
```

结论：

- 灰度模块是黑线输出高电平，所以代码使用 `GRAY_BLACK_IS_LOW = 0`。
- 实车灰度左右方向和代码原始顺序相反，所以 `board_port.c` 已经做了反向映射。

修正后期望：

```text
全白 = 0
L2 = 1
L1 = 2
C  = 4
R1 = 8
R2 = 16
全黑 = 31
```

## TB6612 电机驱动

| TB6612 | MSPM0 引脚 | 功能 |
|---|---:|---|
| AIN1 | PB23 | A 通道方向 1 |
| AIN2 | PB26 | A 通道方向 2 |
| BIN1 | PB08 | B 通道方向 1 |
| BIN2 | PB09 | B 通道方向 2 |
| PWMA | PA12 | TIMG0_C0 PWM |
| PWMB | PA13 | TIMG0_C1 PWM |
| STBY | PB27 或接高电平 | 驱动使能 |

当前实车处理：

- `MOTOR_FORWARD_SIGN = (-1)`：让代码输出后，车实际往前走。
- `set_motor_debug()` 里交换了 left/right：适配当前电机左右接线。

## 编码器预留

| 信号 | MSPM0 引脚 |
|---|---:|
| 左编码器 A | PA00 |
| 左编码器 B | PA01 |
| 右编码器 A | PB04 |
| 右编码器 B | PB05 |

当前主循环还没有接速度闭环，调灰度 PID 时先不用管这里。

## MPU6050

| 信号 | MSPM0 引脚 |
|---|---:|
| SCL | PA17 |
| SDA | PA16 |
| INT | PA14 |

基础循迹不依赖 MPU6050。