# GRAY_PWM 调车说明

本工程用于 MSPM0G3507 天猛星小车：五路数字灰度 + TB6612 电机驱动。当前主程序已经切到灰度循迹模式。

## 当前状态

- `main.c` 当前为 `#define MOTOR_DIAGNOSTIC_MODE 0`，上电后走灰度循迹。
- 电机实际前进方向和代码默认方向相反，所以 `MOTOR_FORWARD_SIGN` 当前为 `(-1)`。
- 左右电机输出当前在 `set_motor_debug()` 里做了交换，因为实车左右通道和代码逻辑相反。
- 编码器和速度闭环文件已经存在，但当前主循环还没有接入速度闭环。

## 当前必须先确认的变量

烧录后先在 CCS 里看这些变量：

```c
g_gray_bits
g_line_error
g_line_left_speed
g_line_right_speed
g_motor_left_cmd
g_motor_right_cmd
```

灰度修正后，手动压黑线时应看到：

```text
全白：0
只 L2 压黑线：1
只 L1 压黑线：2
只 C  压黑线：4
只 R1 压黑线：8
只 R2 压黑线：16
全黑：31
```

如果不是这个顺序，先不要调 PID，先查灰度接线、灰度板方向、黑白电平。

## 主要调参位置

在 `main.c` 的 `g_line_config`：

```c
.base_speed      = 220,
.max_speed       = 330,
.kp_q10          = 70,
.ki_q10          = 0,
.kd_q10          = 140,
.lost_turn_speed = 180,
```

参数含义：

- `base_speed`：正常巡线速度。觉得车快，先降它。
- `max_speed`：左右轮修正时的最高速度。弯道突然冲，降它。
- `kp_q10`：看见偏差后修正力度。转不过来就加，左右摆就减。
- `ki_q10`：积分项。初期保持 0，不要动。
- `kd_q10`：抑制摆动和提前刹一点。左右摆时可以适当加。
- `lost_turn_speed`：丢线后原地找线速度。找线太猛就降。

## 调参顺序

1. 先确认灰度变量顺序正确。
2. 先用低速：`base_speed = 180~250`，`max_speed = base_speed + 100` 左右。
3. 如果车不跟线，先加 `kp_q10`。
4. 如果车左右摆，先减 `kp_q10` 或加一点 `kd_q10`。
5. 如果弯道冲出去，先降 `base_speed`，再考虑加 `kp_q10`。
6. `ki_q10` 暂时一直保持 0。

## PWM 说明

当前 `board_port.h`：

```c
#define MOTOR_PWM_COMPARE_IS_INVERTED 1
#define MOTOR_PWM_PERIOD_COUNTS 1000U
```

你实测过“数值越小转得越快”，所以当前用反向 compare 是合理的。后面如果发现速度逻辑又反了，再改这个宏。

## 丢线逻辑

`line_follow.c` 里已经有丢线检测：当 `active_count == 0`，也就是 5 个灰度都没看到黑线时，程序会按上一次偏差方向原地找线，速度由 `lost_turn_speed` 决定。

## 速度闭环说明

`encoder.c` 和 `speed_control.c` 目前只是准备好的模块，主循环还没用。现在先把灰度循迹调通，再接速度闭环。否则灰度、方向、编码器、速度 PID 会混在一起，很难定位问题。