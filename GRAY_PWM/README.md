# MSPM0G3507 五路灰度循迹小车

本工程面向电赛小车快速验证：MSPM0G3507 + 五路数字灰度 + TB6612 差速驱动，可扩展 MPU6050、编码器测速和赛道状态机。

## 当前状态

- `main.c` 默认打开 `MOTOR_DIAGNOSTIC_MODE = 1`，上电后只做电机正反转诊断，不会循迹。
- 要进入循迹模式，把 `main.c` 顶部改为 `#define MOTOR_DIAGNOSTIC_MODE 0`，再重新编译下载。
- 控制周期由 `Board_init()` 中 SysTick 设置，当前为 100 Hz，即 10 ms 一次，不是 1 ms。
- 主循环当前只调用 `LineFollow_update()` 和 `Board_setMotorSpeed()`；`encoder.c`、`speed_control.c`、`track_fsm.c` 已提供扩展模块，但尚未接入默认主循环。

## 文件分工

| 文件 | 作用 | 比赛调试关注点 |
|---|---|---|
| `main.c` | 诊断模式 / 循迹主循环入口 | 先用诊断模式确认电机方向，再关闭诊断模式跑线 |
| `board_port.c/.h` | GPIO、PWM、电机方向、灰度读取 | 引脚名、PWM 计数周期、黑线电平极性 |
| `line_follow.c/.h` | 五路灰度位置估计和 PID/PD 修正 | `kp_q10`、`kd_q10`、`base_speed`、`lost_turn_speed` |
| `encoder.c/.h` | 编码器 1 倍频计数框架 | 需要在主循环中显式接入才生效 |
| `speed_control.c/.h` | 速度 PID 框架 | `SPEED_TICKS_PER_1000` 必须实测 |
| `track_fsm.c/.h` | 直道、弯道、十字、丢线状态机 | 当前未接入 `main.c` |
| `mpu6050.c/.h` | MPU6050 I2C 读取 | 当前未参与循迹控制 |
| `TIANMENG_IO_MAPPING.md` | 天猛星/天狼星板 IO 对照 | 接线和 SysConfig 命名依据 |

## 必配 SysConfig 外设

### 1. 五路灰度 GPIO 输入

建立 5 个 GPIO 输入，建议名称如下：

| 位置 | MSPM0 引脚 | 建议名称 |
|---|---:|---|
| L2 最左 | A26 | `GRAY_L2` |
| L1 左中 | A27 | `GRAY_L1` |
| C 中间 | A24 | `GRAY_C` |
| R1 右中 | A25 | `GRAY_R1` |
| R2 最右 | B24 | `GRAY_R2` |

代码默认认为灰度模块检测到黑线时输出低电平：

```c
#define GRAY_BLACK_IS_LOW 1
```

如果你的模块黑线输出高电平，改为：

```c
#define GRAY_BLACK_IS_LOW 0
```

### 2. TB6612 方向 GPIO 输出

| TB6612 | MSPM0 引脚 | 建议名称 |
|---|---:|---|
| AIN1 | B23 | `MOTOR_AIN1` |
| AIN2 | B26 | `MOTOR_AIN2` |
| BIN1 | B08 | `MOTOR_BIN1` |
| BIN2 | B09 | `MOTOR_BIN2` |
| STBY | 3.3 V 或 GPIO | 若接 GPIO，建议命名 `MOTOR_STBY` |

默认约定：A 通道为左电机，B 通道为右电机。若小车前进方向反了，优先调换对应电机的 `AIN1/AIN2` 或 `BIN1/BIN2`；若左右电机接反，可交换电机接线或在 `Board_setMotorSpeed()` 中交换 left/right。

### 3. 两路 PWM

推荐使用同一个 Timer 的两个通道：

| TB6612 | MSPM0 引脚 | Timer 通道 | 代码默认 |
|---|---:|---|---|
| PWMA | A12 | `TIMG0_C0` | `LEFT_PWM_INST = PWM_0_INST`, `LEFT_PWM_CC_INDEX = DL_TIMER_CC_0_INDEX` |
| PWMB | A13 | `TIMG0_C1` | `RIGHT_PWM_INST = PWM_0_INST`, `RIGHT_PWM_CC_INDEX = DL_TIMER_CC_1_INDEX` |

PWM 周期要与代码一致：

```c
#define MOTOR_PWM_PERIOD_COUNTS 1000U
```

如果 SysConfig 的 Timer period/load 不是 1000，必须同步修改该宏，否则 `base_speed`、`max_speed` 和占空比比例都会失真。

当前代码默认使用反向 compare：

```c
#define MOTOR_PWM_COMPARE_IS_INVERTED 1
```

即 `compare = period - duty`。若实际现象是速度越大占空比越小，或速度为 0 时电机满速，把它改成 `0`。

## 灰度位定义

`Board_readGray5()` 返回 5 bit，黑线识别到时对应 bit 为 1：

```text
传感器：L2  L1  C   R1  R2
bit：   0   1   2   3   4
权重： -2000 -1000 0 1000 2000
```

典型状态：

- `C`：车在中线，直行。
- `L1 + C` 或 `L2 + L1`：线在左侧，应左修正。
- `C + R1` 或 `R1 + R2`：线在右侧，应右修正。
- `0x00`：丢线，按上一次误差原地找线。
- `0x1F`：全黑，代码按十字/交叉线低速通过。
- 非连续组合，例如 `L2 + R2`：默认判为异常，沿用上一帧有效灰度状态。

## 调车流程

1. 架空车轮，保持 `MOTOR_DIAGNOSTIC_MODE = 1`，确认左右轮都能正转、停止、反转。
2. 若某一侧方向反，先调电机方向引脚或电机线，不要急着改 PID。
3. 用串口/调试变量观察 `g_gray_bits`，确认五路灰度从左到右对应 bit 0 到 bit 4。
4. 关闭诊断模式：`#define MOTOR_DIAGNOSTIC_MODE 0`。
5. 低速起跑，建议初值：

```c
.base_speed = 300,
.max_speed = 650,
.kp_q10 = 120,
.ki_q10 = 0,
.kd_q10 = 180,
.lost_turn_speed = 220,
```

6. 直线摆动大：降低 `kp_q10` 或提高 `kd_q10`。
7. 弯道跟不上：提高 `kp_q10` 或降低 `base_speed`。
8. 丢线找线太猛：降低 `lost_turn_speed`。
9. 车速提高后再逐步增加 `base_speed` 和 `max_speed`，每次只改一个参数。

`kp_q10`、`ki_q10`、`kd_q10` 是 Q10 定点数，实际系数 = 参数 / 1024。例如 `150 / 1024 = 0.146`。

## 编码器和速度闭环

编码器引脚已在 `encoder.c` 中给出：

| 信号 | MSPM0 引脚 |
|---|---:|
| 左 A | A00 |
| 左 B | A01 |
| 右 A | B04 |
| 右 B | B05 |

当前 `main.c` 没有调用 `Encoder_init()`、`Encoder_getLeftTicks()`、`Encoder_getRightTicks()` 或 `SpeedPID_compute()`。如果要做速度闭环，需要先把编码器初始化和速度 PID 接入 10 ms 控制循环，并实测 `SPEED_TICKS_PER_1000`。

## MPU6050

| 信号 | MSPM0 引脚 | 功能 |
|---|---:|---|
| T_SCL | A17 | `I2C1_SCL` |
| T_SDA | A16 | `I2C1_SDA` |
| INT | A14 | GPIO 输入，可先不接 |

SysConfig 中添加 I2C Controller，SCL 选 A17，SDA 选 A16，建议先用 100 kHz。默认地址为 `0x68`；若 AD0 接高电平，改用 `MPU6050_I2C_ADDR_AD0_HIGH`。

## 常见问题

- 编译报 `PWM_0_INST` 未定义：SysConfig 生成的 PWM 实例名不同，改 `board_port.c` 顶部 `LEFT_PWM_INST` / `RIGHT_PWM_INST`。
- 下载后不循迹：检查 `MOTOR_DIAGNOSTIC_MODE` 是否仍为 `1`。
- 灰度全反：检查 `GRAY_BLACK_IS_LOW`。
- 电机速度数值改了无效：核对 Timer period/load 是否等于 `MOTOR_PWM_PERIOD_COUNTS`。
- 车一放地就冲出去：先把 `base_speed` 降到 250 到 300，确认方向和灰度 bit 后再加速。
