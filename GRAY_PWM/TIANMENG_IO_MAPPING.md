# IO 口对应表读取结果

来源文件：`D:\qq\IO口对应.xlsx`

表格名称显示为“天猛星引脚”。如果你手里的“天狼星/天猛星”开发板接口一致，可以按下面配置；如果板子版本不同，需要再对照原理图确认。

## 五路灰度模块

表格中灰度传感器位于右侧外/内排针：

| 灰度模块 | MSPM0 引脚 | 建议 SysConfig 名称 |
|---|---:|---|
| OUT1 | A26 | `GRAY_L2` |
| OUT2 | A27 | `GRAY_L1` |
| OUT3 | A24 | `GRAY_C` |
| OUT4 | A25 | `GRAY_R1` |
| OUT5 | B24 | `GRAY_R2` |

代码默认顺序是：

```text
L2, L1, C, R1, R2
```

如果你实际模块排线方向相反，最简单的处理是交换 `GRAY_L2` 和 `GRAY_R2`、交换 `GRAY_L1` 和 `GRAY_R1`。

## TB6612 电机驱动

表格中明确给出的 TB6612 控制引脚：

| TB6612 引脚 | MSPM0 引脚 | 功能 | 建议 SysConfig 名称 |
|---|---:|---|---|
| BIN1 | B08 | GPIO 输出 | `MOTOR_BIN1` |
| BIN2 | B09 | GPIO 输出 | `MOTOR_BIN2` |
| PWMA | A12 | PWM 输出，`TIMG0_C0` | 左电机 PWM |
| PWMB | A13 | PWM 输出，`TIMG0_C1` | 右电机 PWM |
| AIN1 | B23 | GPIO 输出 | `MOTOR_AIN1` |
| AIN2 | B26 | GPIO 输出 | `MOTOR_AIN2` |

当前代码默认：

```text
TB6612 A 通道 = 左电机
TB6612 B 通道 = 右电机
```

也就是：

```text
AIN1/AIN2/PWMA -> 左电机
BIN1/BIN2/PWMB -> 右电机
```

如果你的小车实际接线是 A 通道右电机、B 通道左电机，有两种处理方式：

1. 交换电机接线；
2. 在 `Board_setMotorSpeed()` 里交换 left/right 输出。

## 可能是编码器输入的引脚

表格前半部分还有这些“电机”相关 IO：

| 表格标注 | MSPM0 引脚 | 可能用途 |
|---|---:|---|
| 电机，A_L | A00 | 左电机编码器 A 相 |
| 电机，B_L | A01 | 左电机编码器 B 相 |
| 电机，A_R | B04 | 右电机编码器 A 相 |
| 电机，B_R | B05 | 右电机编码器 B 相 |

当前五路灰度循迹代码暂时不需要编码器。后面要做速度闭环 PID 时再配置这四个 GPIO/中断或定时器捕获。

## MPU6050 / 陀螺仪

表格中陀螺仪接口为：

| 模块引脚 | MSPM0 引脚 | 功能 |
|---|---:|---|
| T_SCL | A17 | `I2C1_SCL` |
| T_SDA | A16 | `I2C1_SDA` |
| INT | A14 | GPIO 输入 |

当前 `mpu6050.c` 使用轮询 I2C，不需要先接入 `INT`。SysConfig 里先配置
A17/A16 为 I2C Controller 即可。

## CCS / SysConfig 配置建议

### GPIO 输入组 `GRAY`

建立 GPIO input group：`GRAY`

```text
L2 -> A26
L1 -> A27
C  -> A24
R1 -> A25
R2 -> B24
```

### GPIO 输出组 `MOTOR`

建立 GPIO output group：`MOTOR`

```text
AIN1 -> B23
AIN2 -> B26
BIN1 -> B08
BIN2 -> B09
```

`STBY` 没有在表格中看到明确分配。如果 TB6612 模块的 `STBY` 接到了排针，请配置成 GPIO 输出并命名为 `STBY`；如果没有，就直接把 `STBY` 接 3.3 V。

### PWM

配置一个 Timer PWM：

```text
TIMG0_C0 -> A12 -> TB6612 PWMA
TIMG0_C1 -> A13 -> TB6612 PWMB
```

需要同时核对 Timer PWM 的 period/load 是否等于代码里的
`MOTOR_PWM_PERIOD_COUNTS`。当前代码默认是 `1000U`。

还要核对 compare 与占空比方向。当前代码默认：

```c
#define MOTOR_PWM_COMPARE_IS_INVERTED 1
```

即 `compare = period - duty`。如果 SysConfig 配的是正向 PWM，请改为 `0`。

如果 SysConfig 生成的 PWM 实例名不是 `PWM_0_INST` / `PWM_1_INST`，需要修改 `board_port.c` 顶部的：

```c
#define LEFT_PWM_INST PWM_0_INST
#define LEFT_PWM_CC_INDEX DL_TIMER_CC_0_INDEX
#define RIGHT_PWM_INST PWM_1_INST
#define RIGHT_PWM_CC_INDEX DL_TIMER_CC_0_INDEX
```

若左右 PWM 用同一个 Timer，也可以写成类似：

```c
#define LEFT_PWM_INST PWM_0_INST
#define LEFT_PWM_CC_INDEX DL_TIMER_CC_0_INDEX
#define RIGHT_PWM_INST PWM_0_INST
#define RIGHT_PWM_CC_INDEX DL_TIMER_CC_1_INDEX
```
