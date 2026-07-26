# 天猛星 / 天狼星 IO 对应表

来源文件：`D:\qq\IO口对应.xlsx`。表格名称显示为“天猛星引脚”。如果开发板版本不同，最终以原理图和万用表核对为准。

## 五路灰度模块

| 模块输出 | 物理位置建议 | MSPM0 引脚 | 建议 SysConfig 名称 | 代码 bit |
|---|---|---:|---|---:|
| OUT1 | 最左 L2 | A26 | `GRAY_L2` | 0 |
| OUT2 | 左中 L1 | A27 | `GRAY_L1` | 1 |
| OUT3 | 中间 C | A24 | `GRAY_C` | 2 |
| OUT4 | 右中 R1 | A25 | `GRAY_R1` | 3 |
| OUT5 | 最右 R2 | B24 | `GRAY_R2` | 4 |

如果排线方向相反，交换 `GRAY_L2` 与 `GRAY_R2`、`GRAY_L1` 与 `GRAY_R1`。

## TB6612 电机驱动

| TB6612 引脚 | MSPM0 引脚 | 功能 | 建议 SysConfig 名称 |
|---|---:|---|---|
| AIN1 | B23 | 左电机方向 1 | `MOTOR_AIN1` |
| AIN2 | B26 | 左电机方向 2 | `MOTOR_AIN2` |
| BIN1 | B08 | 右电机方向 1 | `MOTOR_BIN1` |
| BIN2 | B09 | 右电机方向 2 | `MOTOR_BIN2` |
| PWMA | A12 | `TIMG0_C0` PWM | `PWM_0` / CC0 |
| PWMB | A13 | `TIMG0_C1` PWM | `PWM_0` / CC1 |
| STBY | 3.3 V 或 GPIO | 驱动使能 | 若接 GPIO，建议 `MOTOR_STBY` |

当前代码默认：

```text
AIN1/AIN2/PWMA -> 左电机
BIN1/BIN2/PWMB -> 右电机
```

若实车左右通道相反，优先交换电机接线；不方便改线时，再在 `Board_setMotorSpeed()` 中交换 left/right 输出。

## 编码器预留

| 表格标注 | MSPM0 引脚 | 建议用途 |
|---|---:|---|
| 电机 A_L | A00 | 左编码器 A 相，中断 |
| 电机 B_L | A01 | 左编码器 B 相，方向判定 |
| 电机 A_R | B04 | 右编码器 A 相，中断 |
| 电机 B_R | B05 | 右编码器 B 相，方向判定 |

`encoder.c` 已按 1 倍频方式写好基本计数框架，但默认主循环未接入速度闭环。

## MPU6050

| 模块引脚 | MSPM0 引脚 | 功能 |
|---|---:|---|
| T_SCL | A17 | `I2C1_SCL` |
| T_SDA | A16 | `I2C1_SDA` |
| INT | A14 | GPIO 输入，可先不接 |

`mpu6050.c` 使用轮询 I2C，基础循迹不依赖 `INT`。

## SysConfig 核对清单

- `GRAY` 输入组：A26、A27、A24、A25、B24。
- `MOTOR` 输出组：B23、B26、B08、B09。
- PWM：A12 = `TIMG0_C0`，A13 = `TIMG0_C1`，period/load 与 `MOTOR_PWM_PERIOD_COUNTS` 一致。
- 若使用 STBY GPIO，上电初始化后必须输出高电平。
- 若黑线电平与代码相反，改 `GRAY_BLACK_IS_LOW`。
