# 电赛小车代码交接备忘录

更新时间：2026-07-28
工程路径：C:\Users\17570\workspace_ccstheia\GRAY_PWM
远程仓库：https://github.com/mayisforfun/mayisforfun.git
当前已 push 提交：29d25ee Add contest communication and control modules

## 1. 当前总体状态

当前代码已经可以作为电赛前基础版本使用：

- 灰度循迹能跑稳。
- 丢线检测和找线功能可用。
- 十字/全黑停车测试可用。
- 电机方向已经按实际车修正。
- 编码器左右和方向已经按实际车修正。
- 速度闭环已经接入，但目前参数偏保守。
- 按键启停可用。
- 蜂鸣器是低电平触发，代码已反相处理。
- HC-05 蓝牙 UART3 已配置并接入，当前用于通信验证、安全启停和状态查询。

比赛前原则：能跑稳的地方不要乱大改。后续主要加赛题动作逻辑，不重写底层。

## 2. 硬件和 IO 关键信息

### 灰度传感器

来自 IO 表：

- OUT1 -> PA26
- OUT2 -> PA27
- OUT3 -> PA24
- OUT4 -> PA25
- OUT5 -> PB24

实际测试结果：

- 全黑：0，后来代码已按黑线为 1 做了转换/映射。
- 全白：31，后来根据实际逻辑修正后，主算法里黑线 bit = 1。
- 当前算法期望逻辑：L2=1，L1=2，C=4，R1=8，R2=16，全白=0，全黑=31。

注意：灰度板物理左右方向和代码最初理解相反，已经在 `board_port.c` 里做了左右映射修正。不要轻易再换灰度左右。

### 电机/TB6612

- AIN1 -> PB23
- AIN2 -> PB26
- BIN1 -> PB08
- BIN2 -> PB09
- PWMA -> PA12 / TIMG0_C0
- PWMB -> PA13 / TIMG0_C1
- STBY -> PB27，SysConfig 初始值 SET，高电平使能。

电机实际方向也做过修正：`main.c` 里 `MOTOR_FORWARD_SIGN = -1`，并且 `set_motor_debug()` 里左右输出有交换。不要只看变量名判断物理左右，要结合实际测试。

### 编码器

IO 表：

- 左 A -> PA00
- 左 B -> PA01
- 右 A -> PB04
- 右 B -> PB05

实际测试发现编码器左右接反、右侧方向也需要修正。现在在 `encoder.c` 中已经处理：

- 逻辑左轮读 raw right。
- 逻辑右轮读 raw left，并取反。

当前 `Encoder_getLeftTicks()` / `Encoder_getRightTicks()` 返回的是每次调用后的增量 tick，不是总 tick。

### 蜂鸣器

- BEEP -> PB25
- 蜂鸣器低电平触发。
- `BEEP_ACTIVE_LOW = 1`
- `Board_setBuzzer(true)` 会拉低并响。
- `Board_setBuzzer(false)` 会拉高并关闭。
- SysConfig 中 BEEP 初始值已设为 SET。

### HC-05 蓝牙

当前配置为 UART3：

- HC-05 TXD -> PB03 / UART3_RX
- HC-05 RXD -> PB02 / UART3_TX
- HC-05 GND -> GND
- HC-05 VCC -> 按模块要求接 5V 或 3.3V

`STATE` 和 `EN/KEY` 暂时不用接。

SysConfig 已生成：

- `BT_INST = UART3`
- `GPIO_BT_RX_PIN = PB03`
- `GPIO_BT_TX_PIN = PB02`
- 波特率 9600

注意：HC-05 很多默认波特率是 9600，当前工程已经改为 9600。

### MPU6050

当前使用 I2C1：

- MPU6050 VCC -> 3.3V 优先
- MPU6050 GND -> GND
- MPU6050 SDA -> PA16 / I2C1_SDA
- MPU6050 SCL -> PA17 / I2C1_SCL
- AD0 默认接 GND 或悬空，地址为 0x68
- INT、XDA、XCL 暂时不用接

上电后保持小车静止一小会儿，程序会估计 Z 轴陀螺仪零偏。蓝牙 `Q` 状态会回传 `MPU=1` 和 `YAW=`，用于确认 MPU6050 是否正常。

## 3. 主要代码模块

### board_port.c / board_port.h

负责底层硬件：

- 初始化 SysConfig。
- 灰度读取和物理左右映射。
- 电机 PWM 和方向脚控制。
- 按键读取。
- 蜂鸣器控制。
- 可选状态 LED 接口。

### line_follow.c / line_follow.h

负责灰度循迹算法：

- 读取 5 路灰度 bit。
- 根据权重算线的位置。
- 根据 PID 算左右轮速度差。
- 全白时进入丢线找线：左右轮一正一负原地找线。

重点调参：

- `kp_q10`：转向力度。大了转得狠，太大会抖。
- `kd_q10`：抑制抖动。大了会稳，但太大会反应慢。
- `max_speed`：最大逻辑速度。
- `lost_turn_speed`：丢线找线速度。

### track_fsm.c / track_fsm.h

负责基础赛道状态：

- `TRACK_STRAIGHT`：直道
- `TRACK_CURVE`：弯道
- `TRACK_CROSS`：十字/大面积黑线
- `TRACK_LOST`：丢线

它主要决定 base speed，不直接控制电机。

重点速度：

- `straight_speed`
- `curve_speed`
- `cross_speed`
- `lost_speed`

### speed_control.c / speed_control.h

负责编码器速度闭环 PID。

调试变量在 `main.c`：

- `g_speed_left_target_ticks`
- `g_speed_right_target_ticks`
- `g_speed_left_actual_ticks`
- `g_speed_right_actual_ticks`
- `g_speed_left_correction`
- `g_speed_right_correction`
- `g_speed_left_pwm`
- `g_speed_right_pwm`

简单判断：actual 接近 target，correction 不长期顶满，就算正常。

### cross_detector.h

路口检测模块，头文件实现。

功能：

- 判断当前灰度是否像路口。
- 路口计数。
- 防止同一个路口重复计数。

调试变量：

- `g_cross_candidate`
- `g_cross_event`
- `g_cross_count`
- `g_cross_latched`

### run_indicator.h

启动/停车提示模块。

当前逻辑：

- 启动短响。
- 运动中不亮灯、不响。
- 停车响一下。
- LED 接口目前是可选，SysConfig 没正式配置 STATUS_LED 时不会实际亮。

### bluetooth_uart.h / bluetooth_uart.c

HC-05 蓝牙模块。

当前已接入安全启停和状态查询；`L/R/M` 只做任务提示确认，不直接控制车。

支持命令：

- `G\n` -> `BT_CMD_GO = 1`
- `S\n` -> `BT_CMD_STOP = 2`
- `L\n` -> `BT_CMD_LEFT = 3`
- `R\n` -> `BT_CMD_RIGHT = 4`
- `M\n` -> `BT_CMD_MODE = 5`
- `Q\n` -> `BT_CMD_QUERY = 6`
- 其他 -> `BT_CMD_UNKNOWN = 7`

收到一行后的回复：

- `G\n`：回复 `OK GO`，启动小车，并重置循迹/FSM/路口/编码器/PID 状态。
- `S\n`：回复 `OK STOP`，停车，并重置循迹/FSM/路口/编码器/PID 状态。
- `Q\n`：回复类似 `STAT RUN=1 GRAY=4 TRACK=0 CROSS=0 LOST=0 STOP=0`。
- `L\n` / `R\n` / `M\n`：分别回复 `OK LEFT` / `OK RIGHT` / `OK MODE`。
- 空行回复 `OK`，未知命令回复 `ERR`。

调试变量：

- `g_bt_rx_count`
- `g_bt_line_count`
- `g_bt_last_line`
- `g_bt_last_cmd`
- `g_bt_line_event`
- `g_bt_overflow_count`

## 4. main.c 当前行为

当前正常主循环大致流程：

1. 等待 10ms SysTick 控制周期。
2. 轮询蓝牙 UART3，有新一行则记录并处理 `G/S/Q/L/R/M`。
3. 检测按键，切换 `g_run_enabled`。
4. 读取灰度。
5. 更新循迹算法。
6. 更新路口检测。
7. 更新赛道状态机。
8. 如果 `CROSS_STOP_TEST_ENABLE = 1`，检测到连续全黑 31 就停车。
9. 如果未启动，电机输出 0。
10. 如果启动，进入速度闭环输出电机 PWM。

当前已加入 2024 题任务层：

- KEY1：要求 1，A -> B，直行约 100cm 后停车蜂鸣。
- KEY2：要求 2，A -> B -> C -> D -> A，一圈后停车。
- KEY3：要求 3，A -> C -> B -> D -> A，一圈后停车。
- KEY4：要求 4，按要求 3 路径自动跑 4 圈后停车。

任务层关键调参在 `main.c` 顶部：

- `TASK_TICKS_PER_CM`：编码器距离换算，必须实车标定。
- `TASK_TURN_SIGN`：如果 MPU 控制转向方向反了，改成 -1。
- `TASK_DIAGONAL_DEG`：斜线段角度，当前约 39 度。
- `TASK_AB_CD_CM`、`TASK_AC_BD_CM`、`TASK_ARC_CM`：直线/斜线/半圆弧目标距离。

当前十字停车测试开关：

```c
#define CROSS_STOP_TEST_ENABLE 1
#define CROSS_STOP_FULL_BLACK_TICKS 2U
```

含义：连续约 20ms 全黑才停车。

如果后续不想十字自动停车，把 `CROSS_STOP_TEST_ENABLE` 改成 0。

## 5. 当前关键调参方向

如果车抖：

- 降低 `g_line_config.kp_q10`
- 或略微增加 `g_line_config.kd_q10`
- 或降低直道速度

如果转弯不够：

- 增大 `g_line_config.kp_q10`
- 或降低 `curve_speed`

如果弯道冲出去：

- 降低 `track_fsm.curve_speed`
- 降低 `straight_speed`

如果丢线找得慢：

- 增大 `lost_turn_speed`

如果十字误停：

- 增大 `CROSS_STOP_FULL_BLACK_TICKS`

如果十字停不住：

- 减小 `CROSS_STOP_FULL_BLACK_TICKS`
- 或降低速度

如果蓝牙乱码：

- 当前 BT UART 波特率是 9600；如果换过模块，再确认模块实际波特率。

## 6. K230 通信理解

K230 不是直接控制电机。正确分工：

- K230 负责视觉识别：看见什么。
- 天猛星负责底盘控制：怎么开车。

推荐协议：

- `L\n`：视觉看到左侧任务
- `R\n`：视觉看到右侧任务
- `S\n`：停车
- `G\n`：继续
- `Q\n`：查询状态，后续可加

K230 使用 UART1，蓝牙使用 UART3，不冲突。UART0 继续不碰。

后续如果要接 K230，建议写一个和 `bluetooth_uart.h` 类似的 `vision_uart.h`，但不要让 K230 直接改电机 PWM，只让它更新 `vision_cmd`，再由主状态机决定动作。

## 7. Git 状态

已 push 到 GitHub 的提交：

```text
29d25ee Add contest communication and control modules
```

push 成功记录：

```text
master -> master
243825f..29d25ee
```

如果后续新开对话，先让助手读取：

- `HANDOFF_MEMO.md`
- `main.c`
- `board_port.c/h`
- `line_follow.c/h`
- `track_fsm.c/h`
- `encoder.c/h`
- `speed_control.c/h`
- `bluetooth_uart.h`
- `cross_detector.h`
- `run_indicator.h`

## 8. 比赛前建议

后天开赛前不要大范围重构。

优先做：

1. 确认当前版本能重新烧录、能跑。
2. 蓝牙只测试收发，不先接入控制。
3. 如果题目要求短距离通信，再根据题目把 `G/S/L/R/M/Q` 映射到比赛动作。
4. K230 等题目明确视觉任务后再接入主状态机。
5. 保留当前稳定版本，改坏了就回退到 `29d25ee`。
