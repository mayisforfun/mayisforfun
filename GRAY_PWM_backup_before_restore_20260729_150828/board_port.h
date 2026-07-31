#ifndef BOARD_PORT_H_
#define BOARD_PORT_H_

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * Pin mapping from D:\qq\IO???.xlsx.
 * Gray input order is corrected in board_port.c for the actual car wiring.
 *
 * Gray physical pins:
 *   OUT1 -> PA26
 *   OUT2 -> PA27
 *   OUT3 -> PA24
 *   OUT4 -> PA25
 *   OUT5 -> PB24
 *
 * TB6612:
 *   AIN1 -> PB23
 *   AIN2 -> PB26
 *   BIN1 -> PB08
 *   BIN2 -> PB09
 *   PWMA -> PA12 / TIMG0_C0
 *   PWMB -> PA13 / TIMG0_C1
 *   BEEP -> PB25
 */

#ifndef GRAY_BLACK_IS_LOW
#define GRAY_BLACK_IS_LOW 0
#endif

#ifndef GRAY_SAMPLE_COUNT
#define GRAY_SAMPLE_COUNT 3U
#endif

#ifndef GRAY_SAMPLE_INTERVAL_CYCLES
#define GRAY_SAMPLE_INTERVAL_CYCLES 0U
#endif

#ifndef MOTOR_PWM_COMPARE_IS_INVERTED
#define MOTOR_PWM_COMPARE_IS_INVERTED 1
#endif

#ifndef MOTOR_PWM_PERIOD_COUNTS
#define MOTOR_PWM_PERIOD_COUNTS 1000U
#endif

#ifndef GRAY_L2_PORT
#define GRAY_L2_PORT GPIOA
#endif
#ifndef GRAY_L2_PIN
#ifdef GRAY_L2_PIN_0_PIN
#define GRAY_L2_PIN  GRAY_L2_PIN_0_PIN
#else
#define GRAY_L2_PIN  DL_GPIO_PIN_26
#endif
#endif

#ifndef GRAY_L1_PORT
#define GRAY_L1_PORT GPIOA
#endif
#ifndef GRAY_L1_PIN
#ifdef GRAY_L1_PIN_1_PIN
#define GRAY_L1_PIN  GRAY_L1_PIN_1_PIN
#else
#define GRAY_L1_PIN  DL_GPIO_PIN_27
#endif
#endif

#ifndef GRAY_C_PORT
#define GRAY_C_PORT GPIOA
#endif
#ifndef GRAY_C_PIN
#ifdef GRAY_C_PIN_2_PIN
#define GRAY_C_PIN   GRAY_C_PIN_2_PIN
#else
#define GRAY_C_PIN  DL_GPIO_PIN_24
#endif
#endif

#ifndef GRAY_R1_PORT
#define GRAY_R1_PORT GPIOA
#endif
#ifndef GRAY_R1_PIN
#ifdef GRAY_R1_PIN_3_PIN
#define GRAY_R1_PIN  GRAY_R1_PIN_3_PIN
#else
#define GRAY_R1_PIN  DL_GPIO_PIN_25
#endif
#endif

#ifndef GRAY_R2_PORT
#define GRAY_R2_PORT GPIOB
#endif
#ifndef GRAY_R2_PIN
#ifdef GRAY_R2_PIN_4_PIN
#define GRAY_R2_PIN  GRAY_R2_PIN_4_PIN
#else
#define GRAY_R2_PIN  DL_GPIO_PIN_24
#endif
#endif

#ifndef MOTOR_AIN1_PORT
#define MOTOR_AIN1_PORT GPIOB
#endif
#ifndef MOTOR_AIN1_PIN
#ifdef MOTOR_AIN1_PIN_5_PIN
#define MOTOR_AIN1_PIN MOTOR_AIN1_PIN_5_PIN
#else
#define MOTOR_AIN1_PIN  DL_GPIO_PIN_23
#endif
#endif

#ifndef MOTOR_AIN2_PORT
#define MOTOR_AIN2_PORT GPIOB
#endif
#ifndef MOTOR_AIN2_PIN
#ifdef MOTOR_AIN2_PIN_6_PIN
#define MOTOR_AIN2_PIN MOTOR_AIN2_PIN_6_PIN
#else
#define MOTOR_AIN2_PIN  DL_GPIO_PIN_26
#endif
#endif

#ifndef MOTOR_BIN1_PORT
#define MOTOR_BIN1_PORT GPIOB
#endif
#ifndef MOTOR_BIN1_PIN
#ifdef MOTOR_BIN1_PIN_7_PIN
#define MOTOR_BIN1_PIN MOTOR_BIN1_PIN_7_PIN
#else
#define MOTOR_BIN1_PIN  DL_GPIO_PIN_8
#endif
#endif

#ifndef MOTOR_BIN2_PORT
#define MOTOR_BIN2_PORT GPIOB
#endif
#ifndef MOTOR_BIN2_PIN
#ifdef MOTOR_BIN2_PIN_8_PIN
#define MOTOR_BIN2_PIN MOTOR_BIN2_PIN_8_PIN
#else
#define MOTOR_BIN2_PIN  DL_GPIO_PIN_9
#endif
#endif

#ifndef START_BUTTON_PORT
#define START_BUTTON_PORT GPIOA
#endif
#ifndef START_BUTTON_PIN
#define START_BUTTON_PIN DL_GPIO_PIN_7
#endif
#ifndef START_BUTTON_IOMUX
#define START_BUTTON_IOMUX IOMUX_PINCM14
#endif
#ifndef START_BUTTON_ACTIVE_LOW
#define START_BUTTON_ACTIVE_LOW 1
#endif

#ifndef STATUS_LED_ENABLE
#define STATUS_LED_ENABLE 0
#endif

#ifndef BEEP_ACTIVE_LOW
#define BEEP_ACTIVE_LOW 1
#endif

#ifndef BEEP_PORT
#define BEEP_PORT GPIOB
#endif
#ifndef BEEP_PIN
#ifdef BEEP_PIN_9_PIN
#define BEEP_PIN BEEP_PIN_9_PIN
#else
#define BEEP_PIN DL_GPIO_PIN_25
#endif
#endif

/* 舵机 PWM 板级开关。
 *
 * 当前工程的 PWM_0 已经给左右电机使用，舵机必须使用另一个 50Hz PWM。
 * 原因是电机 PWM 通常是高频短周期，而舵机需要固定 20ms 周期：
 *   周期 20ms，约 50Hz
 *   高电平 1000us 左右 -> 一端
 *   高电平 1500us 左右 -> 中位
 *   高电平 2000us 左右 -> 另一端
 *
 * 不能把舵机直接挂在左右电机 PWM 上，否则舵机读到的不是标准舵机信号。
 *
 * 在 SysConfig 里新增舵机 PWM 后，再把 SERVO_PWM_ENABLE 置 1，并在工程
 * 或本文件中定义下面这些宏：
 *   SERVO_PWM_INST              舵机定时器实例
 *   SERVO_PWM_CC_INDEX          舵机 PWM 比较通道
 *   SERVO_PWM_TIMER_CLK_HZ      舵机定时器计数频率
 *   SERVO_PWM_PERIOD_COUNTS     20ms 周期对应的计数值
 *
 * 推荐配置：
 *   让舵机定时器计数频率为 1MHz，这样 1 个计数 = 1us。
 *   此时 SERVO_PWM_PERIOD_COUNTS = 20000，计算最直观。
 *
 * 当前默认 SERVO_PWM_ENABLE=0，是为了在没配舵机硬件前也能正常编译。
 * 这时 ServoControl_update() 会正常运行，但 Board_setServoPulseUs() 不会
 * 真正输出 PWM。
 */
#ifndef SERVO_PWM_ENABLE
#define SERVO_PWM_ENABLE 0
#endif

#ifndef SERVO_PWM_TIMER_CLK_HZ
#define SERVO_PWM_TIMER_CLK_HZ 1000000U
#endif

#ifndef SERVO_PWM_PERIOD_COUNTS
#define SERVO_PWM_PERIOD_COUNTS 20000U
#endif

#ifndef SERVO_PWM_CENTER_PULSE_US
#define SERVO_PWM_CENTER_PULSE_US 1500U
#endif

#ifndef SERVO_PWM_COMPARE_IS_INVERTED
#define SERVO_PWM_COMPARE_IS_INVERTED 0
#endif

void Board_init(void);
uint8_t Board_readGray5(void);
void Board_setMotorSpeed(int16_t left_speed, int16_t right_speed);
void Board_setServoPulseUs(uint16_t pulse_us);
bool Board_readStartButton(void);
bool Board_readTaskKey(uint8_t key_id);
void Board_setBuzzer(bool on);
void Board_setStatusLed(bool on);
void Board_delayMs(uint32_t ms);

#endif

