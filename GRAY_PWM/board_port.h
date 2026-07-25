#ifndef BOARD_PORT_H_
#define BOARD_PORT_H_

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * Default pin mapping from D:\qq\IO口对应.xlsx:
 *
 * Gray:
 *   OUT1/L2 -> PA26
 *   OUT2/L1 -> PA27
 *   OUT3/C  -> PA24
 *   OUT4/R1 -> PA25
 *   OUT5/R2 -> PB24
 *
 * TB6612:
 *   AIN1 -> PB23
 *   AIN2 -> PB26
 *   BIN1 -> PB08
 *   BIN2 -> PB09
 *   PWMA -> PA12 / TIMG0_C0
 *   PWMB -> PA13 / TIMG0_C1
 *   BEEP -> PB25
 *
 * SysConfig still needs to initialize these pins as GPIO input/output/PWM.
 */

#ifndef GRAY_BLACK_IS_LOW
#define GRAY_BLACK_IS_LOW 1
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

void Board_init(void);
uint8_t Board_readGray5(void);
void Board_setMotorSpeed(int16_t left_speed, int16_t right_speed);
void Board_setBuzzer(bool on);
void Board_delayMs(uint32_t ms);

#endif
