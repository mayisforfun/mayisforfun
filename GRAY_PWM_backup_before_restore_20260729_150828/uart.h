#ifndef UART_H_
#define UART_H_

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * 最简单的 UART1 收发模块
 * ============================================================
 *
 * 这里只做串口最基础的事情：
 *   1. 发送 1 个字节；
 *   2. 发送 1 个字符串；
 *   3. UART1 中断收到 1 个字节后保存下来；
 *   4. 记录最近收到的 8 个原始字节，方便 CCS 表达式窗口观察。
 *
 * 这里不做 K230 坐标协议解析，不找 AA AA 帧头，不判断 FF FF 帧尾，
 * 不计算 center_x / center_y。
 */

#ifndef UART_RX_DEBUG_SIZE
#define UART_RX_DEBUG_SIZE 8U
#endif

/* 保留旧调试数组长度宏，避免 main.c 里原来的观察变量大改名。 */
#ifndef K230_UART_RAW_DEBUG_SIZE
#define K230_UART_RAW_DEBUG_SIZE UART_RX_DEBUG_SIZE
#endif

/* 保留旧 parse_state 初值，当前简化版不再使用状态机。 */
#ifndef K230_PARSE_WAIT_HEAD1
#define K230_PARSE_WAIT_HEAD1 0U
#endif

#define K230_PARSE_WAIT_HEAD2 1U
#define K230_PARSE_X_HIGH     2U
#define K230_PARSE_X_LOW      3U
#define K230_PARSE_Y_HIGH     4U
#define K230_PARSE_Y_LOW      5U
#define K230_PARSE_WAIT_TAIL1 6U
#define K230_PARSE_WAIT_TAIL2 7U

#ifndef SIMPLE_UART_INST
#ifdef K230_INST
#define SIMPLE_UART_INST K230_INST
#endif
#endif

/* 最近收到的 1 个字节。 */
extern volatile uint8_t g_uart_rx_data;

/* 最近收到的 8 个字节，环形保存。 */
extern volatile uint8_t g_uart_rx_buffer[UART_RX_DEBUG_SIZE];
extern volatile uint8_t g_uart_rx_index;

/* 串口调试计数。 */
extern volatile uint32_t g_uart_rx_count;
extern volatile uint32_t g_uart_irq_count;
extern volatile uint32_t g_uart_rx_irq_count;
extern volatile uint32_t g_uart_last_irq_iidx;

/* 兼容之前 CCS 表达式窗口里已经添加过的变量名。 */
extern volatile uint32_t g_k230_irq_count;
extern volatile uint32_t g_k230_rx_irq_count;
extern volatile uint32_t g_k230_error_irq_count;
extern volatile uint32_t g_k230_last_irq_iidx;

/* K230 frame: AA AA XH XL YH YL FF FF. */
extern volatile bool g_k230_position_valid;
extern volatile bool g_k230_position_fresh;
extern volatile bool g_k230_frame_event;
extern volatile uint16_t g_k230_center_x;
extern volatile uint16_t g_k230_center_y;
extern volatile uint32_t g_k230_frame_count;
extern volatile uint32_t g_k230_bad_frame_count;
extern volatile uint32_t g_k230_last_frame_ticks;
extern volatile uint8_t g_k230_parse_state;
extern volatile uint8_t g_k230_frame0;
extern volatile uint8_t g_k230_frame1;
extern volatile uint8_t g_k230_frame2;
extern volatile uint8_t g_k230_frame3;
extern volatile uint8_t g_k230_frame4;
extern volatile uint8_t g_k230_frame5;
extern volatile uint8_t g_k230_frame6;
extern volatile uint8_t g_k230_frame7;
extern volatile uint32_t g_sys_ticks;

void send_char(UART_Regs *uart, uint8_t data);
void send_str(UART_Regs *uart, const uint8_t *data);
void UART1_enableRxInterrupt(void);
void UART1_poll(void);

#endif
