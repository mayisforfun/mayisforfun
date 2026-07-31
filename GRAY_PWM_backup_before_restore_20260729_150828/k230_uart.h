#ifndef K230_UART_H_
#define K230_UART_H_

/*
 * 兼容入口。
 *
 * 现在 K230 串口通信和协议解析已经整理到 uart.c / uart.h。
 * 保留这个文件是为了避免旧代码里 include "k230_uart.h" 时直接报错。
 */
#include "uart.h"

#endif
