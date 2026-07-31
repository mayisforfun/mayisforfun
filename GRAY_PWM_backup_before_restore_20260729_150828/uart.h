#ifndef UART_H_
#define UART_H_

#include "ti_msp_dl_config.h"

//发送字符
void send_char(UART_Regs *uart, uint8_t data);

//发送字符串
void send_str(UART_Regs *uart, uint8_t * data);

#endif
