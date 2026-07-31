#include "uart.h"
#include <stddef.h>

/* 最近收到的 1 个字节。 */
volatile uint8_t g_uart_rx_data = 0U;

/* 最近收到的 8 个字节，按环形数组保存。 */
volatile uint8_t g_uart_rx_buffer[UART_RX_DEBUG_SIZE] = {0};
volatile uint8_t g_uart_rx_index = 0U;

/* 调试计数：
 * g_uart_rx_count      收到的字节总数；
 * g_uart_irq_count     进入 UART1_IRQHandler 的次数；
 * g_uart_rx_irq_count  进入 RX 接收中断的次数；
 * g_uart_last_irq_iidx 最近一次中断类型编号。
 */
volatile uint32_t g_uart_rx_count = 0U;
volatile uint32_t g_uart_irq_count = 0U;
volatile uint32_t g_uart_rx_irq_count = 0U;
volatile uint32_t g_uart_last_irq_iidx = 0U;

/* 兼容旧变量名，只做简单计数，不再表示 K230 协议解析状态。 */
volatile uint32_t g_k230_irq_count = 0U;
volatile uint32_t g_k230_rx_irq_count = 0U;
volatile uint32_t g_k230_error_irq_count = 0U;
volatile uint32_t g_k230_last_irq_iidx = 0U;

static uint8_t g_k230_frame[8] = {0};

void send_char(UART_Regs *uart, uint8_t data)
{
    while (DL_UART_isBusy(uart) == true) {
    }
    DL_UART_transmitData(uart, data);
}

void send_str(UART_Regs *uart, const uint8_t *data)
{
    if (data == NULL) {
        return;
    }

    while (*data != 0U) {
        send_char(uart, *data);
        data++;
    }
}

static void UART1_resetParser(uint8_t data)
{
    if (data == 0xAAU) {
        g_k230_frame[0] = data;
        g_k230_parse_state = K230_PARSE_WAIT_HEAD2;
    } else {
        g_k230_parse_state = K230_PARSE_WAIT_HEAD1;
    }
}

static void UART1_parseRxByte(uint8_t data)
{
    switch (g_k230_parse_state) {
        case K230_PARSE_WAIT_HEAD1:
            if (data == 0xAAU) {
                g_k230_frame[0] = data;
                g_k230_parse_state = K230_PARSE_WAIT_HEAD2;
            }
            break;

        case K230_PARSE_WAIT_HEAD2:
            if (data == 0xAAU) {
                g_k230_frame[1] = data;
                g_k230_parse_state = K230_PARSE_X_HIGH;
            } else {
                UART1_resetParser(data);
            }
            break;

        case K230_PARSE_X_HIGH:
        case K230_PARSE_X_LOW:
        case K230_PARSE_Y_HIGH:
        case K230_PARSE_Y_LOW:
            g_k230_frame[g_k230_parse_state] = data;
            g_k230_parse_state++;
            break;

        case K230_PARSE_WAIT_TAIL1:
            if (data == 0xFFU) {
                g_k230_frame[6] = data;
                g_k230_parse_state = K230_PARSE_WAIT_TAIL2;
            } else {
                g_k230_bad_frame_count++;
                UART1_resetParser(data);
            }
            break;

        case K230_PARSE_WAIT_TAIL2:
            if (data == 0xFFU) {
                g_k230_frame[7] = data;
                g_k230_center_x =
                    (uint16_t) (((uint16_t) g_k230_frame[2] << 8) |
                                (uint16_t) g_k230_frame[3]);
                g_k230_center_y =
                    (uint16_t) (((uint16_t) g_k230_frame[4] << 8) |
                                (uint16_t) g_k230_frame[5]);
                g_k230_frame0 = g_k230_frame[0];
                g_k230_frame1 = g_k230_frame[1];
                g_k230_frame2 = g_k230_frame[2];
                g_k230_frame3 = g_k230_frame[3];
                g_k230_frame4 = g_k230_frame[4];
                g_k230_frame5 = g_k230_frame[5];
                g_k230_frame6 = g_k230_frame[6];
                g_k230_frame7 = g_k230_frame[7];
                g_k230_frame_count++;
                g_k230_last_frame_ticks = g_sys_ticks;
                g_k230_position_valid = true;
                g_k230_position_fresh = true;
                g_k230_frame_event = true;
                g_k230_parse_state = K230_PARSE_WAIT_HEAD1;
            } else {
                g_k230_bad_frame_count++;
                UART1_resetParser(data);
            }
            break;

        default:
            g_k230_bad_frame_count++;
            UART1_resetParser(data);
            break;
    }
}

static void UART1_saveRxWord(uint32_t rx_word)
{
    const uint32_t error_mask =
        DL_UART_ERROR_OVERRUN | DL_UART_ERROR_BREAK |
        DL_UART_ERROR_PARITY | DL_UART_ERROR_FRAMING;
    uint8_t data = (uint8_t) (rx_word & UART_RXDATA_DATA_MASK);

    g_uart_rx_data = data;
    g_uart_rx_count++;

    g_uart_rx_buffer[g_uart_rx_index] = data;
    g_uart_rx_index++;
    if (g_uart_rx_index >= UART_RX_DEBUG_SIZE) {
        g_uart_rx_index = 0U;
    }

    if ((rx_word & error_mask) != 0U) {
        g_k230_error_irq_count++;
        g_k230_bad_frame_count++;
        g_k230_parse_state = K230_PARSE_WAIT_HEAD1;
        return;
    }

    UART1_parseRxByte(data);
}

void UART1_enableRxInterrupt(void)
{
#ifndef SIMPLE_UART_INST
#error "SIMPLE_UART_INST is not defined. Check SysConfig UART instance name."
#endif
#ifndef K230_INST_INT_IRQN
#error "UART1 IRQ number is not defined. Check SysConfig instance name."
#endif
    DL_UART_Main_enableInterrupt(
        SIMPLE_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
}

void UART1_poll(void)
{
#ifndef SIMPLE_UART_INST
#error "SIMPLE_UART_INST is not defined. Check SysConfig UART instance name."
#endif
    while (!DL_UART_Main_isRXFIFOEmpty(SIMPLE_UART_INST)) {
        UART1_saveRxWord(SIMPLE_UART_INST->RXDATA);
    }
}

void UART1_IRQHandler(void)
{
#ifndef SIMPLE_UART_INST
#error "SIMPLE_UART_INST is not defined. Check SysConfig UART instance name."
#endif
    uint32_t pending_irq;

    do {
        pending_irq =
            (uint32_t) DL_UART_Main_getPendingInterrupt(SIMPLE_UART_INST);
        if (pending_irq == DL_UART_MAIN_IIDX_NO_INTERRUPT) {
            break;
        }

        g_uart_irq_count++;
        g_uart_last_irq_iidx = pending_irq;
        g_k230_irq_count = g_uart_irq_count;
        g_k230_last_irq_iidx = pending_irq;

        if (pending_irq == DL_UART_MAIN_IIDX_RX) {
            g_uart_rx_irq_count++;
            g_k230_rx_irq_count = g_uart_rx_irq_count;
            while (!DL_UART_Main_isRXFIFOEmpty(SIMPLE_UART_INST)) {
                UART1_saveRxWord(SIMPLE_UART_INST->RXDATA);
            }
        } else if ((pending_irq == DL_UART_MAIN_IIDX_OVERRUN_ERROR) ||
                   (pending_irq == DL_UART_MAIN_IIDX_BREAK_ERROR) ||
                   (pending_irq == DL_UART_MAIN_IIDX_PARITY_ERROR) ||
                   (pending_irq == DL_UART_MAIN_IIDX_FRAMING_ERROR) ||
                   (pending_irq == DL_UART_MAIN_IIDX_NOISE_ERROR)) {
            g_k230_error_irq_count++;
            g_k230_bad_frame_count++;
            g_k230_parse_state = K230_PARSE_WAIT_HEAD1;
        }
    } while (true);
}
