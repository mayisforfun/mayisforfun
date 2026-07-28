#ifndef BLUETOOTH_UART_H_
#define BLUETOOTH_UART_H_

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * Bluetooth UART module, header-only version.
 * Default is disabled so the stable line-follow build is not affected before
 * UART2 is configured in SysConfig.
 *
 * To enable after SysConfig is ready:
 *   1. Add UART in SysConfig, name it BT, choose UART2, 115200, TX/RX.
 *   2. Set BT_UART_ENABLE to 1 below or define it before including this file.
 *   3. If your SysConfig name is not BT, define BT_UART_INST manually.
 *
 * Simple command protocol:
 *   G\n : go / continue
 *   S\n : stop
 *   L\n : left task hint
 *   R\n : right task hint
 *   M\n : mode switch hint
 */
#ifndef BT_UART_ENABLE
#define BT_UART_ENABLE 1
#endif

#ifndef BT_UART_LINE_SIZE
#define BT_UART_LINE_SIZE 32U
#endif

#ifndef BT_UART_INST
#ifdef BT_INST
#define BT_UART_INST BT_INST
#elif defined(BLUETOOTH_INST)
#define BT_UART_INST BLUETOOTH_INST
#elif defined(PRINT_INST)
#define BT_UART_INST PRINT_INST
#endif
#endif

typedef enum {
    BT_CMD_NONE = 0,
    BT_CMD_GO,
    BT_CMD_STOP,
    BT_CMD_LEFT,
    BT_CMD_RIGHT,
    BT_CMD_MODE,
    BT_CMD_UNKNOWN,
} BluetoothCommand;

typedef struct {
    char line[BT_UART_LINE_SIZE];
    uint8_t length;
    bool line_ready;
    uint32_t rx_count;
    uint32_t line_count;
    uint32_t overflow_count;
    BluetoothCommand last_command;
} BluetoothUART;

static inline void BluetoothUART_init(BluetoothUART *bt)
{
    bt->length = 0;
    bt->line_ready = false;
    bt->rx_count = 0;
    bt->line_count = 0;
    bt->overflow_count = 0;
    bt->last_command = BT_CMD_NONE;
    bt->line[0] = '\0';
}

static inline BluetoothCommand BluetoothUART_parseCommandChar(char ch)
{
    switch (ch) {
    case 'G':
    case 'g':
        return BT_CMD_GO;
    case 'S':
    case 's':
        return BT_CMD_STOP;
    case 'L':
    case 'l':
        return BT_CMD_LEFT;
    case 'R':
    case 'r':
        return BT_CMD_RIGHT;
    case 'M':
    case 'm':
        return BT_CMD_MODE;
    case '\0':
        return BT_CMD_NONE;
    default:
        return BT_CMD_UNKNOWN;
    }
}

static inline void BluetoothUART_pushChar(BluetoothUART *bt, char ch)
{
    bt->rx_count++;

    if (ch == '\r') {
        return;
    }

    if (ch == '\n') {
        bt->line[bt->length] = '\0';
        bt->line_ready = true;
        bt->line_count++;
        bt->last_command = BluetoothUART_parseCommandChar(bt->line[0]);
        bt->length = 0;
        return;
    }

    if (bt->length + 1U < BT_UART_LINE_SIZE) {
        bt->line[bt->length] = ch;
        bt->length++;
    } else {
        bt->overflow_count++;
        bt->length = 0;
        bt->line_ready = false;
    }
}

static inline void BluetoothUART_poll(BluetoothUART *bt)
{
#if BT_UART_ENABLE
#ifndef BT_UART_INST
#error "BT_UART_ENABLE is 1, but BT_UART_INST is not defined. Name the SysConfig UART as BT or define BT_UART_INST."
#endif
    while (!DL_UART_Main_isRXFIFOEmpty(BT_UART_INST)) {
        char ch = (char) DL_UART_Main_receiveData(BT_UART_INST);
        BluetoothUART_pushChar(bt, ch);
    }
#else
    (void) bt;
#endif
}

static inline void BluetoothUART_sendChar(char ch)
{
#if BT_UART_ENABLE
#ifndef BT_UART_INST
#error "BT_UART_ENABLE is 1, but BT_UART_INST is not defined. Name the SysConfig UART as BT or define BT_UART_INST."
#endif
    DL_UART_Main_transmitDataBlocking(BT_UART_INST, (uint8_t) ch);
#else
    (void) ch;
#endif
}

static inline void BluetoothUART_sendString(const char *str)
{
#if BT_UART_ENABLE
    while (*str != '\0') {
        BluetoothUART_sendChar(*str);
        str++;
    }
#else
    (void) str;
#endif
}

static inline void BluetoothUART_sendLine(const char *str)
{
    BluetoothUART_sendString(str);
    BluetoothUART_sendString("\r\n");
}

static inline bool BluetoothUART_getLine(BluetoothUART *bt, char *out, uint8_t out_size)
{
    uint8_t i = 0;

    if (!bt->line_ready) {
        return false;
    }

    if (out_size > 0U) {
        while ((i + 1U < out_size) && (bt->line[i] != '\0')) {
            out[i] = bt->line[i];
            i++;
        }
        out[i] = '\0';
    }

    bt->line_ready = false;
    return true;
}

static inline BluetoothCommand BluetoothUART_getCommand(BluetoothUART *bt)
{
    BluetoothCommand cmd = bt->last_command;
    bt->last_command = BT_CMD_NONE;
    return cmd;
}

#endif

