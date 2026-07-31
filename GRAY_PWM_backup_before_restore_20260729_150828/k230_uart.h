#ifndef K230_UART_H_
#define K230_UART_H_

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * K230 视觉坐标通信模块
 * ============================================================
 *
 * 这个文件只做 K230 -> MSPM0 的串口通信和协议解析，不做舵机控制，
 * 不做 PID，也不决定小球应该往哪边走。
 *
 * K230 端当前发送代码：
 *
 *   tx_position = [0xAA, 0xAA, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF]
 *   def send_position(center_x, center_y):
 *       tx_position[2] = (center_x // 256) % 256
 *       tx_position[3] = center_x % 256
 *       tx_position[4] = (center_y // 256) % 256
 *       tx_position[5] = center_y % 256
 *       uart1.write(bytes(tx_position))
 *
 * 上面这段 Python 发出来的是“二进制字节”，不是字符串。
 * 例如 tx_position 里写 0xAA，串口线上真正发出的就是一个字节 0xAA，
 * 不是字符 'A'、'A' 两个字节。
 *
 * 接线：
 *   K230 IO3 / UART1_TXD  ->  MSPM0 PA9 / UART1_RX
 *   K230 GND              ->  MSPM0 GND
 *
 * 串口参数：
 *   115200 波特率，8 数据位，无校验，1 停止位，也就是常说的 115200 8N1。
 *
 * 协议帧固定 8 字节：
 *   [0] 0xAA        帧头 1
 *   [1] 0xAA        帧头 2
 *   [2] center_x_H  X 坐标高 8 位
 *   [3] center_x_L  X 坐标低 8 位
 *   [4] center_y_H  Y 坐标高 8 位
 *   [5] center_y_L  Y 坐标低 8 位
 *   [6] 0xFF        帧尾 1
 *   [7] 0xFF        帧尾 2
 *
 * 坐标合成方式：
 *   center_x = center_x_H * 256 + center_x_L
 *   center_y = center_y_H * 256 + center_y_L
 *
 * 固定测试帧：
 *   K230 发送：AA AA 01 02 03 04 FF FF
 *   MSPM0 解析：center_x = 0x0102 = 258
 *              center_y = 0x0304 = 772
 *
 * 调试判断：
 *   rx_count 增加但 frame_count 不增加：
 *     说明 PA9 收到了字节，但字节内容不符合本协议。
 *
 *   rx_count 不增加：
 *     说明 MSPM0 没收到 K230 的电平，优先查 TX/RX、GND、串口号和电平。
 *
 *   frame_count 增加：
 *     说明完整协议帧已经解析成功，可以看 center_x/center_y。
 *
 * 小球丢失时：
 *   这版协议没有 valid 标志。建议 K230 没识别到小球时停止发送坐标。
 *   MSPM0 超过 K230_UART_TIMEOUT_TICKS 没收到新帧后，fresh 会变 false。
 */

#ifndef K230_UART_ENABLE
#define K230_UART_ENABLE 1
#endif

/* 主循环 10ms 进一次控制周期，20 个 tick 大约是 200ms。
 * 超过这个时间没有新坐标，就认为视觉数据已经不新鲜。
 */
#ifndef K230_UART_TIMEOUT_TICKS
#define K230_UART_TIMEOUT_TICKS 20U
#endif

/* 保存最近 8 个原始接收字节，方便在 CCS 里看实际收到什么。 */
#ifndef K230_UART_RAW_DEBUG_SIZE
#define K230_UART_RAW_DEBUG_SIZE 8U
#endif

#ifndef K230_UART_INST
#ifdef K230_INST
#define K230_UART_INST K230_INST
#endif
#endif

typedef enum {
    K230_PARSE_WAIT_HEAD1 = 0,  /* 等待第 1 个 0xAA。 */
    K230_PARSE_WAIT_HEAD2,      /* 已收到 1 个 0xAA，等待第 2 个 0xAA。 */
    K230_PARSE_READ_XH,         /* 读取 X 坐标高字节。 */
    K230_PARSE_READ_XL,         /* 读取 X 坐标低字节。 */
    K230_PARSE_READ_YH,         /* 读取 Y 坐标高字节。 */
    K230_PARSE_READ_YL,         /* 读取 Y 坐标低字节。 */
    K230_PARSE_WAIT_TAIL1,      /* 等待第 1 个 0xFF。 */
    K230_PARSE_WAIT_TAIL2,      /* 等待第 2 个 0xFF。 */
} K230ParseState;

typedef struct {
    K230ParseState parse_state;

    /* 原始字节调试缓存。
     * last_byte 是最近收到的 1 个字节。
     * raw_bytes 是循环数组，保存最近 K230_UART_RAW_DEBUG_SIZE 个字节。
     */
    uint8_t last_byte;
    uint8_t raw_bytes[K230_UART_RAW_DEBUG_SIZE];
    uint8_t raw_index;

    /* 最近一次成功解析出的完整帧。
     * 这组数据只在 AA AA ... FF FF 全部校验通过后更新，
     * 所以比 raw_bytes 更适合判断协议是否真正成功。
     */
    uint8_t last_frame[8];

    /* 协议同步窗口。
     * 每收到一个新字节，就把它放到 sync_window[7]，
     * 旧字节整体左移。只要窗口里出现 AA AA xx xx yy yy FF FF，
     * 就说明已经对齐到一帧完整坐标。
     */
    uint8_t sync_window[8];
    uint8_t sync_count;

    /* 临时保存正在接收的一帧坐标字节。
     * 只有帧头、坐标、帧尾全部正确时，才会合成 center_x/center_y。
     */
    uint8_t x_high;
    uint8_t x_low;
    uint8_t y_high;
    uint8_t y_low;

    /* 最近一次成功解析出来的小球中心坐标。 */
    uint16_t center_x;
    uint16_t center_y;

    /* 最近一次成功收到完整帧时的系统 tick，用来判断坐标是否超时。 */
    uint32_t last_frame_tick;

    bool position_valid; /* 上电后是否至少收到过一帧正确坐标。 */
    bool frame_ready;    /* 是否刚刚收到一帧新坐标，主循环读取后清零。 */

    uint32_t rx_count;        /* 累计收到的原始字节数量。 */
    uint32_t frame_count;     /* 累计成功解析的完整帧数量。 */
    uint32_t bad_frame_count; /* 帧头、帧尾错误或 UART 错误次数。 */
} K230UART;

static inline void K230UART_init(volatile K230UART *k230)
{
    k230->parse_state = K230_PARSE_WAIT_HEAD1;

    k230->last_byte = 0U;
    k230->raw_index = 0U;
    for (uint8_t i = 0U; i < K230_UART_RAW_DEBUG_SIZE; i++) {
        k230->raw_bytes[i] = 0U;
    }
    for (uint8_t i = 0U; i < 8U; i++) {
        k230->last_frame[i] = 0U;
        k230->sync_window[i] = 0U;
    }
    k230->sync_count = 0U;

    k230->x_high = 0U;
    k230->x_low = 0U;
    k230->y_high = 0U;
    k230->y_low = 0U;

    k230->center_x = 0U;
    k230->center_y = 0U;
    k230->last_frame_tick = 0U;

    k230->position_valid = false;
    k230->frame_ready = false;

    k230->rx_count = 0U;
    k230->frame_count = 0U;
    k230->bad_frame_count = 0U;
}

static inline void K230UART_resetParser(volatile K230UART *k230)
{
    k230->parse_state = K230_PARSE_WAIT_HEAD1;
}

static inline void K230UART_acceptFrame(volatile K230UART *k230,
                                        uint32_t now_tick)
{
    k230->center_x = (uint16_t) ((((uint16_t) k230->x_high) << 8) |
                                 ((uint16_t) k230->x_low));
    k230->center_y = (uint16_t) ((((uint16_t) k230->y_high) << 8) |
                                 ((uint16_t) k230->y_low));
    k230->last_frame[0] = 0xAAU;
    k230->last_frame[1] = 0xAAU;
    k230->last_frame[2] = k230->x_high;
    k230->last_frame[3] = k230->x_low;
    k230->last_frame[4] = k230->y_high;
    k230->last_frame[5] = k230->y_low;
    k230->last_frame[6] = 0xFFU;
    k230->last_frame[7] = 0xFFU;
    k230->last_frame_tick = now_tick;
    k230->position_valid = true;
    k230->frame_ready = true;
    k230->frame_count++;
}

static inline void K230UART_acceptWindowFrame(volatile K230UART *k230,
                                              uint32_t now_tick)
{
    k230->x_high = k230->sync_window[2];
    k230->x_low  = k230->sync_window[3];
    k230->y_high = k230->sync_window[4];
    k230->y_low  = k230->sync_window[5];
    K230UART_acceptFrame(k230, now_tick);
}

static inline void K230UART_rememberRawByte(volatile K230UART *k230,
                                            uint8_t byte)
{
    k230->last_byte = byte;
    k230->raw_bytes[k230->raw_index] = byte;
    k230->raw_index++;
    if (k230->raw_index >= K230_UART_RAW_DEBUG_SIZE) {
        k230->raw_index = 0U;
    }
}

static inline void K230UART_pushSyncWindow(volatile K230UART *k230,
                                           uint8_t byte)
{
    for (uint8_t i = 0U; i < 7U; i++) {
        k230->sync_window[i] = k230->sync_window[i + 1U];
    }
    k230->sync_window[7] = byte;

    if (k230->sync_count < 8U) {
        k230->sync_count++;
    }
}

static inline bool K230UART_syncWindowIsFrame(volatile K230UART *k230)
{
    if (k230->sync_count < 8U) {
        return false;
    }

    return ((k230->sync_window[0] == 0xAAU) &&
            (k230->sync_window[1] == 0xAAU) &&
            (k230->sync_window[6] == 0xFFU) &&
            (k230->sync_window[7] == 0xFFU));
}

/* 输入 UART 收到的 1 个原始字节。
 * 解析器使用最近 8 字节滑动窗口自动寻找：
 *   AA AA xH xL yH yL FF FF
 * 这种做法不怕从半帧开始接收，也不怕中间短暂错位。
 */
static inline void K230UART_pushByte(volatile K230UART *k230,
                                     uint8_t byte,
                                     uint32_t now_tick)
{
    k230->rx_count++;
    K230UART_rememberRawByte(k230, byte);
    K230UART_pushSyncWindow(k230, byte);

    if (K230UART_syncWindowIsFrame(k230)) {
        K230UART_acceptWindowFrame(k230, now_tick);
        k230->parse_state = K230_PARSE_WAIT_HEAD1;
    } else if (k230->sync_count >= 8U) {
        k230->bad_frame_count++;
    }
}

/* 主循环兜底轮询。
 * 正常情况下 UART1_IRQHandler 已经会及时收字节；
 * 这里保留轮询，是为了防止中断初始化异常时完全没有调试入口。
 */
static inline void K230UART_poll(volatile K230UART *k230, uint32_t now_tick)
{
#if K230_UART_ENABLE
#ifndef K230_UART_INST
#error "K230_UART_ENABLE is 1, but K230_UART_INST is not defined. Name the SysConfig UART as K230 or define K230_UART_INST."
#endif
    while (!DL_UART_Main_isRXFIFOEmpty(K230_UART_INST)) {
        uint8_t byte = DL_UART_Main_receiveData(K230_UART_INST);
        K230UART_pushByte(k230, byte, now_tick);
    }
#else
    (void) k230;
    (void) now_tick;
#endif
}

static inline bool K230UART_isFresh(const volatile K230UART *k230,
                                    uint32_t now_tick)
{
    if (!k230->position_valid) {
        return false;
    }

    return (now_tick - k230->last_frame_tick) <= K230_UART_TIMEOUT_TICKS;
}

static inline bool K230UART_hasNewFrame(const volatile K230UART *k230)
{
    return k230->frame_ready;
}

static inline void K230UART_clearNewFrame(volatile K230UART *k230)
{
    k230->frame_ready = false;
}

#endif
