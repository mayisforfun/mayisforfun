#ifndef OLED_DEBUG_H_
#define OLED_DEBUG_H_

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define OLED_DEBUG_ENABLE 1

#ifndef OLED_SDA_PORT
#define OLED_SDA_PORT GPIOA
#endif
#ifndef OLED_SDA_PIN
#define OLED_SDA_PIN DL_GPIO_PIN_28
#endif
#ifndef OLED_SDA_IOMUX
#define OLED_SDA_IOMUX IOMUX_PINCM3
#endif

#ifndef OLED_SCL_PORT
#define OLED_SCL_PORT GPIOA
#endif
#ifndef OLED_SCL_PIN
#define OLED_SCL_PIN DL_GPIO_PIN_31
#endif
#ifndef OLED_SCL_IOMUX
#define OLED_SCL_IOMUX IOMUX_PINCM6
#endif

#define OLED_ADDR_7BIT 0x3CU

static const uint8_t oled_font5x7_digits[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
};

static const uint8_t oled_font5x7_letters[26][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43},
};

static void oled_i2c_delay(void)
{
    delay_cycles(CPUCLK_FREQ / 2000000U);
}

static void oled_sda_low(void)
{
    DL_GPIO_clearPins(OLED_SDA_PORT, OLED_SDA_PIN);
    DL_GPIO_enableOutput(OLED_SDA_PORT, OLED_SDA_PIN);
}

static void oled_sda_release(void)
{
    DL_GPIO_disableOutput(OLED_SDA_PORT, OLED_SDA_PIN);
}

static void oled_scl_low(void)
{
    DL_GPIO_clearPins(OLED_SCL_PORT, OLED_SCL_PIN);
    DL_GPIO_enableOutput(OLED_SCL_PORT, OLED_SCL_PIN);
}

static void oled_scl_release(void)
{
    DL_GPIO_disableOutput(OLED_SCL_PORT, OLED_SCL_PIN);
}

static void oled_i2c_start(void)
{
    oled_sda_release();
    oled_scl_release();
    oled_i2c_delay();
    oled_sda_low();
    oled_i2c_delay();
    oled_scl_low();
}

static void oled_i2c_stop(void)
{
    oled_sda_low();
    oled_i2c_delay();
    oled_scl_release();
    oled_i2c_delay();
    oled_sda_release();
    oled_i2c_delay();
}

static void oled_i2c_write_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8U; i++) {
        if ((data & 0x80U) != 0U) {
            oled_sda_release();
        } else {
            oled_sda_low();
        }
        oled_i2c_delay();
        oled_scl_release();
        oled_i2c_delay();
        oled_scl_low();
        data <<= 1;
    }

    oled_sda_release();
    oled_i2c_delay();
    oled_scl_release();
    oled_i2c_delay();
    oled_scl_low();
}

static void OLED_Debug_write(uint8_t control, uint8_t data)
{
    oled_i2c_start();
    oled_i2c_write_byte((uint8_t) (OLED_ADDR_7BIT << 1));
    oled_i2c_write_byte(control);
    oled_i2c_write_byte(data);
    oled_i2c_stop();
}

static void OLED_Debug_cmd(uint8_t cmd)
{
    OLED_Debug_write(0x00U, cmd);
}

static void OLED_Debug_data(uint8_t data)
{
    OLED_Debug_write(0x40U, data);
}

static void OLED_Debug_setPos(uint8_t x, uint8_t page)
{
    OLED_Debug_cmd((uint8_t) (0xB0U + page));
    OLED_Debug_cmd((uint8_t) (x & 0x0FU));
    OLED_Debug_cmd((uint8_t) (0x10U | (x >> 4)));
}

static const uint8_t *OLED_Debug_glyph(char ch)
{
    static const uint8_t blank[5] = {0,0,0,0,0};
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t dot[5] = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t dash[5] = {0x08,0x08,0x08,0x08,0x08};

    if ((ch >= '0') && (ch <= '9')) {
        return oled_font5x7_digits[ch - '0'];
    }
    if ((ch >= 'a') && (ch <= 'z')) {
        ch = (char) (ch - 'a' + 'A');
    }
    if ((ch >= 'A') && (ch <= 'Z')) {
        return oled_font5x7_letters[ch - 'A'];
    }
    if (ch == ':') {
        return colon;
    }
    if (ch == '.') {
        return dot;
    }
    if (ch == '-') {
        return dash;
    }
    return blank;
}

static void OLED_Debug_clear(void)
{
    for (uint8_t page = 0; page < 8U; page++) {
        OLED_Debug_setPos(0U, page);
        for (uint8_t col = 0; col < 128U; col++) {
            OLED_Debug_data(0x00U);
        }
    }
}

static void OLED_Debug_showString(uint8_t x, uint8_t page, const char *text)
{
    while ((*text != '\0') && (x < 123U) && (page < 8U)) {
        const uint8_t *glyph = OLED_Debug_glyph(*text++);

        OLED_Debug_setPos(x, page);
        for (uint8_t i = 0; i < 5U; i++) {
            OLED_Debug_data(glyph[i]);
        }
        OLED_Debug_data(0x00U);
        x = (uint8_t) (x + 6U);
    }
}

static void OLED_Debug_formatTime(char *out, uint32_t ticks)
{
    uint32_t tenths = ticks / 10U;
    uint32_t seconds = tenths / 10U;
    uint32_t frac = tenths % 10U;

    out[0] = 'T';
    out[1] = 'I';
    out[2] = 'M';
    out[3] = 'E';
    out[4] = ' ';
    out[5] = (char) ('0' + ((seconds / 100U) % 10U));
    out[6] = (char) ('0' + ((seconds / 10U) % 10U));
    out[7] = (char) ('0' + (seconds % 10U));
    out[8] = '.';
    out[9] = (char) ('0' + frac);
    out[10] = 'S';
    out[11] = '\0';
}

static void OLED_Debug_init(void)
{
    DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SCL_IOMUX);
    DL_GPIO_disableOutput(OLED_SDA_PORT, OLED_SDA_PIN);
    DL_GPIO_disableOutput(OLED_SCL_PORT, OLED_SCL_PIN);
    Board_delayMs(50U);

    OLED_Debug_cmd(0xAE);
    OLED_Debug_cmd(0x20);
    OLED_Debug_cmd(0x02);
    OLED_Debug_cmd(0xB0);
    OLED_Debug_cmd(0xC8);
    OLED_Debug_cmd(0x00);
    OLED_Debug_cmd(0x10);
    OLED_Debug_cmd(0x40);
    OLED_Debug_cmd(0x81);
    OLED_Debug_cmd(0x7F);
    OLED_Debug_cmd(0xA1);
    OLED_Debug_cmd(0xA6);
    OLED_Debug_cmd(0xA8);
    OLED_Debug_cmd(0x3F);
    OLED_Debug_cmd(0xA4);
    OLED_Debug_cmd(0xD3);
    OLED_Debug_cmd(0x00);
    OLED_Debug_cmd(0xD5);
    OLED_Debug_cmd(0x80);
    OLED_Debug_cmd(0xD9);
    OLED_Debug_cmd(0xF1);
    OLED_Debug_cmd(0xDA);
    OLED_Debug_cmd(0x12);
    OLED_Debug_cmd(0xDB);
    OLED_Debug_cmd(0x40);
    OLED_Debug_cmd(0x8D);
    OLED_Debug_cmd(0x14);
    OLED_Debug_cmd(0xAF);
    OLED_Debug_clear();
}

static void OLED_Debug_update(uint32_t elapsed_ticks, bool running, uint8_t gray_bits)
{
    char line[12];
    char gray[12] = {
        'G','R','A','Y',' ',
        (gray_bits & (1U << 0)) ? '1' : '0',
        (gray_bits & (1U << 1)) ? '1' : '0',
        (gray_bits & (1U << 2)) ? '1' : '0',
        (gray_bits & (1U << 3)) ? '1' : '0',
        (gray_bits & (1U << 4)) ? '1' : '0',
        '\0','\0'
    };

    OLED_Debug_formatTime(line, elapsed_ticks);
    OLED_Debug_showString(0U, 0U, line);
    OLED_Debug_showString(0U, 2U, running ? "RUN " : "STOP");
    OLED_Debug_showString(0U, 4U, gray);
}

#endif
