#include "encoder.h"
#include "ti_msp_dl_config.h"

/*
 * Left encoder:  A = PA00 (interrupt), B = PA01 (read-only)
 * Right encoder: A = PB04 (interrupt), B = PB05 (read-only)
 *
 * Counting method: 1x quadrature — interrupt on A-phase rising edge,
 *                  read B-phase level to determine direction.
 *                  B high = forward (+1), B low = backward (-1).
 */

#define ENC_LEFT_A_PORT  GPIOA
#define ENC_LEFT_A_PIN   DL_GPIO_PIN_0
#define ENC_LEFT_A_IOMUX IOMUX_PINCM1
#define ENC_LEFT_B_PORT  GPIOA
#define ENC_LEFT_B_PIN   DL_GPIO_PIN_1
#define ENC_LEFT_B_IOMUX IOMUX_PINCM2

#define ENC_RIGHT_A_PORT  GPIOB
#define ENC_RIGHT_A_PIN   DL_GPIO_PIN_4
#define ENC_RIGHT_A_IOMUX IOMUX_PINCM17
#define ENC_RIGHT_B_PORT  GPIOB
#define ENC_RIGHT_B_PIN   DL_GPIO_PIN_5
#define ENC_RIGHT_B_IOMUX IOMUX_PINCM18

static volatile int32_t g_enc_left_ticks  = 0;
static volatile int32_t g_enc_right_ticks = 0;
static int32_t          g_enc_left_last   = 0;
static int32_t          g_enc_right_last  = 0;

void Encoder_init(void)
{
    /* Configure 4 encoder pins as digital inputs with pull-up */
    DL_GPIO_initDigitalInputFeatures(ENC_LEFT_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENC_LEFT_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENC_RIGHT_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENC_RIGHT_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    /* Rising-edge interrupt on A-phase pins only */
    DL_GPIO_setLowerPinsPolarity(ENC_LEFT_A_PORT, DL_GPIO_PIN_0_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(ENC_LEFT_A_PORT, ENC_LEFT_A_PIN);
    DL_GPIO_enableInterrupt(ENC_LEFT_A_PORT, ENC_LEFT_A_PIN);

    DL_GPIO_setLowerPinsPolarity(ENC_RIGHT_A_PORT, DL_GPIO_PIN_4_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(ENC_RIGHT_A_PORT, ENC_RIGHT_A_PIN);
    DL_GPIO_enableInterrupt(ENC_RIGHT_A_PORT, ENC_RIGHT_A_PIN);

    /* GPIOA/GPIOB interrupts are routed to GROUP1 on MSPM0G3507. */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    /* Sync snapshot counters */
    g_enc_left_last  = 0;
    g_enc_right_last = 0;
}

int32_t Encoder_getLeftTicks(void)
{
    int32_t current = (int32_t)g_enc_left_ticks;
    int32_t delta   = current - g_enc_left_last;
    g_enc_left_last = current;
    return delta;
}

int32_t Encoder_getRightTicks(void)
{
    int32_t current  = (int32_t)g_enc_right_ticks;
    int32_t delta    = current - g_enc_right_last;
    g_enc_right_last = current;
    return delta;
}

void Encoder_clearDeltas(void)
{
    g_enc_left_last  = (int32_t)g_enc_left_ticks;
    g_enc_right_last = (int32_t)g_enc_right_ticks;
}

/* ------------------------------------------------------------------ */
/*  GPIO Group Interrupt Handlers (override weak Default_Handler)      */
/* ------------------------------------------------------------------ */

void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIOA, ENC_LEFT_A_PIN);

    if (pending != 0U) {
        /* B-phase high = forward, low = backward */
        if ((DL_GPIO_readPins(ENC_LEFT_B_PORT, ENC_LEFT_B_PIN) &
             ENC_LEFT_B_PIN) != 0U) {
            g_enc_left_ticks++;
        } else {
            g_enc_left_ticks--;
        }
        DL_GPIO_clearInterruptStatus(GPIOA, ENC_LEFT_A_PIN);
    }

    pending = DL_GPIO_getEnabledInterruptStatus(GPIOB, ENC_RIGHT_A_PIN);

    if (pending != 0U) {
        if ((DL_GPIO_readPins(ENC_RIGHT_B_PORT, ENC_RIGHT_B_PIN) &
             ENC_RIGHT_B_PIN) != 0U) {
            g_enc_right_ticks++;
        } else {
            g_enc_right_ticks--;
        }
        DL_GPIO_clearInterruptStatus(GPIOB, ENC_RIGHT_A_PIN);
    }
}
