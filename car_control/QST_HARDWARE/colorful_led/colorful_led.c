#include "colorful_led.h"

/*
 * Board connections, verified from the vehicle schematic:
 *   PC13 -> FR-WS2812 (six front LEDs)
 *   PC14 -> BA-WS2812 (six rear LEDs)
 * WS2812 transfers color data in GRB order.
 */
#define FRONT_LED_PIN GPIO_Pin_13
#define REAR_LED_PIN  GPIO_Pin_14
#define FRONT_DATA    PCout(13)
#define REAR_DATA     PCout(14)

static u8 front_data[WS2812_LED_COUNT * 3U];
static u8 rear_data[WS2812_LED_COUNT * 3U];

/* These macros and the bit-band GPIO writes are retained from the reference
 * project.  Their instruction count is the WS2812 timing source at 72 MHz. */
#define WAIT_10_NOPS  { __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
                        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); }
#define WAIT_250_NS   { __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
                        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); }
#define WAIT_400_NS   { WAIT_250_NS; WAIT_10_NOPS; }
#define WAIT_850_NS   { WAIT_250_NS; WAIT_10_NOPS; WAIT_10_NOPS; \
                        WAIT_10_NOPS; WAIT_10_NOPS; __NOP(); __NOP(); \
                        __NOP(); __NOP(); __NOP(); }

static void front_send_zero(void)
{
    FRONT_DATA = 1U;
    WAIT_400_NS;
    FRONT_DATA = 0U;
    WAIT_850_NS;
}

static void front_send_one(void)
{
    FRONT_DATA = 1U;
    WAIT_850_NS;
    FRONT_DATA = 0U;
    WAIT_400_NS;
}

static void rear_send_zero(void)
{
    REAR_DATA = 1U;
    WAIT_400_NS;
    REAR_DATA = 0U;
    WAIT_850_NS;
}

static void rear_send_one(void)
{
    REAR_DATA = 1U;
    WAIT_850_NS;
    REAR_DATA = 0U;
    WAIT_400_NS;
}

static void front_refresh(void)
{
    u8 index;

    for (index = 0U; index < (WS2812_LED_COUNT * 3U); ++index)
    {
        if ((front_data[index] & 0x80U) == 0U) front_send_zero(); else front_send_one();
        if ((front_data[index] & 0x40U) == 0U) front_send_zero(); else front_send_one();
        if ((front_data[index] & 0x20U) == 0U) front_send_zero(); else front_send_one();
        if ((front_data[index] & 0x10U) == 0U) front_send_zero(); else front_send_one();
        if ((front_data[index] & 0x08U) == 0U) front_send_zero(); else front_send_one();
        if ((front_data[index] & 0x04U) == 0U) front_send_zero(); else front_send_one();
        if ((front_data[index] & 0x02U) == 0U) front_send_zero(); else front_send_one();
        if ((front_data[index] & 0x01U) == 0U) front_send_zero(); else front_send_one();
    }
    FRONT_DATA = 0U;
    delay_us(80U);
}

static void rear_refresh(void)
{
    u8 index;

    for (index = 0U; index < (WS2812_LED_COUNT * 3U); ++index)
    {
        if ((rear_data[index] & 0x80U) == 0U) rear_send_zero(); else rear_send_one();
        if ((rear_data[index] & 0x40U) == 0U) rear_send_zero(); else rear_send_one();
        if ((rear_data[index] & 0x20U) == 0U) rear_send_zero(); else rear_send_one();
        if ((rear_data[index] & 0x10U) == 0U) rear_send_zero(); else rear_send_one();
        if ((rear_data[index] & 0x08U) == 0U) rear_send_zero(); else rear_send_one();
        if ((rear_data[index] & 0x04U) == 0U) rear_send_zero(); else rear_send_one();
        if ((rear_data[index] & 0x02U) == 0U) rear_send_zero(); else rear_send_one();
        if ((rear_data[index] & 0x01U) == 0U) rear_send_zero(); else rear_send_one();
    }
    REAR_DATA = 0U;
    delay_us(80U);
}

static void ws2812_set_pixel(u8 *data, u8 led_index, u8 red, u8 green, u8 blue)
{
    u8 offset = led_index * 3U;

    data[offset] = green;
    data[offset + 1U] = red;
    data[offset + 2U] = blue;
}

static void ws2812_clear(u8 *data)
{
    u8 index;

    for (index = 0U; index < (WS2812_LED_COUNT * 3U); ++index)
    {
        data[index] = 0U;
    }
}

void colorful_led_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin = FRONT_LED_PIN | REAR_LED_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    colorful_led_off();
}

void colorful_led_off(void)
{
    ws2812_clear(front_data);
    ws2812_clear(rear_data);
    front_refresh();
    rear_refresh();
}

void colorful_led_running_step(void)
{
    static u8 led_index = 0U;
    static u8 moving_forward = 1U;

    ws2812_clear(front_data);
    ws2812_clear(rear_data);

    /* White point moves over the front and rear LED chains together. */
    ws2812_set_pixel(front_data, led_index, 48U, 48U, 48U);
    ws2812_set_pixel(rear_data, led_index, 48U, 48U, 48U);
    front_refresh();
    rear_refresh();

    if (moving_forward != 0U)
    {
        if (led_index == (WS2812_LED_COUNT - 1U))
        {
            moving_forward = 0U;
        }
        else
        {
            ++led_index;
        }
    }
    else if (led_index == 0U)
    {
        moving_forward = 1U;
    }
    else
    {
        --led_index;
    }

    delay_ms(RUNNING_LIGHT_STEP_MS);
}
