#include "colorful_led.h"

/*
 * Board connections, verified from the vehicle schematic:
 *   PC13 -> FR-WS2812 (six front LEDs)
 *   PC14 -> BA-WS2812 (six rear LEDs)
 * WS2812 transfers color data in GRB order.
 */
#define FRONT_LED_PIN GPIO_Pin_13
#define REAR_LED_PIN  GPIO_Pin_14

static u8 front_data[WS2812_LED_COUNT * 3U];
static u8 rear_data[WS2812_LED_COUNT * 3U];

/* These delays match the reference project when built for a 72 MHz STM32F103. */
#define WAIT_10_NOPS()  do { __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
                              __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); } while (0)
#define WAIT_400_NS()   do { WAIT_10_NOPS(); WAIT_10_NOPS(); } while (0)
#define WAIT_850_NS()   do { WAIT_10_NOPS(); WAIT_10_NOPS(); WAIT_10_NOPS(); \
                              WAIT_10_NOPS(); WAIT_10_NOPS(); __NOP(); __NOP(); \
                              __NOP(); __NOP(); __NOP(); } while (0)

static void ws2812_send_zero(u16 pin)
{
    GPIOC->BSRR = pin;
    WAIT_400_NS();
    GPIOC->BRR = pin;
    WAIT_850_NS();
}

static void ws2812_send_one(u16 pin)
{
    GPIOC->BSRR = pin;
    WAIT_850_NS();
    GPIOC->BRR = pin;
    WAIT_400_NS();
}

static void ws2812_reset(u16 pin)
{
    GPIOC->BRR = pin;
    delay_us(80U);
}

static void ws2812_refresh(const u8 *data, u16 pin)
{
    u8 byte_index;
    u8 bit_index;

    /* Interrupts must not stretch the 0.4/0.85 us WS2812 waveform. */
    __disable_irq();
    for (byte_index = 0U; byte_index < (WS2812_LED_COUNT * 3U); ++byte_index)
    {
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            if ((data[byte_index] & (0x80U >> bit_index)) != 0U)
            {
                ws2812_send_one(pin);
            }
            else
            {
                ws2812_send_zero(pin);
            }
        }
    }
    __enable_irq();
    ws2812_reset(pin);
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
    ws2812_refresh(front_data, FRONT_LED_PIN);
    ws2812_refresh(rear_data, REAR_LED_PIN);
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
    ws2812_refresh(front_data, FRONT_LED_PIN);
    ws2812_refresh(rear_data, REAR_LED_PIN);

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

    delay_ms(120U);
}
