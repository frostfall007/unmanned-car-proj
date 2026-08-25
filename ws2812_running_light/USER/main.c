#include "stm32f10x.h"
#include "sys.h"
#include "colorful_led.h"

int main(void)
{
    Stm32_Clock_Init(9U);       /* 8 MHz crystal x 9 = 72 MHz */
    MY_NVIC_PriorityGroupConfig(2U);
    delay_init();
    uart_init(115200U);         /* Retained from the serial-port reference project. */
    JTAG_Set(SWD_ENABLE);       /* Release JTAG pins while retaining SWD programming. */

    colorful_led_Init();
    printf("WS2812 running light started (PC13/PC14, 6+6 LEDs)\r\n");

    while (1)
    {
        colorful_led_running_step();
    }
}
