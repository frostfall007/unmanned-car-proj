#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"
#include "colorful_led.h"

#define FORWARD_DUTY_PERCENT 60U    /* Forward speed: 0 to 100 percent. */
#define REVERSE_DUTY_PERCENT 50U    /* Reverse speed: 0 to 100 percent. */
#define FORWARD_TIME_MS       20000U /* Forward duration in milliseconds. */
#define REVERSE_TIME_MS       20000U /* Reverse duration in milliseconds. */
#define DIRECTION_PAUSE_MS    200U  /* Stop time before each direction change. */

static void wait_with_running_light(u16 duration_ms)
{
    while (duration_ms >= RUNNING_LIGHT_STEP_MS)
    {
        colorful_led_running_step();
        duration_ms -= RUNNING_LIGHT_STEP_MS;
    }

    if (duration_ms > 0U)
    {
        delay_ms(duration_ms);
    }
}

int main(void)
{
    Stm32_Clock_Init(9U);       /* 8 MHz crystal x 9 = 72 MHz */
    MY_NVIC_PriorityGroupConfig(2U);
    delay_init();
    uart_init(115200U);         /* Retained from the serial-port reference project. */
    JTAG_Set(SWD_ENABLE);       /* Retain SWD programming and debugging. */
    motor_init();
    colorful_led_Init();

    printf("Car control started\r\n");
    while (1)
    {
        printf("Forward\r\n");
        motor_forward(FORWARD_DUTY_PERCENT);
        wait_with_running_light(FORWARD_TIME_MS);

        motor_stop();
        wait_with_running_light(DIRECTION_PAUSE_MS);

        printf("Reverse\r\n");
        motor_reverse(REVERSE_DUTY_PERCENT);
        wait_with_running_light(REVERSE_TIME_MS);

        motor_stop();
        wait_with_running_light(DIRECTION_PAUSE_MS);
    }
}
