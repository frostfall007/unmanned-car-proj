#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"

#define FORWARD_DUTY_PERCENT 60U    /* Forward speed: 0 to 100 percent. */
#define REVERSE_DUTY_PERCENT 50U    /* Reverse speed: 0 to 100 percent. */
#define FORWARD_TIME_MS       20000U /* Forward duration in milliseconds. */
#define REVERSE_TIME_MS       20000U /* Reverse duration in milliseconds. */
#define DIRECTION_PAUSE_MS    200U  /* Stop time before each direction change. */

int main(void)
{
    Stm32_Clock_Init(9U);       /* 8 MHz crystal x 9 = 72 MHz */
    MY_NVIC_PriorityGroupConfig(2U);
    delay_init();
    uart_init(115200U);         /* Retained from the serial-port reference project. */
    JTAG_Set(SWD_ENABLE);       /* Retain SWD programming and debugging. */
    motor_init();

    printf("Motor PWM control started\r\n");
    while (1)
    {
        printf("Forward\r\n");
        motor_forward(FORWARD_DUTY_PERCENT);
        delay_ms(FORWARD_TIME_MS);

        motor_stop();
        delay_ms(DIRECTION_PAUSE_MS);

        printf("Reverse\r\n");
        motor_reverse(REVERSE_DUTY_PERCENT);
        delay_ms(REVERSE_TIME_MS);

        motor_stop();
        delay_ms(DIRECTION_PAUSE_MS);
    }
}
