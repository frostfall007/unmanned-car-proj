#include "stm32f10x.h"
#include "sys.h"
#include "colorful_led.h"

int main(void)
  { 
		Stm32_Clock_Init(9);						// Set the system clock to 72 MHz.
		MY_NVIC_PriorityGroupConfig(2);	// Set interrupt priority grouping.
		uart_init(115200);	            // Initialize UART at 115200 baud.
		JTAG_Set(JTAG_SWD_DISABLE);     // Disable the JTAG interface.
		JTAG_Set(SWD_ENABLE);           // Keep the SWD interface enabled for debugging.

		colorful_led_Init();            // Initialize the RGB LEDs.
		//SysTick_Config(72000000/1000); // Generate a SysTick interrupt every 1 ms.
    
		/* Update both LED effects in the main loop. */
	while(1)
	{
		L_runingled();                  // Update the front LEDs.
		R_runingled();                  // Update the rear LEDs.
		delay_ms(100);                  // Set the animation speed.
	}
}
	
