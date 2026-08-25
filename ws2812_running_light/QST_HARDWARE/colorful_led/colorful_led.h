#ifndef __COLORFUL_LED_H
#define __COLORFUL_LED_H

#include "sys.h"

/* The schematic shows six WS2812 LEDs on each of the front and rear chains. */
#define WS2812_LED_COUNT 6U

void colorful_led_Init(void);
void colorful_led_running_step(void);
void colorful_led_off(void);

#endif
