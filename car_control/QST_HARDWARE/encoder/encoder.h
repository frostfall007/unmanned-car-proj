#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

#define ENCODER_TIM_PERIOD  65535U

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
int Read_Encoder(u8 timer_number);

#endif
