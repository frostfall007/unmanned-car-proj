#ifndef __CAR_PROTOCOL_H
#define __CAR_PROTOCOL_H

#include "stm32f10x.h"

#define CAR_FRAME_HEAD  0xFCU
#define CAR_FRAME_TAIL  0xFDU
#define CAR_SPEED_MAX   150U

extern volatile u8 CAR_buff[4];
extern volatile u8 uart_rec_flag;

#endif
