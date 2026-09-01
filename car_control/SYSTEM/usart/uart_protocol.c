#include "sys.h"
#include "usart.h"
#include "car_protocol.h"

#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};

FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}

int fputc(int ch, FILE *f)
{
    while ((USART1->SR & USART_SR_TC) == 0U) {}
    USART1->DR = (u8)ch;
    return ch;
}

#if EN_USART1_RX

u8 USART_RX_BUF[USART_REC_LEN];
u8 USART_RX_STA = 0;
volatile u8 CAR_buff[4] = {0, 0, 0, 0};
volatile u8 uart_rec_flag = 0;
static u8 rx_count = 0;

void uart_init(u32 bound)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    usart.USART_BaudRate = bound;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
    u8 byte;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == RESET)
    {
        return;
    }

    byte = (u8)USART_ReceiveData(USART1);
    if (rx_count == 0U)
    {
        if (byte == CAR_FRAME_HEAD)
        {
            USART_RX_BUF[rx_count++] = byte;
        }
        return;
    }

    USART_RX_BUF[rx_count++] = byte;
    if (rx_count < 6U)
    {
        return;
    }

    if (USART_RX_BUF[5] == CAR_FRAME_TAIL)
    {
        CAR_buff[0] = USART_RX_BUF[1];
        CAR_buff[1] = USART_RX_BUF[2];
        CAR_buff[2] = USART_RX_BUF[3];
        CAR_buff[3] = USART_RX_BUF[4];
        uart_rec_flag = 1U;
        USART_RX_STA = 1U;
    }

    rx_count = 0U;
}

#endif
