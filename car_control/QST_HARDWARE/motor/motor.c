#include "motor.h"

/* Schematic mapping:
 *   Left motor:  PB7 (L-IA, TIM4_CH2 PWM), PB14 (L-IB direction)
 *   Right motor: PB6 (R-IB, TIM4_CH1 PWM), PB13 (R-IA direction)
 */
#define LEFT_PWM_PIN        GPIO_Pin_7
#define RIGHT_PWM_PIN       GPIO_Pin_6
#define LEFT_DIRECTION_PIN  GPIO_Pin_14
#define RIGHT_DIRECTION_PIN GPIO_Pin_13

/* TIM4 clock = 72 MHz. 72 MHz / (3 + 1) / (999 + 1) = 18 kHz. */
#define MOTOR_PWM_PRESCALER 3U
#define MOTOR_PWM_PERIOD    999U

static void motor_set_duty(u8 duty_percent)
{
    u16 compare;

    if (duty_percent > 100U)
    {
        duty_percent = 100U;
    }

    compare = (u16)(((u32)(MOTOR_PWM_PERIOD + 1U) * duty_percent) / 100U);
    TIM_SetCompare1(TIM4, compare);  /* PB6, right motor */
    TIM_SetCompare2(TIM4, compare);  /* PB7, left motor */
}

static void motor_set_pwm_mode(u16 mode)
{
    TIM_OCInitTypeDef pwm;

    TIM_OCStructInit(&pwm);
    pwm.TIM_OCMode = mode;
    pwm.TIM_OutputState = TIM_OutputState_Enable;
    pwm.TIM_Pulse = 0U;
    pwm.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &pwm);
    TIM_OC2Init(TIM4, &pwm);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

void motor_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    gpio.GPIO_Pin = LEFT_PWM_PIN | RIGHT_PWM_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = LEFT_DIRECTION_PIN | RIGHT_DIRECTION_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB, LEFT_DIRECTION_PIN | RIGHT_DIRECTION_PIN);

    timer.TIM_Prescaler = MOTOR_PWM_PRESCALER;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_Period = MOTOR_PWM_PERIOD;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM4, &timer);

    motor_set_pwm_mode(TIM_OCMode_PWM1);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    motor_stop();
}

void motor_forward(u8 duty_percent)
{
    /* IA is PWM and IB is low: the L9110 drives the motor forward. */
    motor_set_duty(0U);
    GPIO_ResetBits(GPIOB, LEFT_DIRECTION_PIN | RIGHT_DIRECTION_PIN);
    motor_set_pwm_mode(TIM_OCMode_PWM1);
    motor_set_duty(duty_percent);
}

void motor_reverse(u8 duty_percent)
{
    /* IA is PWM2 (low for the requested duty) and IB is high. This makes
     * reverse torque equal to duty_percent instead of 100 - duty_percent. */
    motor_set_duty(0U);
    GPIO_SetBits(GPIOB, LEFT_DIRECTION_PIN | RIGHT_DIRECTION_PIN);
    motor_set_pwm_mode(TIM_OCMode_PWM2);
    motor_set_duty(duty_percent);
}

void motor_stop(void)
{
    motor_set_duty(0U);
}
