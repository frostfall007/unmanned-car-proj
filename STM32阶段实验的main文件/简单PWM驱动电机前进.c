#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"

int main(void)
{
    RCC->CSR |= 1<<24; //清除
    Stm32_Clock_Init(9); //外部时钟8Mhz 9倍频 8*9= 72mhz倍频72mhz
    MY_NVIC_PriorityGroupConfig(2); //=====中断优先级分组
    uart_init(115200);              //=====串口初始化为115200
    JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试

    PWM_Init(7199, 9);              //=====定时器初始化 频率1000
    colorful_led_Init();            //=====炫彩灯初始化
    printf("QST青软\r\n");

    /**主要程序**/
    while(1)
    {
        Set_Pwm(2500, 2500);        //设置左右轮速度
        delay_ms(100);
    }
}