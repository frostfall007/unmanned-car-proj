#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_ap3216c.h"

/**
 * ir 人体红外传感器
 * als 光强传感器
 * ps 接近传感器
 */
void Task1(void)
{
    AP3216C_Init(); // 三合一传感器初始化
    printf("i2c_ap3216c_demo()!");

    uint16_t ir = 0, als = 0, ps = 0;

    while (1)
    {
        AP3216C_ReadData(&ir, &als, &ps);
        printf("人体红外传感器(ir) = %d\t光强传感器(als) = %d\t接近传感器(ps) = %d\r\n", ir, als, ps);
        sleep(1); // 延时 1s
    }
}

// (2) 任务创建
static void i2c_ap3216c_demo(void)
{
    osThreadAttr_t options;
    options.name = "thread_1";
    options.attr_bits = 0;
    options.cb_mem = NULL;
    options.cb_size = 0;
    options.stack_mem = NULL;
    options.stack_size = 1024;
    options.priority = osPriorityNormal;

    osThreadId_t Task1_ID;
    Task1_ID = osThreadNew((osThreadFunc_t)Task1, NULL, &options);

    if (Task1_ID != NULL)
    {
        printf("ID = %d, Create Task1_ID is OK!", Task1_ID);
    }
}

// (4) 启动任务（添加在整个文件的最末尾）
APP_FEATURE_INIT(i2c_ap3216c_demo);