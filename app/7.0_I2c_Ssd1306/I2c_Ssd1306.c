#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_ssd1306.h"

// ==================== 任务执行函数 ====================
void Task1(void)
{
    uint8_t displayBuff[20] = {0};
    uint8_t hour = 16, min = 0, sec = 0;

    SSD1306_Init(); // OLED 显示屏初始化
    SSD1306_CLS();  // 清屏

    // 写入固定显示内容
    SSD1306_ShowStr(0, 0, (uint8_t *)"   QST CAR   ", 16);
    SSD1306_ShowStr(0, 3, (uint8_t *)"2025:10:08", 16);

    while (1)
    {
        sec++;
        if (sec > 59)
        {
            sec = 0;
            min++;
        }
        if (min > 59)
        {
            min = 0;
            hour++;
        }
        if (hour > 23)
        {
            hour = 0;
        }

        // 清除 displayBuff 中的字符串
        memset(displayBuff, 0, sizeof(displayBuff));
        
        // 格式化时间字符串
        sprintf((char*)displayBuff, "%02d:%02d:%02d", hour, min, sec);
        
        // 写入 OLED 显示
        SSD1306_ShowStr(0, 2, (uint8_t *)displayBuff, 16);
        
        sleep(1); // 延时 1 秒
    }
}

// ==================== 任务创建入口 ====================
static void i2c_ssd1306_demo(void)
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
        printf("ID = %d, Create Task1_ID is OK!\r\n", Task1_ID);
    }
}

// ==================== 任务启动 ====================
APP_FEATURE_INIT(i2c_ssd1306_demo);