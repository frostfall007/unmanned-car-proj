#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_uart.h"

osMutexId_t mutex_id;
#define GPIO2 2
uint8_t flag; // 舵机旋转角度标志位

static void thread1(void);
void thread2(void);
void thread3(void);

// ====== 校准偏移量（单位：度），根据实际测量修改 ======
#define ANGLE_OFFSET 0

// 函数前向声明
static void thread1(void);
void thread2(void);
void thread3(void);

/**
 * 舵机控制核心函数
 */
void set_angle(unsigned int duty)
{
    GpioSetDir(GPIO2, WIFI_IOT_GPIO_DIR_OUT);

    GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);

    GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

/**
 * 角度转脉冲宽度（带偏移校准）
 * 0度=500us, 180度=2500us, 线性映射
 */
unsigned int angle_to_duty(int angle)
{
    int real_angle = angle + ANGLE_OFFSET;
    if (real_angle < 0) real_angle = 0;
    if (real_angle > 180) real_angle = 180;
    return 500 + (unsigned int)real_angle * 2000 / 180;
}

/* ==================== 上电归中校准 ==================== */
void SG90_Calibrate(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("    SG90 Auto Centering (90 degrees)    \r\n");
    printf("========================================\r\n");
    printf("Servo is moving to center position...   \r\n");

    // 连续发送90度脉冲约500ms，让舵机转到中间位置
    for (int i = 0; i < 25; i++) {
        set_angle(1500);
    }
    hi_udelay(600000); // 等待600ms让舵机物理到位

    printf("Servo is at CENTER now!                 \r\n");
    printf("Please adjust the arm to 'forward'.     \r\n");
    printf("Running will start in 5 seconds...      \r\n");
    printf("========================================\r\n");

    // 等待5秒，给你足够时间手动拨正摆臂
    hi_udelay(5000000);

    printf("[OK] Calibration done! Starting...\r\n\r\n");
}
/* ================================================================ */

/** 控制舵机旋转到指定角度（带偏移校准） */
void engine_run_angle(int angle)
{
    unsigned int duty = angle_to_duty(angle);
    for (int i = 0; i < 10; i++) {
        set_angle(duty);
    }
}

/** 控制舵机旋转0度 */
void engine_run_0(void)
{
    engine_run_angle(0);
}

/** 控制舵机旋转45度 */
void engine_run_45(void)
{
    engine_run_angle(45);
}

/** 控制舵机旋转90度 */
void engine_run_90(void)
{
    engine_run_angle(90);
}

/** 控制舵机旋转135度 */
void engine_run_135(void)
{
    engine_run_angle(135);
}

/** 控制舵机旋转180度 */
void engine_run_180(void)
{
    engine_run_angle(180);
}

/* 任务创建 */
static void SG90(void)
{
    // 1. 初始化GPIO
    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_GPIO_DIR_OUT);

    // 2. 上电归中校准
    SG90_Calibrate();

    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;
    attr.priority = 26;

    // 3. 创建线程1
    attr.name = "thread1";
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL) {
        printf("Failed to create thread1!\n");
    }

    // 4. 创建线程2
    attr.name = "thread2";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL) {
        printf("Failed to create thread2!\n");
    }

    // 5. 创建线程3
    attr.name = "thread3";
    attr.priority = 24;
    if (osThreadNew((osThreadFunc_t)thread3, NULL, &attr) == NULL) {
        printf("Failed to create thread3!\n");
    }

    // 6. 创建互斥锁
    mutex_id = osMutexNew(NULL);
    if (mutex_id == NULL) {
        printf("Failed to create Mutex!\n");
    }
}

/* 任务1函数（高优先级，控制舵机转动90度） */
static void thread1(void)
{
    osDelay(100U);
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);
        printf("thread1 is runing. \r\n");
        flag = 90;
        engine_run_90();
        osDelay(500U);
        osMutexRelease(mutex_id);

        // 释放锁后主动让出CPU，给thread3抢锁的机会
        osDelay(100U);
    }
}

/* 任务2函数（中优先级，只负责打印） */
void thread2(void)
{
    osDelay(100U);
    while (1) {
        printf("thread2 is runing. \r\n");
        switch(flag) {
            case 90:
                printf("SG90 turn 90 du. \r\n");
                break;
            case 180:
                printf("SG90 turn 180 du. \r\n");
                break;
            default:
                break;
        }
        flag = 0;
        osDelay(100);
    }
}

/* 任务3函数（低优先级，控制舵机转动180度） */
void thread3(void)
{
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);
        printf("thread3 is runing. \r\n");
        flag = 180;
        engine_run_180();
        osDelay(300U);
        osMutexRelease(mutex_id);
    }
}

// 启动任务
APP_FEATURE_INIT(SG90);