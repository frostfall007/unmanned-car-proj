#include <stdio.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hal_bsp_ap3216c.h"

#define STAIR_LED_GPIO 6

/* 根据实际安装高度和环境光，修改下面四个阈值。 */
#define DARK_ENTER_THRESHOLD 1000U
#define DARK_LEAVE_THRESHOLD 1200U
#define NEAR_ENTER_THRESHOLD 65U
#define NEAR_LEAVE_THRESHOLD 50U

static void StairLightSet(uint8_t on)
{
    GpioSetOutputVal(STAIR_LED_GPIO, on ? WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0);
}

static uint8_t StairIsDark(uint16_t als, uint8_t wasDark)
{
    if (wasDark != 0U) {
        return als < DARK_LEAVE_THRESHOLD;
    }
    return als <= DARK_ENTER_THRESHOLD;
}

static uint8_t StairIsNear(uint16_t ps, uint8_t wasNear)
{
    if (wasNear != 0U) {
        return ps > NEAR_LEAVE_THRESHOLD;
    }
    return ps >= NEAR_ENTER_THRESHOLD;
}

static void StairLightTask(void *argument)
{
    uint8_t isDark = 0U;
    uint8_t isNear = 0U;
    uint8_t isLightOn = 0U;

    (void)argument;
    uint32_t initResult = AP3216C_Init();
    if (initResult != 0U) {
        printf("AP3216C initialization failed: error=0x%08X, I2C0 address=0x3C.\r\n", initResult);
        return;
    }

    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_6, WIFI_IOT_IO_FUNC_GPIO_6_GPIO);
    GpioSetDir(STAIR_LED_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    StairLightSet(0U);
    printf("Stair light started: ALS<=%u and PS>=%u turns LED on.\r\n",
        DARK_ENTER_THRESHOLD, NEAR_ENTER_THRESHOLD);

    while (1) {
        uint16_t ir = 0U;
        uint16_t als = 0U;
        uint16_t ps = 0U;
        uint8_t shouldLight;

        if (AP3216C_ReadData(&ir, &als, &ps) != 0U) {
            printf("AP3216C read failed.\r\n");
            osDelay(20U);
            continue;
        }

        isDark = StairIsDark(als, isDark);
        isNear = StairIsNear(ps, isNear);
        shouldLight = (isDark != 0U) && (isNear != 0U);
        if (shouldLight != isLightOn) {
            StairLightSet(shouldLight);
            isLightOn = shouldLight;
        }

        printf("IR=%u ALS=%u PS=%u | %s | %s | LED=%s\r\n", ir, als, ps,
            isDark ? "dark" : "bright", isNear ? "near" : "far",
            isLightOn ? "on" : "off");
        osDelay(20U);
    }
}

static void Ap3216cStairLightEntry(void)
{
    osThreadAttr_t attr;

    attr.name = "stair_light";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024U * 4U;
    attr.priority = osPriorityNormal;
    if (osThreadNew(StairLightTask, NULL, &attr) == NULL) {
        printf("Failed to create stair light task.\r\n");
    }
}

APP_FEATURE_INIT(Ap3216cStairLightEntry);
