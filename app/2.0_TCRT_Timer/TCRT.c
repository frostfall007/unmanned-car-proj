/*
 * Copyright (c) 2020 Huawei Device Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"

/*
 * 小车原理图中：
 * TC_OUT_L -> GPIO13，TC_OUT_R -> GPIO14。
 * TCRT5000 的红外发射管由 3.3VD 经限流电阻供电，上电即点亮；
 * 因此这里只将两个比较器输出配置为 GPIO 输入，不可用 GPIO 直接驱动发射管。
 */
#define TCRT_LEFT_GPIO  13
#define TCRT_RIGHT_GPIO 14

/* 本工程的系统 tick 为 10 ms，10 tick 即 100 ms。 */
#define TCRT_POLL_PERIOD_TICKS 10U

static osTimerId_t g_tcrtTimer;
static WifiIotGpioValue g_lastLeft = WIFI_IOT_GPIO_VALUE0;
static WifiIotGpioValue g_lastRight = WIFI_IOT_GPIO_VALUE0;
static unsigned char g_hasLastValue;

/* 软件定时器到期后由系统的定时器服务线程调用。 */
static void TcrtTimerCallback(void *argument)
{
    WifiIotGpioValue left;
    WifiIotGpioValue right;

    (void)argument;
    GpioGetInputVal(TCRT_LEFT_GPIO, &left);
    GpioGetInputVal(TCRT_RIGHT_GPIO, &right);

    /* 仅在电平变化时打印，串口输出不会淹没定时器的采样结果。 */
    if ((g_hasLastValue == 0U) || (left != g_lastLeft) || (right != g_lastRight)) {
        printf("TCRT: left=%d, right=%d\r\n", left, right);
        g_lastLeft = left;
        g_lastRight = right;
        g_hasLastValue = 1U;
    }
}

static void TcrtInit(void)
{
    osTimerAttr_t timerAttr;

    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(TCRT_LEFT_GPIO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(TCRT_RIGHT_GPIO, WIFI_IOT_GPIO_DIR_IN);

    timerAttr.name = "tcrt_poll";
    timerAttr.attr_bits = 0U;
    timerAttr.cb_mem = NULL;
    timerAttr.cb_size = 0U;
    g_tcrtTimer = osTimerNew(TcrtTimerCallback, osTimerPeriodic, NULL, &timerAttr);
    if (g_tcrtTimer == NULL) {
        printf("Failed to create TCRT timer!\r\n");
        return;
    }

    if (osTimerStart(g_tcrtTimer, TCRT_POLL_PERIOD_TICKS) != osOK) {
        printf("Failed to start TCRT timer!\r\n");
        return;
    }

    printf("TCRT timer started: GPIO13=left, GPIO14=right\r\n");
}

APP_FEATURE_INIT(TcrtInit);
