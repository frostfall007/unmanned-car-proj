#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include "wifiiot_uart.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_pwm.h"
#include "hi_pwm.h"
#include "hi_uart.h"
#include "wifiiot_gpio_ex.h"

uint8_t uart_sendbuf[20];
osMutexId_t mutex_id;

/* TCRT5000: GPIO13 is the left sensor and GPIO14 is the right sensor. */
#define TCRT_LEFT_GPIO                 WIFI_IOT_IO_NAME_GPIO_13
#define TCRT_RIGHT_GPIO                WIFI_IOT_IO_NAME_GPIO_14

/* The sensor LED is normally on over the table and its GPIO level is 0. */
#define TABLE_DETECTED_LEVEL           WIFI_IOT_GPIO_VALUE0

#define CAR_CONTROL_INTERVAL_US        50000U
#define CAR_CONTROL_INTERVAL_MS        50U
#define CAR_CRUISE_SPEED               50
#define CAR_REVERSE_SPEED              35
#define CAR_TURN_INNER_SPEED           20
#define CAR_TURN_OUTER_SPEED           65
#define CLIFF_REVERSE_MIN_MS           5000U
#define CLIFF_REVERSE_LIMIT_MS         100000U
#define CLIFF_TURN_DURATION_MS         5000U
#define CLIFF_RECOVERY_STOP_MS         200U

typedef enum {
    TURN_NONE = 0,
    TURN_LEFT,
    TURN_RIGHT
} TurnDirection;

/***通信协议***/
/*
 * 函数功能：发送至stm32的数据协议
 * 参数：电机实际转速的一百倍，例如：设置转速为1rad/s，则传入100
 */
void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;

    // 小车运动方向 前进（正转）：0 后退（反转） 1
    if(motorA<0){
        A_dir=1;
        motorA = -motorA;
    }else{
        A_dir=0;
    }

    if(motorB<0){
        B_dir=1;
        motorB = -motorB;
    }else{
        B_dir=0;
    }

    //限制幅度 -150 ~150
    if (motorA > 150) {
        motorA = 150;
    }
    if (motorB > 150) {
        motorB = 150;
    }

    // 数据协议
    uart_sendbuf[0] = 0xFC; // 帧头
    uart_sendbuf[1] = A_dir; // 左轮方向 0正转，1反转
    uart_sendbuf[2] = motorA; // 左轮速度
    uart_sendbuf[3] = B_dir; // 右轮方向 0正转，1反转
    uart_sendbuf[4] = motorB; // 右轮速度
    uart_sendbuf[5] = 0xFD; // 帧尾

    if (mutex_id != NULL) {
        osMutexAcquire(mutex_id, osWaitForever);
    }

    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);

    if (mutex_id != NULL) {
        osMutexRelease(mutex_id);
    }
}

// 小车后退
void car_backward(void)
{
    stm32motor_control(-100, -100);
}

// 小车前进
void car_forward(void)
{
    stm32motor_control(100, 100);
}

// 小车左转
void car_left(void)
{
    stm32motor_control(50, 150);
}

// 小车右转
void car_right(void)
{
    stm32motor_control(150, 50);
}

// 小车停止
void car_stop(void)
{
    stm32motor_control(0, 0);
}

static void car_safe_forward(void)
{
    stm32motor_control(CAR_CRUISE_SPEED, CAR_CRUISE_SPEED);
}

static void car_safe_backward(void)
{
    stm32motor_control(-CAR_REVERSE_SPEED, -CAR_REVERSE_SPEED);
}

static void car_safe_left(void)
{
    stm32motor_control(CAR_TURN_INNER_SPEED, CAR_TURN_OUTER_SPEED);
}

static void car_safe_right(void)
{
    stm32motor_control(CAR_TURN_OUTER_SPEED, CAR_TURN_INNER_SPEED);
}

static void cliff_sensor_init(void)
{
    IoSetFunc(TCRT_LEFT_GPIO, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(TCRT_RIGHT_GPIO, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(TCRT_LEFT_GPIO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(TCRT_RIGHT_GPIO, WIFI_IOT_GPIO_DIR_IN);
}

static int sensor_over_table(WifiIotGpioValue value)
{
    return value == TABLE_DETECTED_LEVEL;
}

static void car_auto_test(void *argument)
{
    WifiIotGpioValue left;
    WifiIotGpioValue right;
    WifiIotGpioValue previous_left = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue previous_right = WIFI_IOT_GPIO_VALUE0;
    TurnDirection pending_turn = TURN_NONE;
    TurnDirection next_double_edge_turn = TURN_RIGHT;
    uint32_t reverse_time_ms = 0U;
    uint32_t turn_time_ms = 0U;
    int recovering = 0;
    int has_previous_sample = 0;
    int left_safe;
    int right_safe;

    (void)argument;

    while (1) {
        GpioGetInputVal(TCRT_LEFT_GPIO, &left);
        GpioGetInputVal(TCRT_RIGHT_GPIO, &right);
        if (!has_previous_sample || (left != previous_left) || (right != previous_right)) {
            printf("TCRT raw: left=%d, right=%d\r\n", left, right);
            previous_left = left;
            previous_right = right;
            has_previous_sample = 1;
        }
        left_safe = sensor_over_table(left);
        right_safe = sensor_over_table(right);

        if (!left_safe || !right_safe) {
            if (!recovering) {
                if (!left_safe && right_safe) {
                    pending_turn = TURN_RIGHT;
                } else if (left_safe && !right_safe) {
                    pending_turn = TURN_LEFT;
                } else {
                    pending_turn = next_double_edge_turn;
                    next_double_edge_turn =
                        (next_double_edge_turn == TURN_RIGHT) ? TURN_LEFT : TURN_RIGHT;
                }

                printf("TCRT edge: left=%d, right=%d\r\n", left, right);
                recovering = 1;
                reverse_time_ms = 0U;
                turn_time_ms = 0U;
                car_stop();
                usleep(CLIFF_RECOVERY_STOP_MS * 1000U);
                continue;
            }

            turn_time_ms = 0U;

            if (reverse_time_ms < CLIFF_REVERSE_LIMIT_MS) {
                car_safe_backward();
                reverse_time_ms += CAR_CONTROL_INTERVAL_MS;
            } else {
                car_stop();
            }

            usleep(CAR_CONTROL_INTERVAL_US);
            continue;
        }

        if (recovering && (reverse_time_ms < CLIFF_REVERSE_MIN_MS)) {
            car_safe_backward();
            reverse_time_ms += CAR_CONTROL_INTERVAL_MS;
            usleep(CAR_CONTROL_INTERVAL_US);
            continue;
        }

        if (recovering) {
            printf("TCRT safe: left=%d, right=%d, turn=%d\r\n",
                   left, right, pending_turn);
            car_stop();
            usleep(CLIFF_RECOVERY_STOP_MS * 1000U);
            recovering = 0;
            reverse_time_ms = 0U;
            continue;
        }

        if (pending_turn != TURN_NONE) {
            if (pending_turn == TURN_RIGHT) {
                car_safe_right();
            } else {
                car_safe_left();
            }

            turn_time_ms += CAR_CONTROL_INTERVAL_MS;
            if (turn_time_ms >= CLIFF_TURN_DURATION_MS) {
                pending_turn = TURN_NONE;
                turn_time_ms = 0U;
            }
        } else {
            car_safe_forward();
        }

        usleep(CAR_CONTROL_INTERVAL_US);
    }
}

/*****任务创建*****/
static void correspondence(void)
{
    GpioInit(); // GPIO功能初始化

    /*********************通讯串口初始化*********************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD); // GPIO_11复用为UART2_TXD
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD); // GPIO_12复用为UART2_RX

    /**************串口参数**************/
    WifiIotUartAttribute uart_attr2 = {
        // 波特率: 115200
        .baudRate = 115200,
        // 数据位: 8bits
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);

    mutex_id = osMutexNew(NULL); // 创建互斥锁
    if (mutex_id == NULL)
    {
        printf("Failed to create Mutex!\n");
    }

    car_stop();
    cliff_sensor_init();
    printf("TCRT ready: table=0, edge=1\r\n");

    {
        osThreadAttr_t attr = {
            .name = "car_auto_test",
            .priority = 25,
            .stack_size = 1024 * 4,
        };

        if (osThreadNew(car_auto_test, NULL, &attr) == NULL)
        {
            printf("Failed to create car_auto_test!\n");
        }
    }
}

APP_FEATURE_INIT(correspondence); // 启动任务
