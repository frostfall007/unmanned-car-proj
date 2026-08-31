#include <stdio.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"

#define MSGQUEUE_OBJECTS 16U
#define UART_TASK_STACK_SIZE (1024U * 16U)
#define UART_TASK_PRIO 25
#define UART_READ_SIZE 1000U

typedef struct {
    char buf[UART_READ_SIZE + 1U];
    uint8_t idx;
} UartMessage;

static osMessageQueueId_t g_uartQueue;
static const char g_startMessage[] = "Hello, QST!\r\n";

static void UART_Task(void *argument);
static void UartReceiveTask(void *argument);
static void DebugTask(void *argument);

static int UartConfig(void)
{
    uint32_t ret;
    WifiIotUartAttribute uartAttr = {
        .baudRate = 9600,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    GpioInit();
    /* 原理图中 GPIO0 -> BLE-RX，GPIO1 -> BLE-TX。 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);

    ret = UartInit(WIFI_IOT_UART_IDX_1, &uartAttr, NULL);
    if (ret != WIFI_IOT_SUCCESS) {
        printf("Failed to init UART1! Err code = %u\r\n", ret);
        return -1;
    }

    return 0;
}

static osThreadId_t CreateUartThread(const char *name, osThreadFunc_t entry, uint32_t stackSize)
{
    osThreadAttr_t attr;

    attr.name = name;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = stackSize;
    attr.priority = UART_TASK_PRIO;
    return osThreadNew(entry, NULL, &attr);
}

/** 创建 UART 初始化、接收和调试任务。 */
static void UART_ExampleEntry(void)
{
    if (UartConfig() != 0) {
        return;
    }

    g_uartQueue = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(UartMessage), NULL);
    if (g_uartQueue == NULL) {
        printf("Failed to create UART message queue!\r\n");
        return;
    }

    if (CreateUartThread("uart_process", UART_Task, UART_TASK_STACK_SIZE) == NULL) {
        printf("Failed to create UART task!\r\n");
    }
    if (CreateUartThread("uart_receive", UartReceiveTask, UART_TASK_STACK_SIZE) == NULL) {
        printf("Failed to create UART receive task!\r\n");
    }
    if (CreateUartThread("uart_debug", DebugTask, 1024U) == NULL) {
        printf("Failed to create UART debug task!\r\n");
    }
}

/* 从队列取得一帧完整字符串。 */
static void UART_Task(void *argument)
{
    UartMessage message;

    (void)argument;
    printf("UART1 test start\r\n");
    UartWrite(WIFI_IOT_UART_IDX_1, (unsigned char *)g_startMessage, strlen(g_startMessage));

    while (1) {
        if (osMessageQueueGet(g_uartQueue, &message, NULL, osWaitForever) != osOK) {
            continue;
        }

        printf("UART1 received [%u]: %s\r\n", message.idx, message.buf);
    }
}

/* UART1 收到的数据复制进消息队列，队列不再保存线程栈缓冲区的指针。 */
static void UartReceiveTask(void *argument)
{
    uint8_t messageIndex = 0U;
    unsigned char uartBuffer[UART_READ_SIZE + 1U];

    (void)argument;
    while (1) {
        UartMessage message = {0};
        uint32_t received = UartRead(WIFI_IOT_UART_IDX_1, uartBuffer, UART_READ_SIZE);

        if (received == 0U) {
            osDelay(1U);
            continue;
        }
        if (received > UART_READ_SIZE) {
            printf("UART1 read failed: %u\r\n", received);
            continue;
        }

        memcpy(message.buf, uartBuffer, received);
        message.buf[received] = '\0';
        message.idx = ++messageIndex;

        if (osMessageQueuePut(g_uartQueue, &message, 0U, osWaitForever) != osOK) {
            printf("Failed to queue UART1 data\r\n");
        }
    }
}

static void DebugTask(void *argument)
{
    (void)argument;
    while (1) {
        printf("UART1 receive task is running\r\n");
        osDelay(200U);
    }
}

APP_FEATURE_INIT(UART_ExampleEntry);
