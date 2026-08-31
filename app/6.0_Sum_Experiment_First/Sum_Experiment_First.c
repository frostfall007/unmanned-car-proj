#include <stdio.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "hi_io.h"
#include "hi_time.h"
#include "hal_bsp_ssd1306.h"
#include "hal_bsp_sht20.h"
#include "hal_bsp_ap3216c.h"

#define SERVO_GPIO 2
#define HCSR_TRIG_GPIO 7
#define HCSR_ECHO_GPIO 8
#define TCRT_LEFT_GPIO 13
#define TCRT_RIGHT_GPIO 14

#define SERVO_PULSE_MIN_US 500U
#define SERVO_PULSE_MAX_US 2500U
#define SERVO_PERIOD_US 20000U
#define SERVO_PULSE_COUNT 25U
#define ULTRASONIC_TIMEOUT_US 30000UL
#define SCAN_INTERVAL_TICKS 1000U
#define REPORT_INTERVAL_TICKS 100U
#define LINE_POLL_TICKS 10U

/* Adjust this level if the tracking module is wired with inverse logic. */
#define TCRT_LINE_ACTIVE_LEVEL WIFI_IOT_GPIO_VALUE0

typedef struct {
    float distanceCm;
    float temperature;
    float humidity;
    uint16_t ir;
    uint16_t als;
    uint16_t ps;
    uint8_t validDistance;
    uint8_t validEnvironment;
} SensorData;

static SensorData g_sensorData;
static osMutexId_t g_dataMutex;
static osMutexId_t g_i2cMutex;
static osMutexId_t g_printMutex;

static void PrintLine(const char *text)
{
    osMutexAcquire(g_printMutex, osWaitForever);
    printf("%s", text);
    osMutexRelease(g_printMutex);
}

static void ConfigureGpio(void)
{
    GpioInit();

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(SERVO_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(SERVO_GPIO, WIFI_IOT_GPIO_VALUE0);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(HCSR_TRIG_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(HCSR_TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);
    GpioSetDir(HCSR_ECHO_GPIO, WIFI_IOT_GPIO_DIR_IN);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(TCRT_LEFT_GPIO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(TCRT_RIGHT_GPIO, WIFI_IOT_GPIO_DIR_IN);
}

static void ServoMoveTo(uint8_t angle)
{
    uint32_t pulseWidth = SERVO_PULSE_MIN_US + ((uint32_t)angle *
        (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180U);
    uint32_t index;

    for (index = 0U; index < SERVO_PULSE_COUNT; ++index) {
        GpioSetOutputVal(SERVO_GPIO, WIFI_IOT_GPIO_VALUE1);
        hi_udelay(pulseWidth);
        GpioSetOutputVal(SERVO_GPIO, WIFI_IOT_GPIO_VALUE0);
        hi_udelay(SERVO_PERIOD_US - pulseWidth);
    }
}

static float UltrasonicReadDistance(void)
{
    WifiIotGpioValue value;
    unsigned long start;
    unsigned long elapsed;

    GpioSetOutputVal(HCSR_TRIG_GPIO, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20U);
    GpioSetOutputVal(HCSR_TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);

    start = hi_get_us();
    do {
        GpioGetInputVal(HCSR_ECHO_GPIO, &value);
        if ((hi_get_us() - start) > ULTRASONIC_TIMEOUT_US) {
            return -1.0f;
        }
    } while (value == WIFI_IOT_GPIO_VALUE0);

    start = hi_get_us();
    do {
        GpioGetInputVal(HCSR_ECHO_GPIO, &value);
        elapsed = hi_get_us() - start;
        if (elapsed > ULTRASONIC_TIMEOUT_US) {
            return -1.0f;
        }
    } while (value == WIFI_IOT_GPIO_VALUE1);

    return (float)elapsed * 0.034f / 2.0f;
}

static const char *LineState(WifiIotGpioValue left, WifiIotGpioValue right)
{
    uint8_t leftOnLine = (left == TCRT_LINE_ACTIVE_LEVEL);
    uint8_t rightOnLine = (right == TCRT_LINE_ACTIVE_LEVEL);

    if ((leftOnLine != 0U) && (rightOnLine != 0U)) return "forward";
    if (leftOnLine != 0U) return "turn_left";
    if (rightOnLine != 0U) return "turn_right";
    return "line_lost";
}

static void ScanTask(void *argument)
{
    static const uint8_t angles[] = {0U, 90U, 180U};
    static const char *positions[] = {"left", "center", "right"};
    uint32_t index = 0U;

    (void)argument;
    while (1) {
        float distance;

        ServoMoveTo(angles[index]);
        distance = UltrasonicReadDistance();
        osMutexAcquire(g_dataMutex, osWaitForever);
        g_sensorData.distanceCm = distance;
        g_sensorData.validDistance = (distance >= 0.0f);
        osMutexRelease(g_dataMutex);

        osMutexAcquire(g_printMutex, osWaitForever);
        if (distance < 0.0f) {
            printf("SCAN position=%s distance=timeout\r\n", positions[index]);
        } else {
            printf("SCAN position=%s distance=%.1fcm\r\n", positions[index], distance);
        }
        osMutexRelease(g_printMutex);

        index = (index + 1U) % 3U;
        osDelay(SCAN_INTERVAL_TICKS);
    }
}

static void LineTrackingTask(void *argument)
{
    WifiIotGpioValue lastLeft = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue lastRight = WIFI_IOT_GPIO_VALUE0;
    uint8_t hasLastValue = 0U;

    (void)argument;
    while (1) {
        WifiIotGpioValue left;
        WifiIotGpioValue right;

        GpioGetInputVal(TCRT_LEFT_GPIO, &left);
        GpioGetInputVal(TCRT_RIGHT_GPIO, &right);
        if ((hasLastValue == 0U) || (left != lastLeft) || (right != lastRight)) {
            osMutexAcquire(g_printMutex, osWaitForever);
            printf("LINE left=%u right=%u command=%s\r\n", left, right, LineState(left, right));
            osMutexRelease(g_printMutex);
            lastLeft = left;
            lastRight = right;
            hasLastValue = 1U;
        }
        osDelay(LINE_POLL_TICKS);
    }
}

static void EnvironmentTask(void *argument)
{
    (void)argument;
    while (1) {
        float temperature = 0.0f;
        float humidity = 0.0f;
        uint16_t ir = 0U;
        uint16_t als = 0U;
        uint16_t ps = 0U;
        uint32_t shtResult;
        uint32_t apResult;

        osMutexAcquire(g_i2cMutex, osWaitForever);
        shtResult = SHT20_ReadData(&temperature, &humidity);
        apResult = AP3216C_ReadData(&ir, &als, &ps);
        osMutexRelease(g_i2cMutex);

        if ((shtResult == 0U) && (apResult == 0U)) {
            osMutexAcquire(g_dataMutex, osWaitForever);
            g_sensorData.temperature = temperature;
            g_sensorData.humidity = humidity;
            g_sensorData.ir = ir;
            g_sensorData.als = als;
            g_sensorData.ps = ps;
            g_sensorData.validEnvironment = 1U;
            osMutexRelease(g_dataMutex);
        } else {
            osMutexAcquire(g_printMutex, osWaitForever);
            printf("ENV read_failed sht20=0x%08X ap3216c=0x%08X\r\n", shtResult, apResult);
            osMutexRelease(g_printMutex);
        }
        osDelay(REPORT_INTERVAL_TICKS);
    }
}

static void ReportTask(void *argument)
{
    (void)argument;
    while (1) {
        SensorData snapshot;

        osMutexAcquire(g_dataMutex, osWaitForever);
        snapshot = g_sensorData;
        osMutexRelease(g_dataMutex);

        osMutexAcquire(g_printMutex, osWaitForever);
        if (snapshot.validDistance != 0U) {
            printf("DISTANCE=%.1fcm ", snapshot.distanceCm);
        } else {
            printf("DISTANCE=unavailable ");
        }
        if (snapshot.validEnvironment != 0U) {
            printf("TEMP=%.2fC HUMIDITY=%.2f%% IR=%u ALS=%u PS=%u\r\n",
                snapshot.temperature, snapshot.humidity, snapshot.ir, snapshot.als, snapshot.ps);
        } else {
            printf("ENVIRONMENT=unavailable\r\n");
        }
        osMutexRelease(g_printMutex);
        osDelay(REPORT_INTERVAL_TICKS);
    }
}

static void BluetoothTask(void *argument)
{
    WifiIotUartAttribute uartAttr = {
        .baudRate = 9600,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    unsigned char buffer[128];
    uint32_t result;

    (void)argument;
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);
    result = UartInit(WIFI_IOT_UART_IDX_1, &uartAttr, NULL);
    if (result != 0U) {
        osMutexAcquire(g_printMutex, osWaitForever);
        printf("BLE initialization_failed error=0x%08X\r\n", result);
        osMutexRelease(g_printMutex);
        return;
    }

    PrintLine("BLE ready baud=9600\r\n");
    while (1) {
        uint32_t received = UartRead(WIFI_IOT_UART_IDX_1, buffer, sizeof(buffer) - 1U);

        if (received == 0U) {
            osDelay(1U);
            continue;
        }
        if (received >= sizeof(buffer)) {
            PrintLine("BLE receive_failed invalid_length\r\n");
            continue;
        }
        buffer[received] = '\0';
        osMutexAcquire(g_printMutex, osWaitForever);
        printf("BLE RX: %s\r\n", buffer);
        osMutexRelease(g_printMutex);
    }
}

static int InitI2cDevices(void)
{
    uint32_t result;

    osMutexAcquire(g_i2cMutex, osWaitForever);
    result = SSD1306_Init();
    if (result == 0U) {
        SSD1306_CLS();
        SSD1306_ShowStr(0U, 1U, (uint8_t *)"Have a nice day!", 16U);
    }
    if (result == 0U) result = SHT20_Init();
    if (result == 0U) result = AP3216C_Init();
    osMutexRelease(g_i2cMutex);

    return (result == 0U) ? 0 : -1;
}

static void CreateTask(const char *name, osThreadFunc_t entry, uint32_t priority)
{
    osThreadAttr_t attr;

    attr.name = name;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024U * 4U;
    attr.priority = (osPriority_t)priority;
    if (osThreadNew(entry, NULL, &attr) == NULL) {
        osMutexAcquire(g_printMutex, osWaitForever);
        printf("Task creation_failed name=%s\r\n", name);
        osMutexRelease(g_printMutex);
    }
}

static void SumExperimentFirst(void)
{
    g_dataMutex = osMutexNew(NULL);
    g_i2cMutex = osMutexNew(NULL);
    g_printMutex = osMutexNew(NULL);
    if ((g_dataMutex == NULL) || (g_i2cMutex == NULL) || (g_printMutex == NULL)) {
        printf("Mutex creation failed.\r\n");
        return;
    }

    ConfigureGpio();
    if (InitI2cDevices() != 0) {
        PrintLine("I2C device initialization failed.\r\n");
        return;
    }

    PrintLine("SUM experiment started.\r\n");
    CreateTask("scan", ScanTask, osPriorityNormal);
    CreateTask("line_tracking", LineTrackingTask, osPriorityNormal);
    CreateTask("environment", EnvironmentTask, osPriorityNormal);
    CreateTask("report", ReportTask, osPriorityNormal);
    CreateTask("bluetooth", BluetoothTask, osPriorityNormal);
}

APP_FEATURE_INIT(SumExperimentFirst);
