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
#define BLE_HELP_INTERVAL_TICKS 500U
#define MOTOR_KEEPALIVE_TICKS 20U
#define MOTOR_SPEED_SLOW 50
#define MOTOR_SPEED_NORMAL 100
#define MOTOR_SPEED_FAST 150

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
static osMutexId_t g_motorMutex;
static int g_activeLeftMotor;
static int g_activeRightMotor;
static uint8_t g_motionActive;

static const char g_bleHelpMessage[] =
    "READY. COMMANDS: O STOP, W FORWARD, A LEFT, D RIGHT, S REVERSE, "
    "I SPEED 50, K SPEED 150.\r\n";

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

static int InitMotorUart(void)
{
    WifiIotUartAttribute uartAttr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    uint32_t result;

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    result = UartInit(WIFI_IOT_UART_IDX_2, &uartAttr, NULL);
    if (result != 0U) {
        osMutexAcquire(g_printMutex, osWaitForever);
        printf("MOTOR UART initialization_failed error=0x%08X\r\n", result);
        osMutexRelease(g_printMutex);
        return -1;
    }
    return 0;
}

static void MotorControl(int leftMotor, int rightMotor, uint8_t reportResult)
{
    unsigned char frame[6];
    int written;

    frame[1] = (leftMotor < 0) ? 1U : 0U;
    frame[3] = (rightMotor < 0) ? 1U : 0U;
    if (leftMotor < 0) leftMotor = -leftMotor;
    if (rightMotor < 0) rightMotor = -rightMotor;
    if (leftMotor > MOTOR_SPEED_FAST) leftMotor = MOTOR_SPEED_FAST;
    if (rightMotor > MOTOR_SPEED_FAST) rightMotor = MOTOR_SPEED_FAST;

    frame[0] = 0xFCU;
    frame[2] = (unsigned char)leftMotor;
    frame[4] = (unsigned char)rightMotor;
    frame[5] = 0xFDU;

    osMutexAcquire(g_motorMutex, osWaitForever);
    written = UartWrite(WIFI_IOT_UART_IDX_2, frame, sizeof(frame));
    osMutexRelease(g_motorMutex);

    if (reportResult != 0U) {
        osMutexAcquire(g_printMutex, osWaitForever);
        printf("MOTOR TX=%d LEFT_DIR=%u LEFT_SPEED=%u RIGHT_DIR=%u RIGHT_SPEED=%u\r\n",
            written, frame[1], frame[2], frame[3], frame[4]);
        osMutexRelease(g_printMutex);
    }
}

static void SetMotionCommand(int leftMotor, int rightMotor)
{
    g_activeLeftMotor = leftMotor;
    g_activeRightMotor = rightMotor;
    g_motionActive = ((leftMotor != 0) || (rightMotor != 0)) ? 1U : 0U;
    MotorControl(leftMotor, rightMotor, 1U);
}

static void CarStop(void)
{
    SetMotionCommand(0, 0);
}

static void MotorKeepaliveTask(void *argument)
{
    (void)argument;
    PrintLine("MOTOR keepalive ready interval=200ms\r\n");
    while (1) {
        if (g_motionActive != 0U) {
            MotorControl(g_activeLeftMotor, g_activeRightMotor, 0U);
        }
        osDelay(MOTOR_KEEPALIVE_TICKS);
    }
}

static const char *HandleBluetoothCommand(unsigned char command)
{
    if ((command >= 'a') && (command <= 'z')) {
        command = (unsigned char)(command - ('a' - 'A'));
    }

    switch (command) {
        case 'O':
            CarStop();
            return "COMMAND=STOP\r\n";
        case 'W':
            SetMotionCommand(MOTOR_SPEED_NORMAL, MOTOR_SPEED_NORMAL);
            return "COMMAND=FORWARD SPEED=100\r\n";
        case 'A':
            SetMotionCommand(50, 150);
            return "COMMAND=LEFT\r\n";
        case 'D':
            SetMotionCommand(150, 50);
            return "COMMAND=RIGHT\r\n";
        case 'S':
            SetMotionCommand(-MOTOR_SPEED_NORMAL, -MOTOR_SPEED_NORMAL);
            return "COMMAND=REVERSE SPEED=100\r\n";
        case 'I':
            SetMotionCommand(MOTOR_SPEED_SLOW, MOTOR_SPEED_SLOW);
            return "COMMAND=FORWARD SPEED=50\r\n";
        case 'K':
            SetMotionCommand(MOTOR_SPEED_FAST, MOTOR_SPEED_FAST);
            return "COMMAND=FORWARD SPEED=150\r\n";
        default:
            CarStop();
            return "COMMAND=INVALID. STOPPED. USE O W A D S I K.\r\n";
    }
}

static void BluetoothSend(const char *text)
{
    UartWrite(WIFI_IOT_UART_IDX_1, (unsigned char *)text, strlen(text));
}

static int InitBluetoothUart(void)
{
    WifiIotUartAttribute uartAttr = {
        .baudRate = 9600,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    uint32_t result;

    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);
    result = UartInit(WIFI_IOT_UART_IDX_1, &uartAttr, NULL);
    if (result != 0U) {
        osMutexAcquire(g_printMutex, osWaitForever);
        printf("BLE initialization_failed error=0x%08X\r\n", result);
        osMutexRelease(g_printMutex);
        return -1;
    }
    PrintLine("BLE UART initialized baud=9600\r\n");
    return 0;
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
    unsigned char buffer[128];
    uint32_t helpElapsed = 0U;

    (void)argument;
    PrintLine("BLE ready baud=9600\r\n");
    BluetoothSend(g_bleHelpMessage);
    while (1) {
        uint32_t received = UartRead(WIFI_IOT_UART_IDX_1, buffer, sizeof(buffer) - 1U);
        uint32_t index;

        if (received == 0U) {
            ++helpElapsed;
            if (helpElapsed >= BLE_HELP_INTERVAL_TICKS) {
                BluetoothSend(g_bleHelpMessage);
                helpElapsed = 0U;
            }
            osDelay(1U);
            continue;
        }
        helpElapsed = 0U;
        if (received >= sizeof(buffer)) {
            PrintLine("BLE receive_failed invalid_length\r\n");
            continue;
        }
        for (index = 0U; index < received; ++index) {
            const char *response;

            if ((buffer[index] == '\r') || (buffer[index] == '\n')) {
                continue;
            }
            response = HandleBluetoothCommand(buffer[index]);
            BluetoothSend(response);
            osMutexAcquire(g_printMutex, osWaitForever);
            printf("BLE RX=%c %s", buffer[index], response);
            osMutexRelease(g_printMutex);
        }
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
    g_motorMutex = osMutexNew(NULL);
    if ((g_dataMutex == NULL) || (g_i2cMutex == NULL) || (g_printMutex == NULL) ||
        (g_motorMutex == NULL)) {
        printf("Mutex creation failed.\r\n");
        return;
    }

    ConfigureGpio();
    if (InitBluetoothUart() != 0) {
        return;
    }
    if (InitMotorUart() != 0) {
        return;
    }
    CarStop();
    PrintLine("SUM experiment started.\r\n");
    CreateTask("scan", ScanTask, osPriorityNormal);
    CreateTask("line_tracking", LineTrackingTask, osPriorityNormal);
    CreateTask("bluetooth", BluetoothTask, osPriorityNormal);
    CreateTask("motor_keepalive", MotorKeepaliveTask, osPriorityNormal);

    if (InitI2cDevices() != 0) {
        PrintLine("I2C device initialization failed. BLE control is available.\r\n");
        return;
    }
    CreateTask("environment", EnvironmentTask, osPriorityNormal);
    CreateTask("report", ReportTask, osPriorityNormal);
}

APP_FEATURE_INIT(SumExperimentFirst);
