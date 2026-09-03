#include <stdio.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "hi_io.h"
#include "hi_time.h"

/* --------------------------- Tunable parameters -------------------------- */
#define OBSTACLE_DISTANCE_CM          20.0f
#define FORWARD_SPEED                 100
#define REVERSE_SPEED                 80
#define TURN_INNER_SPEED              45
#define TURN_OUTER_SPEED              120
#define STOP_TIME_MS                  200UL
#define REVERSE_TIME_MS               4000UL
#define TURN_TIME_MS                  1200UL
#define MOTOR_REFRESH_MS              200UL
#define SENSOR_POLL_TICKS             5U
#define MOTOR_POLL_TICKS              2U
#define ULTRASONIC_TIMEOUT_US         30000UL
#define ULTRASONIC_TIMEOUT_IS_OBSTACLE 1U
#define SERVO_FORWARD_ANGLE           90U
#define TCRT_TRIGGER_SAMPLES          2U

/* Set this to the raw level printed when a sensor is over the exposed white tape. */
#define TCRT_BOUNDARY_LEVEL WIFI_IOT_GPIO_VALUE1

#define SERVO_GPIO       WIFI_IOT_IO_NAME_GPIO_2
#define HCSR_TRIG_GPIO   WIFI_IOT_IO_NAME_GPIO_7
#define HCSR_ECHO_GPIO   WIFI_IOT_IO_NAME_GPIO_8
#define TCRT_LEFT_GPIO   WIFI_IOT_IO_NAME_GPIO_13
#define TCRT_RIGHT_GPIO  WIFI_IOT_IO_NAME_GPIO_14

#define SERVO_PULSE_MIN_US 500U
#define SERVO_PULSE_MAX_US 2500U
#define SERVO_PERIOD_US    20000U
#define SERVO_PULSE_COUNT  25U

typedef enum {
    DRIVE_FORWARD = 0,
    DRIVE_STOP,
    DRIVE_REVERSE,
    DRIVE_TURN
} DriveState;

typedef enum {
    TURN_LEFT = 0,
    TURN_RIGHT
} TurnDirection;

typedef struct {
    float distanceCm;
    WifiIotGpioValue leftRaw;
    WifiIotGpioValue rightRaw;
    uint8_t sampleReady;
    uint8_t distanceValid;
    uint8_t boundaryLeft;
    uint8_t boundaryRight;
} AvoidanceSensors;

static AvoidanceSensors g_sensors;
static osMutexId_t g_sensorMutex;
static osMutexId_t g_motorMutex;
static osMutexId_t g_printMutex;

static void PrintLine(const char *text)
{
    osMutexAcquire(g_printMutex, osWaitForever);
    printf("%s", text);
    osMutexRelease(g_printMutex);
}

static unsigned long NowMilliseconds(void)
{
    return hi_get_us() / 1000UL;
}

static void ConfigureHardware(void)
{
    GpioInit();

    IoSetFunc(SERVO_GPIO, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(SERVO_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(SERVO_GPIO, WIFI_IOT_GPIO_VALUE0);

    IoSetFunc(HCSR_TRIG_GPIO, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(HCSR_ECHO_GPIO, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(HCSR_TRIG_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(HCSR_TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);
    GpioSetDir(HCSR_ECHO_GPIO, WIFI_IOT_GPIO_DIR_IN);

    IoSetFunc(TCRT_LEFT_GPIO, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(TCRT_RIGHT_GPIO, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
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

static void MotorControl(int leftMotor, int rightMotor)
{
    unsigned char frame[6];

    frame[1] = (leftMotor < 0) ? 1U : 0U;
    frame[3] = (rightMotor < 0) ? 1U : 0U;
    if (leftMotor < 0) leftMotor = -leftMotor;
    if (rightMotor < 0) rightMotor = -rightMotor;
    if (leftMotor > 150) leftMotor = 150;
    if (rightMotor > 150) rightMotor = 150;

    frame[0] = 0xFCU;
    frame[2] = (unsigned char)leftMotor;
    frame[4] = (unsigned char)rightMotor;
    frame[5] = 0xFDU;

    osMutexAcquire(g_motorMutex, osWaitForever);
    UartWrite(WIFI_IOT_UART_IDX_2, frame, sizeof(frame));
    osMutexRelease(g_motorMutex);
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

static float ReadUltrasonicDistance(void)
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
        if ((hi_get_us() - start) > ULTRASONIC_TIMEOUT_US) return -1.0f;
    } while (value == WIFI_IOT_GPIO_VALUE0);

    start = hi_get_us();
    do {
        GpioGetInputVal(HCSR_ECHO_GPIO, &value);
        elapsed = hi_get_us() - start;
        if (elapsed > ULTRASONIC_TIMEOUT_US) return -1.0f;
    } while (value == WIFI_IOT_GPIO_VALUE1);

    return (float)elapsed * 0.034f / 2.0f;
}

static void SensorTask(void *argument)
{
    unsigned long lastReport = 0UL;
    uint8_t leftMarkerSamples = 0U;
    uint8_t rightMarkerSamples = 0U;

    (void)argument;
    while (1) {
        AvoidanceSensors sample = {0};
        unsigned long now = NowMilliseconds();

        sample.distanceCm = ReadUltrasonicDistance();
        sample.distanceValid = (sample.distanceCm >= 0.0f) ? 1U : 0U;
        GpioGetInputVal(TCRT_LEFT_GPIO, &sample.leftRaw);
        GpioGetInputVal(TCRT_RIGHT_GPIO, &sample.rightRaw);
        if (sample.leftRaw == TCRT_BOUNDARY_LEVEL) {
            if (leftMarkerSamples < TCRT_TRIGGER_SAMPLES) ++leftMarkerSamples;
        } else {
            leftMarkerSamples = 0U;
        }
        if (sample.rightRaw == TCRT_BOUNDARY_LEVEL) {
            if (rightMarkerSamples < TCRT_TRIGGER_SAMPLES) ++rightMarkerSamples;
        } else {
            rightMarkerSamples = 0U;
        }
        sample.boundaryLeft = (leftMarkerSamples >= TCRT_TRIGGER_SAMPLES) ? 1U : 0U;
        sample.boundaryRight = (rightMarkerSamples >= TCRT_TRIGGER_SAMPLES) ? 1U : 0U;
        sample.sampleReady = 1U;

        osMutexAcquire(g_sensorMutex, osWaitForever);
        g_sensors = sample;
        osMutexRelease(g_sensorMutex);

        if ((now - lastReport) >= 1000UL) {
            osMutexAcquire(g_printMutex, osWaitForever);
            if (sample.distanceValid != 0U) {
                printf("SENSOR distance=%.1fcm tcrt_left=%u tcrt_right=%u boundary_left=%u boundary_right=%u\r\n",
                    sample.distanceCm, sample.leftRaw, sample.rightRaw,
                    sample.boundaryLeft, sample.boundaryRight);
            } else {
                printf("SENSOR distance=timeout tcrt_left=%u tcrt_right=%u boundary_left=%u boundary_right=%u\r\n",
                    sample.leftRaw, sample.rightRaw, sample.boundaryLeft, sample.boundaryRight);
            }
            osMutexRelease(g_printMutex);
            lastReport = now;
        }
        osDelay(SENSOR_POLL_TICKS);
    }
}

static TurnDirection SelectTurnDirection(const AvoidanceSensors *sensors,
    TurnDirection fallback)
{
    if ((sensors->boundaryLeft != 0U) && (sensors->boundaryRight == 0U)) return TURN_RIGHT;
    if ((sensors->boundaryRight != 0U) && (sensors->boundaryLeft == 0U)) return TURN_LEFT;
    return fallback;
}

static void MotorTask(void *argument)
{
    DriveState state = DRIVE_FORWARD;
    TurnDirection turn = TURN_RIGHT;
    unsigned long stateStart = NowMilliseconds();
    unsigned long lastMotorSend = 0UL;
    int lastLeftMotor = 0;
    int lastRightMotor = 0;
    uint8_t waitingForSensor = 1U;

    (void)argument;
    PrintLine("AUTO avoidance started.\r\n");
    while (1) {
        AvoidanceSensors sensors;
        unsigned long now = NowMilliseconds();
        int leftMotor = 0;
        int rightMotor = 0;

        osMutexAcquire(g_sensorMutex, osWaitForever);
        sensors = g_sensors;
        osMutexRelease(g_sensorMutex);

        if (sensors.sampleReady == 0U) {
            if (waitingForSensor != 0U) {
                PrintLine("AUTO waiting for first sensor sample.\r\n");
                waitingForSensor = 0U;
            }
        } else if (state == DRIVE_FORWARD) {
            if ((sensors.boundaryLeft != 0U) || (sensors.boundaryRight != 0U)) {
                turn = SelectTurnDirection(&sensors, turn);
                state = DRIVE_STOP;
                stateStart = now;
                PrintLine("AVOID reason=boundary_marker.\r\n");
            } else if (((sensors.distanceValid != 0U) &&
                (sensors.distanceCm <= OBSTACLE_DISTANCE_CM)) ||
                ((sensors.distanceValid == 0U) && (ULTRASONIC_TIMEOUT_IS_OBSTACLE != 0U))) {
                turn = (turn == TURN_LEFT) ? TURN_RIGHT : TURN_LEFT;
                state = DRIVE_STOP;
                stateStart = now;
                PrintLine("AVOID reason=obstacle.\r\n");
            }
        } else if ((state == DRIVE_STOP) && ((now - stateStart) >= STOP_TIME_MS)) {
            state = DRIVE_REVERSE;
            stateStart = now;
            PrintLine("AVOID action=reverse.\r\n");
        } else if ((state == DRIVE_REVERSE) && ((now - stateStart) >= REVERSE_TIME_MS)) {
            state = DRIVE_TURN;
            stateStart = now;
            PrintLine((turn == TURN_LEFT) ? "AVOID action=turn_left.\r\n" :
                "AVOID action=turn_right.\r\n");
        } else if ((state == DRIVE_TURN) && ((now - stateStart) >= TURN_TIME_MS)) {
            state = DRIVE_FORWARD;
            stateStart = now;
            PrintLine("AVOID action=forward.\r\n");
        }

        if ((sensors.sampleReady != 0U) && (state == DRIVE_FORWARD)) {
            leftMotor = FORWARD_SPEED;
            rightMotor = FORWARD_SPEED;
        } else if ((sensors.sampleReady != 0U) && (state == DRIVE_REVERSE)) {
            leftMotor = -REVERSE_SPEED;
            rightMotor = -REVERSE_SPEED;
        } else if ((sensors.sampleReady != 0U) && (state == DRIVE_TURN)) {
            if (turn == TURN_LEFT) {
                leftMotor = TURN_INNER_SPEED;
                rightMotor = TURN_OUTER_SPEED;
            } else {
                leftMotor = TURN_OUTER_SPEED;
                rightMotor = TURN_INNER_SPEED;
            }
        }

        if ((leftMotor != lastLeftMotor) || (rightMotor != lastRightMotor) ||
            ((now - lastMotorSend) >= MOTOR_REFRESH_MS)) {
            MotorControl(leftMotor, rightMotor);
            lastLeftMotor = leftMotor;
            lastRightMotor = rightMotor;
            lastMotorSend = now;
        }
        osDelay(MOTOR_POLL_TICKS);
    }
}

static void CreateTask(const char *name, osThreadFunc_t entry)
{
    osThreadAttr_t attr = {
        .name = name,
        .stack_size = 1024U * 4U,
        .priority = osPriorityNormal,
    };

    if (osThreadNew(entry, NULL, &attr) == NULL) {
        osMutexAcquire(g_printMutex, osWaitForever);
        printf("Task creation_failed name=%s\r\n", name);
        osMutexRelease(g_printMutex);
    }
}

static void AutoAvoidanceEntry(void)
{
    g_sensorMutex = osMutexNew(NULL);
    g_motorMutex = osMutexNew(NULL);
    g_printMutex = osMutexNew(NULL);
    if ((g_sensorMutex == NULL) || (g_motorMutex == NULL) || (g_printMutex == NULL)) {
        printf("Mutex creation failed.\r\n");
        return;
    }

    ConfigureHardware();
    if (InitMotorUart() != 0) return;
    ServoMoveTo(SERVO_FORWARD_ANGLE);
    MotorControl(0, 0);

    PrintLine("AUTO avoidance ready.\r\n");
    CreateTask("sensor", SensorTask);
    CreateTask("motor", MotorTask);
}

APP_FEATURE_INIT(AutoAvoidanceEntry);
