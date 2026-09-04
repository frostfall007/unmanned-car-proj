#include <stdio.h>
#include <stdint.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "hi_io.h"
#include "hi_time.h"

/* --------------------------- Tunable parameters -------------------------- */
#define LINE_BLACK_LEVEL              WIFI_IOT_GPIO_VALUE1
#define NORMAL_LEFT_SPEED             58
#define NORMAL_RIGHT_SPEED            52
#define LEFT_CORRECTION_INNER_SPEED   30
#define LEFT_CORRECTION_OUTER_SPEED   60
#define RIGHT_REVERSE_LEFT_SPEED      25
#define RIGHT_REVERSE_RIGHT_SPEED     55
#define RIGHT_FORWARD_LEFT_SPEED      60
#define RIGHT_FORWARD_RIGHT_SPEED     40
#define MOTOR_MAX_SPEED               150

#define RIGHT_REVERSE_MAX_MS          1000UL
#define RIGHT_FORWARD_MIN_MS          150UL
#define TARGET_CONFIRM_SAMPLES        3U

#define SENSOR_POLL_TICKS             1U
#define SENSOR_STABLE_SAMPLES         2U
#define CONTROL_POLL_TICKS            2U
#define MOTOR_REFRESH_MS              200UL
#define REPORT_INTERVAL_MS            500UL

#define TCRT_LEFT_GPIO                WIFI_IOT_IO_NAME_GPIO_13
#define TCRT_RIGHT_GPIO               WIFI_IOT_IO_NAME_GPIO_14

/*
 * Normal tracking state: the left sensor is over black tape (LED OFF) and the
 * right sensor is over the white/background area (LED ON). This is right-edge
 * tracking and does not require white tape and floor to have different levels.
 */
typedef enum {
    FOLLOW_FORWARD = 0,
    FOLLOW_CORRECT_LEFT,
    FOLLOW_REVERSE_RIGHT,
    FOLLOW_FORWARD_RIGHT
} FollowAction;

typedef struct {
    WifiIotGpioValue leftRaw;
    WifiIotGpioValue rightRaw;
    uint8_t leftBlack;
    uint8_t rightBlack;
    uint8_t ready;
} LineSample;

static LineSample g_lineSample;
static osMutexId_t g_lineMutex;
static osMutexId_t g_motorMutex;
static osMutexId_t g_printMutex;

static unsigned long NowMilliseconds(void)
{
    return hi_get_us() / 1000UL;
}

static void PrintLine(const char *text)
{
    osMutexAcquire(g_printMutex, osWaitForever);
    printf("%s", text);
    osMutexRelease(g_printMutex);
}

static void ConfigureLineSensors(void)
{
    GpioInit();
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

static int ClampMotor(int value)
{
    if (value > MOTOR_MAX_SPEED) return MOTOR_MAX_SPEED;
    if (value < -MOTOR_MAX_SPEED) return -MOTOR_MAX_SPEED;
    return value;
}

static void MotorControl(int leftMotor, int rightMotor)
{
    unsigned char frame[6];

    leftMotor = ClampMotor(leftMotor);
    rightMotor = ClampMotor(rightMotor);
    frame[1] = (leftMotor < 0) ? 1U : 0U;
    frame[3] = (rightMotor < 0) ? 1U : 0U;
    if (leftMotor < 0) leftMotor = -leftMotor;
    if (rightMotor < 0) rightMotor = -rightMotor;

    frame[0] = 0xFCU;
    frame[2] = (unsigned char)leftMotor;
    frame[4] = (unsigned char)rightMotor;
    frame[5] = 0xFDU;

    osMutexAcquire(g_motorMutex, osWaitForever);
    UartWrite(WIFI_IOT_UART_IDX_2, frame, sizeof(frame));
    osMutexRelease(g_motorMutex);
}

static void LineSensorTask(void *argument)
{
    unsigned long lastReport = 0UL;
    WifiIotGpioValue candidateLeft = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue candidateRight = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue stableLeft = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue stableRight = WIFI_IOT_GPIO_VALUE0;
    uint8_t sameSamples = 0U;
    uint8_t filterReady = 0U;

    (void)argument;
    while (1) {
        LineSample sample;
        unsigned long now = NowMilliseconds();

        GpioGetInputVal(TCRT_LEFT_GPIO, &sample.leftRaw);
        GpioGetInputVal(TCRT_RIGHT_GPIO, &sample.rightRaw);
        if ((sample.leftRaw == candidateLeft) &&
            (sample.rightRaw == candidateRight)) {
            if (sameSamples < SENSOR_STABLE_SAMPLES) ++sameSamples;
        } else {
            candidateLeft = sample.leftRaw;
            candidateRight = sample.rightRaw;
            sameSamples = 1U;
        }
        if (sameSamples >= SENSOR_STABLE_SAMPLES) {
            stableLeft = candidateLeft;
            stableRight = candidateRight;
            filterReady = 1U;
        }
        sample.leftBlack = (stableLeft == LINE_BLACK_LEVEL) ? 1U : 0U;
        sample.rightBlack = (stableRight == LINE_BLACK_LEVEL) ? 1U : 0U;
        sample.ready = filterReady;

        osMutexAcquire(g_lineMutex, osWaitForever);
        g_lineSample = sample;
        osMutexRelease(g_lineMutex);

        if ((now - lastReport) >= REPORT_INTERVAL_MS) {
            osMutexAcquire(g_printMutex, osWaitForever);
            printf("LINE raw_left=%u raw_right=%u led_left=%s led_right=%s\r\n",
                sample.leftRaw, sample.rightRaw,
                (sample.leftBlack != 0U) ? "off" : "on",
                (sample.rightBlack != 0U) ? "off" : "on");
            osMutexRelease(g_printMutex);
            lastReport = now;
        }
        osDelay(SENSOR_POLL_TICKS);
    }
}

static void SetFollowingMotors(FollowAction action, int *leftMotor,
    int *rightMotor)
{
    if (action == FOLLOW_FORWARD) {
        *leftMotor = NORMAL_LEFT_SPEED;
        *rightMotor = NORMAL_RIGHT_SPEED;
    } else if (action == FOLLOW_REVERSE_RIGHT) {
        *leftMotor = -RIGHT_REVERSE_LEFT_SPEED;
        *rightMotor = -RIGHT_REVERSE_RIGHT_SPEED;
    } else if (action == FOLLOW_FORWARD_RIGHT) {
        *leftMotor = RIGHT_FORWARD_LEFT_SPEED;
        *rightMotor = RIGHT_FORWARD_RIGHT_SPEED;
    } else {
        *leftMotor = LEFT_CORRECTION_INNER_SPEED;
        *rightMotor = LEFT_CORRECTION_OUTER_SPEED;
    }
}

static void TrackingControlTask(void *argument)
{
    unsigned long actionStart = NowMilliseconds();
    unsigned long lastMotorSend = 0UL;
    int previousLeftMotor = 0;
    int previousRightMotor = 0;
    FollowAction activeAction = FOLLOW_FORWARD;
    FollowAction previousAction = FOLLOW_FORWARD;
    uint8_t actionReady = 0U;
    uint8_t targetSamples = 0U;
    uint8_t waitingMessagePrinted = 0U;

    (void)argument;
    PrintLine("LINE TRACKING started.\r\n");
    while (1) {
        LineSample sample;
        unsigned long now = NowMilliseconds();
        unsigned long actionElapsed = now - actionStart;
        int leftMotor = 0;
        int rightMotor = 0;
        FollowAction action = FOLLOW_FORWARD;

        osMutexAcquire(g_lineMutex, osWaitForever);
        sample = g_lineSample;
        osMutexRelease(g_lineMutex);

        if (sample.ready == 0U) {
            if (waitingMessagePrinted == 0U) {
                PrintLine("LINE TRACKING waiting for sensors.\r\n");
                waitingMessagePrinted = 1U;
            }
        } else {
            uint8_t targetState = ((sample.leftBlack != 0U) &&
                (sample.rightBlack == 0U)) ? 1U : 0U;

            if (activeAction == FOLLOW_FORWARD) {
                targetSamples = 0U;
                if (targetState == 0U) {
                    activeAction = (sample.rightBlack != 0U) ?
                        FOLLOW_REVERSE_RIGHT : FOLLOW_CORRECT_LEFT;
                    actionStart = now;
                }
            } else if (activeAction == FOLLOW_CORRECT_LEFT) {
                if (sample.rightBlack != 0U) {
                    activeAction = FOLLOW_REVERSE_RIGHT;
                    actionStart = now;
                    targetSamples = 0U;
                } else if (targetState != 0U) {
                    if (targetSamples < TARGET_CONFIRM_SAMPLES) ++targetSamples;
                    if (targetSamples >= TARGET_CONFIRM_SAMPLES) {
                        activeAction = FOLLOW_FORWARD;
                        actionStart = now;
                    }
                } else {
                    targetSamples = 0U;
                }
            } else if (activeAction == FOLLOW_REVERSE_RIGHT) {
                if (targetState != 0U) {
                    if (targetSamples < TARGET_CONFIRM_SAMPLES) ++targetSamples;
                } else {
                    targetSamples = 0U;
                }
                if ((targetSamples >= TARGET_CONFIRM_SAMPLES) ||
                    (actionElapsed >= RIGHT_REVERSE_MAX_MS)) {
                    activeAction = FOLLOW_FORWARD_RIGHT;
                    actionStart = now;
                    targetSamples = 0U;
                }
            } else {
                if (targetState != 0U) {
                    if (targetSamples < TARGET_CONFIRM_SAMPLES) ++targetSamples;
                } else {
                    targetSamples = 0U;
                }
                if ((actionElapsed >= RIGHT_FORWARD_MIN_MS) &&
                    (targetSamples >= TARGET_CONFIRM_SAMPLES)) {
                    activeAction = FOLLOW_FORWARD;
                    actionStart = now;
                }
            }
            action = activeAction;
            SetFollowingMotors(action, &leftMotor, &rightMotor);
            if ((actionReady == 0U) || (action != previousAction)) {
                if (action == FOLLOW_FORWARD) {
                    PrintLine("TRACK action=forward left_led=off right_led=on.\r\n");
                } else if (action == FOLLOW_CORRECT_LEFT) {
                    PrintLine("TRACK action=correct_left waiting_left_led_off.\r\n");
                } else if (action == FOLLOW_REVERSE_RIGHT) {
                    PrintLine("TRACK action=reverse_right waiting_left_off_right_on.\r\n");
                } else {
                    PrintLine("TRACK action=forward_right waiting_left_off_right_on.\r\n");
                }
                previousAction = action;
                actionReady = 1U;
            }
        }

        if ((leftMotor != previousLeftMotor) || (rightMotor != previousRightMotor) ||
            ((now - lastMotorSend) >= MOTOR_REFRESH_MS)) {
            MotorControl(leftMotor, rightMotor);
            previousLeftMotor = leftMotor;
            previousRightMotor = rightMotor;
            lastMotorSend = now;
        }
        osDelay(CONTROL_POLL_TICKS);
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

static void EdgeLineTrackingEntry(void)
{
    g_lineMutex = osMutexNew(NULL);
    g_motorMutex = osMutexNew(NULL);
    g_printMutex = osMutexNew(NULL);
    if ((g_lineMutex == NULL) || (g_motorMutex == NULL) || (g_printMutex == NULL)) {
        printf("Mutex creation failed.\r\n");
        return;
    }

    ConfigureLineSensors();
    if (InitMotorUart() != 0) return;
    MotorControl(0, 0);
    PrintLine("LINE TRACKING ready. Put the left sensor over the black line.\r\n");
    PrintLine("EXPECTED state: left LED off, right LED on.\r\n");
    CreateTask("line_sensor", LineSensorTask);
    CreateTask("line_control", TrackingControlTask);
}

APP_FEATURE_INIT(EdgeLineTrackingEntry);
