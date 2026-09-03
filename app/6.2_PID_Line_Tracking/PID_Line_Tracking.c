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
#define NORMAL_SPEED                  75
#define MARKER_SCAN_SPEED             50
#define BACKTRACK_SPEED               50
#define BRANCH_TURN_SPEED             65
#define MOTOR_MAX_SPEED               150

#define PID_KP                        35.0f
#define PID_KI                        0.20f
#define PID_KD                        22.0f
#define PID_INTEGRAL_LIMIT            20.0f

#define END_GAP_MIN_MS                100UL
#define END_GAP_MAX_MS                1000UL
#define END_BAR_CONFIRM_SAMPLES       3U
#define DEAD_END_STOP_MS              200UL
#define BACKTRACK_IGNORE_MS           500UL
#define BACKTRACK_TIMEOUT_MS          15000UL
#define BRANCH_CONFIRM_SAMPLES        4U
#define BRANCH_TURN_MIN_MS            150UL
#define BRANCH_TURN_MAX_MS            1800UL
#define CENTER_CONFIRM_SAMPLES        3U

#define SENSOR_POLL_TICKS             1U
#define CONTROL_POLL_TICKS            2U
#define MOTOR_REFRESH_MS              200UL
#define REPORT_INTERVAL_MS            500UL

#define TCRT_LEFT_GPIO                WIFI_IOT_IO_NAME_GPIO_13
#define TCRT_RIGHT_GPIO               WIFI_IOT_IO_NAME_GPIO_14

/*
 * The black line must stay between the two sensors while the car is centered.
 * Both sensor indicator LEDs should be ON in the centered position. The raw
 * level over black tape must equal LINE_BLACK_LEVEL.
 */
typedef enum {
    TRACK_FOLLOW = 0,
    TRACK_CHECK_MARKER,
    TRACK_DEAD_END_STOP,
    TRACK_BACKTRACK,
    TRACK_BRANCH_TURN,
    TRACK_FINISHED
} TrackState;

typedef struct {
    WifiIotGpioValue leftRaw;
    WifiIotGpioValue rightRaw;
    uint8_t leftBlack;
    uint8_t rightBlack;
    uint8_t ready;
} LineSample;

typedef struct {
    float integral;
    float previousError;
} PidController;

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

    (void)argument;
    while (1) {
        LineSample sample;
        unsigned long now = NowMilliseconds();

        GpioGetInputVal(TCRT_LEFT_GPIO, &sample.leftRaw);
        GpioGetInputVal(TCRT_RIGHT_GPIO, &sample.rightRaw);
        sample.leftBlack = (sample.leftRaw == LINE_BLACK_LEVEL) ? 1U : 0U;
        sample.rightBlack = (sample.rightRaw == LINE_BLACK_LEVEL) ? 1U : 0U;
        sample.ready = 1U;

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

static void ResetPid(PidController *pid)
{
    pid->integral = 0.0f;
    pid->previousError = 0.0f;
}

static void CalculatePidMotors(PidController *pid, const LineSample *sample,
    int *leftMotor, int *rightMotor)
{
    float error;
    float correction;

    if ((sample->leftBlack != 0U) && (sample->rightBlack == 0U)) {
        error = -1.0f;
    } else if ((sample->rightBlack != 0U) && (sample->leftBlack == 0U)) {
        error = 1.0f;
    } else {
        error = 0.0f;
    }

    pid->integral += error;
    if (pid->integral > PID_INTEGRAL_LIMIT) pid->integral = PID_INTEGRAL_LIMIT;
    if (pid->integral < -PID_INTEGRAL_LIMIT) pid->integral = -PID_INTEGRAL_LIMIT;
    correction = PID_KP * error + PID_KI * pid->integral +
        PID_KD * (error - pid->previousError);
    pid->previousError = error;

    *leftMotor = ClampMotor(NORMAL_SPEED + (int)correction);
    *rightMotor = ClampMotor(NORMAL_SPEED - (int)correction);
}

static void TrackingControlTask(void *argument)
{
    TrackState state = TRACK_FOLLOW;
    PidController pid = {0.0f, 0.0f};
    unsigned long stateStart = NowMilliseconds();
    unsigned long lastMotorSend = 0UL;
    unsigned int secondBarSamples = 0U;
    unsigned int branchSamples = 0U;
    unsigned int centeredSamples = 0U;
    int previousLeftMotor = 0;
    int previousRightMotor = 0;
    int8_t branchDirection = 0;
    uint8_t markerGapSeen = 0U;
    uint8_t waitingMessagePrinted = 0U;

    (void)argument;
    PrintLine("LINE TRACKING started.\r\n");
    while (1) {
        LineSample sample;
        unsigned long now = NowMilliseconds();
        unsigned long elapsed = now - stateStart;
        int leftMotor = 0;
        int rightMotor = 0;
        uint8_t bothBlack;
        uint8_t bothWhite;

        osMutexAcquire(g_lineMutex, osWaitForever);
        sample = g_lineSample;
        osMutexRelease(g_lineMutex);
        bothBlack = ((sample.leftBlack != 0U) && (sample.rightBlack != 0U));
        bothWhite = ((sample.leftBlack == 0U) && (sample.rightBlack == 0U));

        if (sample.ready == 0U) {
            if (waitingMessagePrinted == 0U) {
                PrintLine("LINE TRACKING waiting for sensors.\r\n");
                waitingMessagePrinted = 1U;
            }
        } else if (state == TRACK_FOLLOW) {
            if (bothBlack != 0U) {
                state = TRACK_CHECK_MARKER;
                stateStart = now;
                secondBarSamples = 0U;
                markerGapSeen = 0U;
                ResetPid(&pid);
                leftMotor = MARKER_SCAN_SPEED;
                rightMotor = MARKER_SCAN_SPEED;
                PrintLine("MARKER possible_end slowing_down.\r\n");
            } else {
                CalculatePidMotors(&pid, &sample, &leftMotor, &rightMotor);
            }
        } else if (state == TRACK_CHECK_MARKER) {
            leftMotor = MARKER_SCAN_SPEED;
            rightMotor = MARKER_SCAN_SPEED;
            if (bothWhite != 0U) {
                markerGapSeen = 1U;
                secondBarSamples = 0U;
            } else if ((bothBlack == 0U) && (markerGapSeen == 0U)) {
                state = TRACK_FOLLOW;
                stateStart = now;
                CalculatePidMotors(&pid, &sample, &leftMotor, &rightMotor);
                PrintLine("MARKER cancelled line_correction.\r\n");
            } else if ((bothBlack != 0U) && (markerGapSeen != 0U) &&
                (elapsed >= END_GAP_MIN_MS)) {
                ++secondBarSamples;
                if (secondBarSamples >= END_BAR_CONFIRM_SAMPLES) {
                    state = TRACK_FINISHED;
                    leftMotor = 0;
                    rightMotor = 0;
                    PrintLine("FINISH double_bar_detected car_stopped.\r\n");
                }
            } else {
                secondBarSamples = 0U;
            }

            if ((state == TRACK_CHECK_MARKER) && (elapsed >= END_GAP_MAX_MS)) {
                state = TRACK_DEAD_END_STOP;
                stateStart = now;
                leftMotor = 0;
                rightMotor = 0;
                PrintLine("DEAD_END single_bar_detected.\r\n");
            }
        } else if (state == TRACK_DEAD_END_STOP) {
            if (elapsed >= DEAD_END_STOP_MS) {
                state = TRACK_BACKTRACK;
                stateStart = now;
                branchSamples = 0U;
                branchDirection = 0;
                PrintLine("DEAD_END action=backtrack.\r\n");
            }
        } else if (state == TRACK_BACKTRACK) {
            leftMotor = -BACKTRACK_SPEED;
            rightMotor = -BACKTRACK_SPEED;
            if (elapsed >= BACKTRACK_IGNORE_MS) {
                if ((sample.leftBlack != 0U) && (sample.rightBlack == 0U)) {
                    if (branchDirection == -1) {
                        ++branchSamples;
                    } else {
                        branchDirection = -1;
                        branchSamples = 1U;
                    }
                } else if ((sample.rightBlack != 0U) && (sample.leftBlack == 0U)) {
                    if (branchDirection == 1) {
                        ++branchSamples;
                    } else {
                        branchDirection = 1;
                        branchSamples = 1U;
                    }
                } else {
                    branchDirection = 0;
                    branchSamples = 0U;
                }
            }

            if (branchSamples >= BRANCH_CONFIRM_SAMPLES) {
                state = TRACK_BRANCH_TURN;
                stateStart = now;
                centeredSamples = 0U;
                leftMotor = 0;
                rightMotor = 0;
                PrintLine((branchDirection < 0) ?
                    "JUNCTION detected_side=left.\r\n" :
                    "JUNCTION detected_side=right.\r\n");
            } else if (elapsed >= BACKTRACK_TIMEOUT_MS) {
                state = TRACK_FINISHED;
                leftMotor = 0;
                rightMotor = 0;
                PrintLine("BACKTRACK junction_not_found car_stopped.\r\n");
            }
        } else if (state == TRACK_BRANCH_TURN) {
            if (branchDirection < 0) {
                leftMotor = -BRANCH_TURN_SPEED;
                rightMotor = BRANCH_TURN_SPEED;
            } else {
                leftMotor = BRANCH_TURN_SPEED;
                rightMotor = -BRANCH_TURN_SPEED;
            }

            if ((elapsed >= BRANCH_TURN_MIN_MS) && (bothWhite != 0U)) {
                ++centeredSamples;
            } else {
                centeredSamples = 0U;
            }
            if (centeredSamples >= CENTER_CONFIRM_SAMPLES) {
                state = TRACK_FOLLOW;
                stateStart = now;
                ResetPid(&pid);
                leftMotor = NORMAL_SPEED;
                rightMotor = NORMAL_SPEED;
                PrintLine("JUNCTION turn_complete line_centered.\r\n");
            } else if (elapsed >= BRANCH_TURN_MAX_MS) {
                state = TRACK_FINISHED;
                leftMotor = 0;
                rightMotor = 0;
                PrintLine("JUNCTION turn_timeout car_stopped.\r\n");
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

static void PidLineTrackingEntry(void)
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
    PrintLine("LINE TRACKING ready. Put the black line between both sensors.\r\n");
    CreateTask("line_sensor", LineSensorTask);
    CreateTask("line_control", TrackingControlTask);
}

APP_FEATURE_INIT(PidLineTrackingEntry);
