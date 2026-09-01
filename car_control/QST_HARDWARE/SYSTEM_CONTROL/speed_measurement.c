#include "control_system.h"
#include "car_protocol.h"
#include "car_lights.h"
#include "encoder.h"
#include "motor.h"

#define SPEED_SAMPLE_PERIOD_MS  100U
#define COMMAND_TIMEOUT_MS      1500U
#define ENCODER_PULSES_PER_REV  2800.0f
#define MOTOR_PWM_LIMIT         7199.0f
#define REVERSE_PWM_FEEDFORWARD 1200

#define LEFT_KP                 4.0f
#define LEFT_KI                 0.35f
#define RIGHT_KP                4.0f
#define RIGHT_KI                0.35f

static volatile uint32_t millis = 0;
static volatile uint16_t command_age_ms = COMMAND_TIMEOUT_MS;
static volatile u8 control_due = 0;
static float left_target_rps = 0.0f;
static float right_target_rps = 0.0f;
static float left_pwm = 0.0f;
static float right_pwm = 0.0f;
static float left_previous_error = 0.0f;
static float right_previous_error = 0.0f;

static int Rps_To_Counts(float rps)
{
    return (int)(rps * ENCODER_PULSES_PER_REV
                 * ((float)SPEED_SAMPLE_PERIOD_MS / 1000.0f));
}

static int Add_Reverse_Feedforward(int pwm, int target_count)
{
    if (target_count < 0)
    {
        pwm -= REVERSE_PWM_FEEDFORWARD;
        if (pwm < -(int)MOTOR_PWM_LIMIT)
        {
            pwm = -(int)MOTOR_PWM_LIMIT;
        }
    }
    return pwm;
}

int Incremental_PI_A(int encoder_count, int target_count)
{
    float error = (float)(target_count - encoder_count);

    left_pwm += LEFT_KP * (error - left_previous_error) + LEFT_KI * error;
    if (left_pwm > MOTOR_PWM_LIMIT) left_pwm = MOTOR_PWM_LIMIT;
    if (left_pwm < -MOTOR_PWM_LIMIT) left_pwm = -MOTOR_PWM_LIMIT;

    left_previous_error = error;
    return (int)left_pwm;
}

int Incremental_PI_B(int encoder_count, int target_count)
{
    float error = (float)(target_count - encoder_count);

    right_pwm += RIGHT_KP * (error - right_previous_error) + RIGHT_KI * error;
    if (right_pwm > MOTOR_PWM_LIMIT) right_pwm = MOTOR_PWM_LIMIT;
    if (right_pwm < -MOTOR_PWM_LIMIT) right_pwm = -MOTOR_PWM_LIMIT;

    right_previous_error = error;
    return (int)right_pwm;
}

void System_Control_Reset(void)
{
    left_pwm = 0.0f;
    right_pwm = 0.0f;
    left_previous_error = 0.0f;
    right_previous_error = 0.0f;
}

static float Decode_Speed(u8 direction, u8 speed)
{
    float target;

    if (speed > CAR_SPEED_MAX)
    {
        speed = CAR_SPEED_MAX;
    }
    target = (float)speed / 100.0f;
    return (direction == 0U) ? target : -target;
}

static int Direction_Changed(float old_target, float new_target)
{
    return ((old_target > 0.0f) && (new_target < 0.0f))
        || ((old_target < 0.0f) && (new_target > 0.0f));
}

static float Absolute(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void Update_Lights(void)
{
    if (left_target_rps == 0.0f && right_target_rps == 0.0f)
    {
        L_led_off();
        R_led_CLC();
    }
    else if (left_target_rps < 0.0f && right_target_rps < 0.0f)
    {
        L_led_off();
        R_led_mode();
    }
    else if (left_target_rps >= 0.0f && right_target_rps >= 0.0f)
    {
        R_led_CLC();
        if (Absolute(left_target_rps) < Absolute(right_target_rps))
        {
            L_led_left_turn();
        }
        else if (Absolute(left_target_rps) > Absolute(right_target_rps))
        {
            L_led_right_turn();
        }
        else
        {
            L_led_forward();
        }
    }
}

static void Apply_Command(void)
{
    float new_left_target;
    float new_right_target;

    if (uart_rec_flag == 0U)
    {
        return;
    }

    new_left_target = Decode_Speed(CAR_buff[0], CAR_buff[1]);
    new_right_target = Decode_Speed(CAR_buff[2], CAR_buff[3]);
    if (Direction_Changed(left_target_rps, new_left_target)
        || Direction_Changed(right_target_rps, new_right_target))
    {
        System_Control_Reset();
    }
    left_target_rps = new_left_target;
    right_target_rps = new_right_target;
    uart_rec_flag = 0U;
    command_age_ms = 0U;
}

void System_Control(void)
{
    int left_count;
    int right_count;
    int left_target;
    int right_target;
    int left_output;
    int right_output;

    if (control_due == 0U)
    {
        return;
    }
    control_due = 0U;
    Apply_Command();

    if (command_age_ms >= COMMAND_TIMEOUT_MS)
    {
        left_target_rps = 0.0f;
        right_target_rps = 0.0f;
        System_Control_Reset();
        Set_Pwm(0, 0);
        Update_Lights();
        return;
    }

    if ((left_target_rps == 0.0f) && (right_target_rps == 0.0f))
    {
        System_Control_Reset();
        Set_Pwm(0, 0);
        Update_Lights();
        return;
    }

    left_count = Read_Encoder(2);
    right_count = Read_Encoder(3);
    left_target = Rps_To_Counts(left_target_rps);
    right_target = Rps_To_Counts(right_target_rps);
    left_output = Incremental_PI_A(left_count, left_target);
    right_output = Incremental_PI_B(right_count, right_target);
    left_output = Add_Reverse_Feedforward(left_output, left_target);
    right_output = Add_Reverse_Feedforward(right_output, right_target);

    Set_Pwm(left_output, right_output);
    Update_Lights();
    printf("L:%d/%d R:%d/%d PWM:%d,%d\r\n",
           left_count, left_target, right_count, right_target,
           left_output, right_output);
}

void System_Control_Tick(void)
{
    ++millis;
    if (command_age_ms < COMMAND_TIMEOUT_MS)
    {
        ++command_age_ms;
    }
    if (millis >= SPEED_SAMPLE_PERIOD_MS)
    {
        millis = 0;
        control_due = 1U;
    }
}
