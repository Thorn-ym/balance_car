#include "balance_car/balance_control.h"

#include "balance_car/app_sensors.h"
#include "balance_car/encoder_hal.h"
#include "balance_car/motor_tb6612.h"
#include "balance_car/mpu6050_hal.h"
#include "balance_car/raspi_link.h"
#include "balance_car/remote_control.h"
#include <math.h>

#define PI_F                         3.14159f
#define ANGLE_LOOP_PERIOD_S          0.01f
#define SPEED_LOOP_PERIOD_S          0.05f
#define ENCODER_MAGNET_LINES         13.0f
#define MOTOR_REDUCTION_RATIO        30.0f
#define RUN_LED_PORT                 GPIOC
#define RUN_LED_PIN                  GPIO_PIN_13
#define RUN_LED_ON                   GPIO_PIN_RESET
#define RUN_LED_OFF                  GPIO_PIN_SET

volatile BalanceCarDebug_t g_balance_debug = {
    .run_enable = 0U,          /* Ozone: 默认停机；确认车架空和传感器正常后再改为1 */
    .reset_pid_request = 0U,   /* Ozone: 写1可清PID历史，适合每次重新调参前使用 */
    .clear_fault_request = 0U, /* Ozone: 写1可清故障标志，硬件问题没修好会再次置位 */
    .speed_target = 0.0f,      /* Ozone: 调平衡阶段保持0；速度环稳定后再小幅给目标 */
    .turn_target = 0.0f,       /* Ozone: 调平衡阶段保持0；转向环稳定后再小幅给目标 */
    .gyro_y_offset = 20,    /* Ozone: 静止时观察g_balance_state.gy，把零漂填到这里 */
    .gyro_z_offset = -60.0f,     /* Ozone: 静止时观察g_balance_state.gz，把Z轴零漂填到这里 */
    .turn_gyro_scale = 60.0f,  /* Ozone: turn_target=1时目标Z轴角速度约60deg/s，可按手感调 */
    .angle_offset = 4.5f,      /* Ozone: 竖直时调这个，让g_balance_state.angle接近0 */
    .fall_angle_limit = 40.0f, /* Ozone: 倒车保护阈值，超过后自动停机 */
};

volatile BalanceCarState_t g_balance_state = {0}; /* Ozone: 运行状态观察区，不建议手动修改 */

PID_t g_angle_pid = {       /* Ozone: 角度环，第一阶段只调这个 */
    .Kp = 12.0f,             /* 比例: 第一次架空可先降到1.0测试方向 */
    .Ki = 0.25f,             /* 积分: 初调可先设0，最后再少量加 */
    .Kd = 8.0f,             /* 微分: 抑制摆动，Kp方向正确后再加 */
    .OutMax = 100.0f,
    .OutMin = -100.0f,
    .OutOffset = 3.0f,      /* 电机死区补偿；初调可先设0 */
    .ErrorIntMax = 600.0f,
    .ErrorIntMin = -600.0f,
};

PID_t g_speed_pid = {       /* Ozone: 速度环，角度环稳定后再调 */
    .Kp = 0.60f,
    .Ki = 0.07f, 
    .Kd = 0,
    .OutMax = 5.0f,
    .OutMin = -5.0f,
    .ErrorIntMax = 150.0f,
    .ErrorIntMin = -150.0f,
};

PID_t g_turn_pid = {        /* Ozone: 转向环，速度环稳定后最后调 */
    .Kp = 3,
    .Ki = 0,
    .Kd = 0.0f,
    .OutMax = 5.0f,
    .OutMin = -5.0f,
    .ErrorIntMax = 20.0f,
    .ErrorIntMin = -20.0f,
};

static TIM_HandleTypeDef s_htim4;
static volatile uint8_t s_angle_tick_pending;
static volatile uint8_t s_speed_tick_pending;
static uint8_t s_last_run_enable;
static float s_angle;
static float s_dif_pwm;

static void BalanceCar_RunLedInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = RUN_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RUN_LED_PORT, &gpio);
    HAL_GPIO_WritePin(RUN_LED_PORT, RUN_LED_PIN, RUN_LED_OFF);
}

static void BalanceCar_RunLedApply(void)
{
    GPIO_PinState led_state = (g_balance_debug.run_enable != 0U) ? RUN_LED_ON : RUN_LED_OFF;
    HAL_GPIO_WritePin(RUN_LED_PORT, RUN_LED_PIN, led_state);
}

static void BalanceCar_SetFault(uint32_t fault)
{
    g_balance_state.fault_flags |= fault;
}

static void BalanceCar_ClearRuntimeFaults(void)
{
    g_balance_state.fault_flags &= ~(uint32_t)(BALANCE_FAULT_MPU_READ |
                                              BALANCE_FAULT_FALL |
                                              BALANCE_FAULT_TIMER_OVERRUN);
    g_balance_state.timer_error_flag = 0U;
}

static void BalanceCar_ButtonInit(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    BalanceCar_RunLedInit();

    g_balance_state.button_toggle = 0U;
    g_balance_state.button_raw = 0U;
    g_balance_state.button_stable = 0U;
}

static void BalanceCar_ButtonTick1ms(void)
{
    BalanceCar_RunLedApply();
}

static void BalanceCar_ResetPidAndOutputs(void)
{
    PID_Init(&g_angle_pid);
    PID_Init(&g_speed_pid);
    PID_Init(&g_turn_pid);
    s_dif_pwm = 0.0f;
    g_balance_state.left_pwm = 0;
    g_balance_state.right_pwm = 0;
    g_balance_state.ave_pwm = 0;
    g_balance_state.dif_pwm = 0;
}

static int16_t BalanceCar_ClampPwm(float value)
{
    if (value > (float)MOTOR_PWM_LIMIT) {
        return MOTOR_PWM_LIMIT;
    }
    if (value < (float)-MOTOR_PWM_LIMIT) {
        return -MOTOR_PWM_LIMIT;
    }
    return (int16_t)value;
}

static void BalanceCar_Stop(void)
{
    g_balance_state.run_active = 0U;
    g_balance_state.left_pwm = 0;
    g_balance_state.right_pwm = 0;
    g_balance_state.ave_pwm = 0;
    g_balance_state.dif_pwm = 0;
    TB6612_Stop();
}

static void BalanceCar_RunAngleLoop(void)
{
    Mpu6050Raw_t raw;

    if (Mpu6050_ReadRaw(&raw) != HAL_OK) {
        BalanceCar_SetFault(BALANCE_FAULT_MPU_READ);
        g_balance_state.imu_ready = 0U;
        BalanceCar_Stop();
        return;
    }

    g_balance_state.imu_ready = 1U;
    g_balance_state.ax = raw.ax;
    g_balance_state.ay = raw.ay;
    g_balance_state.az = raw.az;
    g_balance_state.gx = raw.gx;
    g_balance_state.gy = raw.gy;
    g_balance_state.gz = raw.gz;

    float gy_calibrated = (float)raw.gy - g_balance_debug.gyro_y_offset;
    // float gz_calibrated = (float)raw.gz - g_balance_debug.gyro_z_offset;
    // float gyro_z_rate = gz_calibrated / 32768.0f * 2000.0f;
    float angle_acc = -atan2f((float)raw.ax, (float)raw.az) / PI_F * 180.0f;
    angle_acc += g_balance_debug.angle_offset;

    float angle_gyro = s_angle + gy_calibrated / 32768.0f * 2000.0f * ANGLE_LOOP_PERIOD_S;
    float alpha = 0.01f;
    s_angle = alpha * angle_acc + (1.0f - alpha) * angle_gyro;

    g_balance_state.angle_acc = angle_acc;
    g_balance_state.angle_gyro = angle_gyro;
    g_balance_state.angle = s_angle;
    // g_balance_state.gyro_z_rate = gyro_z_rate;

    if (s_angle > g_balance_debug.fall_angle_limit || s_angle < -g_balance_debug.fall_angle_limit) {
        BalanceCar_SetFault(BALANCE_FAULT_FALL);
        g_balance_debug.run_enable = 0U;
        BalanceCar_Stop();
        return;
    }

    if (g_balance_debug.run_enable == 0U || g_balance_state.imu_ready == 0U) {
        BalanceCar_Stop();
        return;
    }

    g_balance_state.run_active = 1U;
    g_angle_pid.Actual = s_angle;
    PID_Update(&g_angle_pid);

    float ave_pwm = g_angle_pid.Out;
    float left_pwm = ave_pwm + s_dif_pwm / 2.0f;
    float right_pwm = ave_pwm - s_dif_pwm / 2.0f;

    int16_t left = BalanceCar_ClampPwm(left_pwm);
    int16_t right = BalanceCar_ClampPwm(right_pwm);

    g_balance_state.ave_pwm = BalanceCar_ClampPwm(ave_pwm);
    g_balance_state.dif_pwm = BalanceCar_ClampPwm(s_dif_pwm);
    g_balance_state.left_pwm = left;
    g_balance_state.right_pwm = right;
    TB6612_SetMotors(left, right);
}

static void BalanceCar_RunSpeedLoop(void)
{
    int16_t left_delta = Encoder_GetLeftDelta();
    int16_t right_delta = Encoder_GetRightDelta();

    float left_speed = (float)left_delta / ENCODER_MAGNET_LINES / SPEED_LOOP_PERIOD_S / MOTOR_REDUCTION_RATIO;
    float right_speed = (float)right_delta / ENCODER_MAGNET_LINES / SPEED_LOOP_PERIOD_S / MOTOR_REDUCTION_RATIO;
    float ave_speed = (left_speed + right_speed) / 2.0f;
    float dif_speed = left_speed - right_speed;

    g_balance_state.left_speed = left_speed;
    g_balance_state.right_speed = right_speed;
    g_balance_state.ave_speed = ave_speed;
    g_balance_state.dif_speed = dif_speed;

    if (g_balance_debug.run_enable == 0U || g_balance_state.run_active == 0U) {
        return;
    }

    g_speed_pid.Target = g_balance_debug.speed_target;
    g_speed_pid.Actual = ave_speed;
    PID_Update(&g_speed_pid);
    g_angle_pid.Target = g_speed_pid.Out;

    float turn_rate_target = -g_balance_debug.turn_target * g_balance_debug.turn_gyro_scale;
    g_balance_state.turn_rate_target = turn_rate_target;
    g_turn_pid.Target = turn_rate_target;
    // g_turn_pid.Actual = g_balance_state.gyro_z_rate;
    PID_Update(&g_turn_pid);
    s_dif_pwm = g_turn_pid.Out * 5.0f;
}

HAL_StatusTypeDef BalanceCar_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    BalanceCar_ButtonInit();
    BalanceCar_ResetPidAndOutputs();

    if (TB6612_Init() != HAL_OK) {
        BalanceCar_SetFault(BALANCE_FAULT_MOTOR_INIT);
        status = HAL_ERROR;
    }
    if (Encoder_Init() != HAL_OK) {
        BalanceCar_SetFault(BALANCE_FAULT_ENCODER_INIT);
        status = HAL_ERROR;
    }
    if (Mpu6050_Init() != HAL_OK) {
        BalanceCar_SetFault(BALANCE_FAULT_MPU_INIT);
        g_balance_state.imu_ready = 0U;
        status = HAL_ERROR;
    } else {
        g_balance_state.imu_ready = 1U;
    }
    g_balance_state.mpu_id = Mpu6050_GetLastId();

    (void)AppSensors_Init();
    (void)RemoteControl_Init();
    (void)RaspiLink_Init();

    s_htim4.Instance = TIM4;
    s_htim4.Init.Prescaler = 64U - 1U;
    s_htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim4.Init.Period = 1000U - 1U;
    s_htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&s_htim4) != HAL_OK || HAL_TIM_Base_Start_IT(&s_htim4) != HAL_OK) {
        BalanceCar_SetFault(BALANCE_FAULT_TIMER_INIT);
        status = HAL_ERROR;
    }

    BalanceCar_Stop();
    return status;
}

void BalanceCar_Background(void)
{
    RaspiLink_Background();
    AppSensors_Background();
    RemoteControl_Background();

    if (g_balance_debug.clear_fault_request != 0U) {
        g_balance_debug.clear_fault_request = 0U;
        g_balance_state.fault_flags = BALANCE_FAULT_NONE;
    }

    if (g_balance_debug.reset_pid_request != 0U) {
        g_balance_debug.reset_pid_request = 0U;
        BalanceCar_ResetPidAndOutputs();
    }

    if (g_balance_debug.run_enable != 0U && s_last_run_enable == 0U) {
        BalanceCar_ClearRuntimeFaults();
        BalanceCar_ResetPidAndOutputs();
        s_angle = g_balance_state.angle;
    }
    s_last_run_enable = g_balance_debug.run_enable;

    while (s_angle_tick_pending != 0U) {
        __disable_irq();
        s_angle_tick_pending--;
        __enable_irq();
        BalanceCar_RunAngleLoop();
    }

    while (s_speed_tick_pending != 0U) {
        __disable_irq();
        s_speed_tick_pending--;
        __enable_irq();
        BalanceCar_RunSpeedLoop();
    }

    if (g_balance_debug.run_enable == 0U) {
        BalanceCar_Stop();
    }
}

void BalanceCar_TimerTick1ms(void)
{
    static uint8_t count_angle;
    static uint8_t count_speed;

    g_balance_state.control_ms++;
    BalanceCar_ButtonTick1ms();

    count_angle++;
    if (count_angle >= 10U) {
        count_angle = 0U;
        if (s_angle_tick_pending == 255U) {
            BalanceCar_SetFault(BALANCE_FAULT_TIMER_OVERRUN);
            g_balance_state.timer_error_flag = 1U;
        } else {
            s_angle_tick_pending++;
        }
    }

    count_speed++;
    if (count_speed >= 50U) {
        count_speed = 0U;
        if (s_speed_tick_pending == 255U) {
            BalanceCar_SetFault(BALANCE_FAULT_TIMER_OVERRUN);
            g_balance_state.timer_error_flag = 1U;
        } else {
            s_speed_tick_pending++;
        }
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance != TIM4) {
        return;
    }

    __HAL_RCC_TIM4_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM4_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&s_htim4, TIM_FLAG_UPDATE) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(&s_htim4, TIM_IT_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&s_htim4, TIM_IT_UPDATE);
        BalanceCar_TimerTick1ms();
    }
}
