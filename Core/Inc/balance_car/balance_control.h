#ifndef BALANCE_CAR_BALANCE_CONTROL_H
#define BALANCE_CAR_BALANCE_CONTROL_H

#include "balance_car/pid.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum {
    BALANCE_FAULT_NONE = 0x00000000UL,          /* Ozone: 无故障 */
    BALANCE_FAULT_MPU_INIT = 0x00000001UL,      /* Ozone: MPU6050初始化失败，检查PB8/PB9、供电、地址0x68 */
    BALANCE_FAULT_MPU_READ = 0x00000002UL,      /* Ozone: MPU6050运行中读取失败，检查I2C连接和供电 */
    BALANCE_FAULT_FALL = 0x00000004UL,          /* Ozone: 倾角超过fall_angle_limit，程序自动停机 */
    BALANCE_FAULT_TIMER_OVERRUN = 0x00000008UL, /* Ozone: 控制后台处理太慢，10ms/50ms节拍堆积 */
    BALANCE_FAULT_MOTOR_INIT = 0x00000010UL,    /* Ozone: TIM3 PWM或TB6612 GPIO初始化失败 */
    BALANCE_FAULT_ENCODER_INIT = 0x00000020UL,  /* Ozone: TIM1/TIM2编码器初始化失败 */
    BALANCE_FAULT_TIMER_INIT = 0x00000040UL     /* Ozone: TIM4 1kHz控制节拍初始化失败 */
} BalanceCarFault_t;

typedef struct {
    uint8_t run_enable;          /* Ozone写入: 0=停机急停，1=允许平衡控制输出PWM */
    uint8_t reset_pid_request;   /* Ozone写入: 写1后清空三个PID历史量和PWM输出，程序会自动改回0 */
    uint8_t clear_fault_request; /* Ozone写入: 写1后清fault_flags，程序会自动改回0；硬件故障会再次置位 */
    uint8_t reserved;            /* 保留占位，不用调 */
    float speed_target;          /* Ozone写入: 目标前后速度。先保持0，速度环调好后再小幅改变 */
    float turn_target;           /* Ozone写入: 目标转向输入，内部乘turn_gyro_scale变成Z轴目标角速度 */
    float gyro_y_offset;         /* Ozone调试: 陀螺仪Y轴零漂。静止时看gy，填入静止偏置值 */
    float gyro_z_offset;         /* Ozone调试: 陀螺仪Z轴零漂。静止时看gz，填入静止偏置值 */
    float turn_gyro_scale;       /* Ozone调试: turn_target到Z轴目标角速度的比例，单位约deg/s */
    float angle_offset;          /* Ozone调试: 机械竖直角度偏置。竖直时调到angle接近0 */
    float fall_angle_limit;      /* Ozone调试: 倒车保护角度，默认50度，超过后自动run_enable=0 */
} BalanceCarDebug_t;

typedef struct {
    uint32_t control_ms;        /* Ozone观察: 1ms递增，确认TIM4控制节拍在跑 */
    uint32_t fault_flags;       /* Ozone观察: 故障位，含义见BalanceCarFault_t，可按位组合 */
    uint8_t run_active;         /* Ozone观察: 1=当前真的在输出控制；0=停机或安全条件不满足 */
    uint8_t imu_ready;          /* Ozone观察: 1=MPU6050可用；0=初始化或读取失败 */
    uint8_t timer_error_flag;   /* Ozone观察: 1=控制节拍处理堆积，主循环跑不过来 */
    uint8_t mpu_id;             /* Ozone观察: MPU6050 WHO_AM_I，正常应为0x68 */
    uint8_t button_raw;         /* 保留字段: 实体按键已移除，固定为0 */
    uint8_t button_stable;      /* 保留字段: 实体按键已移除，固定为0 */
    uint8_t button_toggle;      /* 保留字段: 实体按键已移除，固定为0 */
    uint8_t reserved0;          /* 保留占位 */

    int16_t ax;                 /* Ozone观察: MPU6050加速度X原始值 */
    int16_t ay;                 /* Ozone观察: MPU6050加速度Y原始值 */
    int16_t az;                 /* Ozone观察: MPU6050加速度Z原始值 */
    int16_t gx;                 /* Ozone观察: MPU6050陀螺仪X原始值 */
    int16_t gy;                 /* Ozone观察: MPU6050陀螺仪Y原始值，用来校准gyro_y_offset */
    int16_t gz;                 /* Ozone观察: MPU6050陀螺仪Z原始值 */

    float angle_acc;            /* Ozone观察: 只由加速度计算出的角度，静止准但运动时抖 */
    float angle_gyro;           /* Ozone观察: 由陀螺仪积分出的角度，短时平滑但会漂移 */
    float angle;                /* Ozone观察: 互补滤波后的最终俯仰角，调平衡主要看它 */
    float gyro_z_rate;          /* Ozone观察: Z轴角速度deg/s，转向环Actual，原地旋转时应明显变化 */
    float turn_rate_target;     /* Ozone观察: 转向环目标Z轴角速度deg/s，由turn_target换算得到 */

    float left_speed;           /* Ozone观察: 左轮输出轴速度，单位约为转/秒，前进方向应为正 */
    float right_speed;          /* Ozone观察: 右轮输出轴速度，单位约为转/秒，前进方向应为正 */
    float ave_speed;            /* Ozone观察: 左右轮平均速度，速度环Actual */
    float dif_speed;            /* Ozone观察: 左右轮速度差，仅用于辅助判断左右轮是否一致 */

    int16_t left_pwm;           /* Ozone观察: 左电机最终PWM，范围-100~100 */
    int16_t right_pwm;          /* Ozone观察: 右电机最终PWM，范围-100~100 */
    int16_t ave_pwm;            /* Ozone观察: 平均PWM，主要来自角度环输出 */
    int16_t dif_pwm;            /* Ozone观察: 差分PWM，主要来自转向环输出 */
} BalanceCarState_t;

extern volatile BalanceCarDebug_t g_balance_debug; /* Ozone主调试入口: 启停、目标、校准、清故障都改这里 */
extern volatile BalanceCarState_t g_balance_state; /* Ozone主观察入口: 角度、速度、PWM、故障都看这里 */
extern PID_t g_angle_pid;                          /* Ozone调参: 角度环PID，先调它让车能直立 */
extern PID_t g_speed_pid;                          /* Ozone调参: 速度环PID，角度环稳定后再打开 */
extern PID_t g_turn_pid;                           /* Ozone调参: 转向环PID，最后再打开 */

HAL_StatusTypeDef BalanceCar_Init(void);
void BalanceCar_Background(void);
void BalanceCar_TimerTick1ms(void);
void BalanceCar_ServiceRequests(void);
void BalanceCar_ControlStep10ms(void);
void BalanceCar_ApplyControlCommand(uint8_t run_valid,
                                    uint8_t run_enable,
                                    uint8_t reset_pid,
                                    uint8_t clear_fault,
                                    float speed_target,
                                    float turn_target);

#endif
