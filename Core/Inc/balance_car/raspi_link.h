#ifndef BALANCE_CAR_RASPI_LINK_H
#define BALANCE_CAR_RASPI_LINK_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum {
    RASPI_LINK_FAULT_NONE = 0x00000000UL,
    RASPI_LINK_FAULT_UART_INIT = 0x00000001UL,
    RASPI_LINK_FAULT_RX_RESTART = 0x00000002UL,
    RASPI_LINK_FAULT_LINE_OVERFLOW = 0x00000004UL,
    RASPI_LINK_FAULT_BAD_FRAME = 0x00000008UL,
    RASPI_LINK_FAULT_CHECKSUM = 0x00000010UL,
    RASPI_LINK_FAULT_TIMEOUT = 0x00000020UL
} RaspiLinkFault_t;

typedef struct {
    uint8_t link_active;
    uint8_t frame_ready;
    uint8_t parser_error;
    uint8_t obstacle_stop;
    uint8_t tx_busy;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t tx_error_count;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t checksum_error_count;
    uint32_t timeout_count;
    uint32_t partial_timeout_count;
    uint32_t last_rx_ms;
    uint32_t last_tx_ms;
    uint32_t last_seq;
    uint32_t odom_seq;
    uint32_t fault_flags;
    float linear_x_cmd;
    float angular_z_cmd;
    float speed_target_rps;
    float speed_actual_rps;
    float speed_actual_mps;
    int16_t left_pwm_snapshot;
    int16_t right_pwm_snapshot;
    float odom_x_m;
    float odom_y_m;
    float odom_yaw_rad;
    float odom_linear_x_mps;
    float odom_angular_z_rps;
    char last_frame[80];
    char last_tx_frame[96];
} RaspiLinkState_t;

typedef struct {
    uint8_t enable;
    uint8_t timeout_stop_enable;
    uint8_t allow_run_enable;
    uint8_t odom_tx_enable;
    uint8_t odom_reset_request;
    float speed_limit;
    float turn_limit;
    float wheel_radius_m;
    float odom_angular_deadband_rps;
    uint32_t timeout_ms;
    uint32_t odom_period_ms;
} RaspiLinkDebug_t;

extern volatile RaspiLinkState_t g_raspi_link_state;
extern volatile RaspiLinkDebug_t g_raspi_link_debug;

HAL_StatusTypeDef RaspiLink_Init(void);
void RaspiLink_Background(void);
void RaspiLink_IRQHandler(void);
uint8_t RaspiLink_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t RaspiLink_TxCpltCallback(UART_HandleTypeDef *huart);
uint8_t RaspiLink_ErrorCallback(UART_HandleTypeDef *huart);

#endif
