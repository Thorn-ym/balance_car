#ifndef BALANCE_CAR_REMOTE_CONTROL_H
#define BALANCE_CAR_REMOTE_CONTROL_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum {
    REMOTE_FAULT_NONE = 0x00000000UL,
    REMOTE_FAULT_UART_INIT = 0x00000001UL,
    REMOTE_FAULT_RX_RESTART = 0x00000002UL,
    REMOTE_FAULT_LINE_OVERFLOW = 0x00000004UL,
    REMOTE_FAULT_BAD_COMMAND = 0x00000008UL
} RemoteControlFault_t;

typedef struct {
    uint8_t link_active;          /* Ozone观察: 1=最近timeout_ms内收到过有效遥控命令 */
    uint8_t command_ready;        /* Ozone观察: 1=收到完整命令行并等待后台解析，通常只会短暂为1 */
    uint8_t parser_error;         /* Ozone观察: 1=最近一次命令格式错误 */
    uint8_t reserved;             /* 保留 */
    uint32_t rx_count;            /* Ozone观察: USART3 PB11收到的字节数 */
    uint32_t valid_cmd_count;     /* Ozone观察: 有效命令计数 */
    uint32_t invalid_cmd_count;   /* Ozone观察: 无效命令计数 */
    uint32_t timeout_count;       /* Ozone观察: 遥控超时次数 */
    uint32_t partial_timeout_count; /* Ozone观察: 半条命令超过行超时时间后被丢弃次数 */
    uint32_t last_rx_ms;          /* Ozone观察: 最近一次有效命令的HAL毫秒时间 */
    uint32_t fault_flags;         /* Ozone观察: 遥控模块故障位，见RemoteControlFault_t */
    float speed_cmd;              /* Ozone观察: 最近一次遥控速度目标，已限幅 */
    float turn_cmd;               /* Ozone观察: 最近一次遥控转向目标，已限幅 */
    char last_command[32];        /* Ozone观察: 最近一次完整命令字符串 */
} RemoteControlState_t;

typedef struct {
    uint8_t enable;              /* Ozone写入: 1=允许遥控命令改变目标，0=忽略遥控命令 */
    uint8_t allow_run_command;   /* Ozone写入: 1=允许RUN 1/RUN 0控制启停 */
    uint8_t timeout_stop_enable; /* Ozone写入: 1=超时后自动清零speed_target/turn_target */
    uint8_t reserved;            /* 保留 */
    float speed_limit;           /* Ozone写入: 遥控速度目标绝对值限幅 */
    float turn_limit;            /* Ozone写入: 遥控转向目标绝对值限幅 */
    uint32_t timeout_ms;         /* Ozone写入: 遥控超时时间，单位ms */
} RemoteControlDebug_t;

extern volatile RemoteControlState_t g_remote_state;
extern volatile RemoteControlDebug_t g_remote_debug;

HAL_StatusTypeDef RemoteControl_Init(void);
void RemoteControl_Background(void);
void RemoteControl_IRQHandler(void);
uint8_t RemoteControl_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t RemoteControl_TxCpltCallback(UART_HandleTypeDef *huart);
uint8_t RemoteControl_ErrorCallback(UART_HandleTypeDef *huart);

#endif
