#include "balance_car/remote_control.h"

#include "balance_car/app_sensors.h"
#include "balance_car/balance_control.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define REMOTE_UART                         USART1
#define REMOTE_UART_BAUDRATE                9600U
#define REMOTE_UART_TX_PORT                 GPIOB
#define REMOTE_UART_TX_PIN                  GPIO_PIN_6
#define REMOTE_UART_RX_PORT                 GPIOB
#define REMOTE_UART_RX_PIN                  GPIO_PIN_7
#define REMOTE_LINE_MAX                     31U
#define REMOTE_TELEMETRY_PERIOD_MS          500U
#define REMOTE_FLOAT_SCALE                  1000

volatile RemoteControlState_t g_remote_state = {0};
volatile RemoteControlDebug_t g_remote_debug = {
    .enable = 1U,
    .allow_run_command = 1U,
    .timeout_stop_enable = 1U,
    .speed_limit = 1.0f,
    .turn_limit = 0.8f,
    .timeout_ms = 500U,
};

static UART_HandleTypeDef s_huart_remote;
static uint8_t s_rx_byte;
static char s_rx_line[REMOTE_LINE_MAX + 1U];
static volatile uint8_t s_rx_index;
static volatile uint8_t s_line_ready;
static char s_parse_line[REMOTE_LINE_MAX + 1U];
static uint32_t s_next_telemetry_ms;

static int32_t RemoteControl_ToMilli(float value)
{
    if (value >= 0.0f) {
        return (int32_t)(value * (float)REMOTE_FLOAT_SCALE + 0.5f);
    }
    return (int32_t)(value * (float)REMOTE_FLOAT_SCALE - 0.5f);
}

static void RemoteControl_TransmitLine(const char *line, int len)
{
    uint32_t timeout_ms;

    if (len <= 0) {
        return;
    }
    timeout_ms = (uint32_t)len * 2U + 10U;
    (void)HAL_UART_Transmit(&s_huart_remote, (uint8_t *)line, (uint16_t)len, timeout_ms);
}

static void RemoteControl_SendDebugLine(const char *loop_name, float target, float actual)
{
    char line[48];
    int32_t target_milli = RemoteControl_ToMilli(target);
    int32_t actual_milli = RemoteControl_ToMilli(actual);
    int32_t target_abs = target_milli < 0 ? -target_milli : target_milli;
    int32_t actual_abs = actual_milli < 0 ? -actual_milli : actual_milli;
    int len = snprintf(line, sizeof(line),
                       "DBG %s %s%ld.%03ld %s%ld.%03ld\n",
                       loop_name,
                       target_milli < 0 ? "-" : "",
                       (long)(target_abs / REMOTE_FLOAT_SCALE),
                       (long)(target_abs % REMOTE_FLOAT_SCALE),
                       actual_milli < 0 ? "-" : "",
                       (long)(actual_abs / REMOTE_FLOAT_SCALE),
                       (long)(actual_abs % REMOTE_FLOAT_SCALE));
    if (len >= (int)sizeof(line)) {
        len = (int)sizeof(line) - 1;
    }
    RemoteControl_TransmitLine(line, len);
}

static float RemoteControl_Clamp(float value, float limit)
{
    if (limit < 0.0f) {
        limit = -limit;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void RemoteControl_SetFault(uint32_t fault)
{
    g_remote_state.fault_flags |= fault;
}

static void RemoteControl_StartReceive(void)
{
    if (HAL_UART_Receive_IT(&s_huart_remote, &s_rx_byte, 1U) != HAL_OK) {
        RemoteControl_SetFault(REMOTE_FAULT_RX_RESTART);
    }
}

static HAL_StatusTypeDef RemoteControl_UartInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_AFIO_REMAP_USART1_ENABLE();

    gpio.Pin = REMOTE_UART_TX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(REMOTE_UART_TX_PORT, &gpio);

    gpio.Pin = REMOTE_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(REMOTE_UART_RX_PORT, &gpio);

    s_huart_remote.Instance = REMOTE_UART;
    s_huart_remote.Init.BaudRate = REMOTE_UART_BAUDRATE;
    s_huart_remote.Init.WordLength = UART_WORDLENGTH_8B;
    s_huart_remote.Init.StopBits = UART_STOPBITS_1;
    s_huart_remote.Init.Parity = UART_PARITY_NONE;
    s_huart_remote.Init.Mode = UART_MODE_TX_RX;
    s_huart_remote.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_huart_remote.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&s_huart_remote) != HAL_OK) {
        RemoteControl_SetFault(REMOTE_FAULT_UART_INIT);
        return HAL_ERROR;
    }

    HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    return HAL_OK;
}

HAL_StatusTypeDef RemoteControl_Init(void)
{
    HAL_StatusTypeDef status;

    memset((void *)&g_remote_state, 0, sizeof(g_remote_state));
    s_rx_index = 0U;
    s_line_ready = 0U;
    s_next_telemetry_ms = HAL_GetTick() + REMOTE_TELEMETRY_PERIOD_MS;
    status = RemoteControl_UartInit();
    if (status == HAL_OK) {
        RemoteControl_StartReceive();
    }
    return status;
}

static void RemoteControl_SendTelemetry(void)
{
    char line[96];
    int temp10 = (int)(g_sensor_state.temperature_c * 10.0f);
    int humi = (int)(g_sensor_state.humidity_percent + 0.5f);
    int weight = (int)(g_sensor_state.weight_g + 0.5f);
    int len;

    len = snprintf(line, sizeof(line),
                   "OLED Temp:%d.%d C|Humi:%d %%|Weight:%d g|HX:%ld\n",
                   temp10 / 10,
                   temp10 < 0 ? -(temp10 % 10) : temp10 % 10,
                   humi,
                   weight,
                   (long)g_sensor_state.hx711_raw);
    if (len >= (int)sizeof(line)) {
        len = (int)sizeof(line) - 1;
    }

    RemoteControl_TransmitLine(line, len);
    RemoteControl_SendDebugLine("SPD", g_speed_pid.Target, g_speed_pid.Actual);
    RemoteControl_SendDebugLine("ANG", g_angle_pid.Target, g_angle_pid.Actual);
    RemoteControl_SendDebugLine("TURN", g_turn_pid.Target, g_turn_pid.Actual);
}

static void RemoteControl_MarkValid(const char *line)
{
    g_remote_state.valid_cmd_count++;
    g_remote_state.parser_error = 0U;
    g_remote_state.last_rx_ms = HAL_GetTick();
    g_remote_state.link_active = 1U;
    strncpy((char *)g_remote_state.last_command, line, sizeof(g_remote_state.last_command) - 1U);
    g_remote_state.last_command[sizeof(g_remote_state.last_command) - 1U] = '\0';
}

static void RemoteControl_MarkInvalid(const char *line)
{
    g_remote_state.invalid_cmd_count++;
    g_remote_state.parser_error = 1U;
    RemoteControl_SetFault(REMOTE_FAULT_BAD_COMMAND);
    strncpy((char *)g_remote_state.last_command, line, sizeof(g_remote_state.last_command) - 1U);
    g_remote_state.last_command[sizeof(g_remote_state.last_command) - 1U] = '\0';
}

static uint8_t RemoteControl_ParseFloat(const char *text, float *value)
{
    char *endptr;
    float parsed;

    if (text == 0 || value == 0) {
        return 0U;
    }

    parsed = strtof(text, &endptr);
    if (endptr == text) {
        return 0U;
    }
    while (*endptr == ' ') {
        endptr++;
    }
    if (*endptr != '\0') {
        return 0U;
    }

    *value = parsed;
    return 1U;
}

static uint8_t RemoteControl_ParsePidValues(char *text, float *kp, float *ki, float *kd)
{
    char *cursor;
    char *endptr;

    if (text == 0 || kp == 0 || ki == 0 || kd == 0) {
        return 0U;
    }

    cursor = text;
    *kp = strtof(cursor, &endptr);
    if (endptr == cursor) {
        return 0U;
    }
    cursor = endptr;
    while (*cursor == ' ') {
        cursor++;
    }

    *ki = strtof(cursor, &endptr);
    if (endptr == cursor) {
        return 0U;
    }
    cursor = endptr;
    while (*cursor == ' ') {
        cursor++;
    }

    *kd = strtof(cursor, &endptr);
    if (endptr == cursor) {
        return 0U;
    }
    while (*endptr == ' ') {
        endptr++;
    }
    if (*endptr != '\0') {
        return 0U;
    }
    return 1U;
}

static uint8_t RemoteControl_SetPid(char *line)
{
    char *values;
    PID_t *pid = 0;
    float kp;
    float ki;
    float kd;

    if (strncmp(line, "SPD ", 4U) == 0) {
        pid = &g_speed_pid;
        values = &line[4];
    } else if (strncmp(line, "ANG ", 4U) == 0) {
        pid = &g_angle_pid;
        values = &line[4];
    } else if (strncmp(line, "TURN ", 5U) == 0) {
        pid = &g_turn_pid;
        values = &line[5];
    } else {
        return 0U;
    }

    if (RemoteControl_ParsePidValues(values, &kp, &ki, &kd) == 0U) {
        return 0U;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    PID_Init(pid);
    return 1U;
}

static void RemoteControl_ProcessLine(char *line)
{
    if (line[0] == '\0') {
        return;
    }

    if (g_remote_debug.enable == 0U) {
        RemoteControl_MarkValid(line);
        return;
    }

    if (strcmp(line, "STOP") == 0) {
        g_balance_debug.speed_target = 0.0f;
        g_balance_debug.turn_target = 0.0f;
        g_remote_state.speed_cmd = 0.0f;
        g_remote_state.turn_cmd = 0.0f;
        RemoteControl_MarkValid(line);
        return;
    }

    if (strcmp(line, "PING") == 0) {
        RemoteControl_MarkValid(line);
        return;
    }

    if (strcmp(line, "PIDRST") == 0) {
        g_balance_debug.reset_pid_request = 1U;
        RemoteControl_MarkValid(line);
        return;
    }

    if (strncmp(line, "RUN ", 4U) == 0) {
        if (strcmp(&line[4], "1") == 0) {
            if (g_remote_debug.allow_run_command != 0U) {
                g_balance_debug.run_enable = 1U;
            }
            RemoteControl_MarkValid(line);
            return;
        }
        if (strcmp(&line[4], "0") == 0) {
            if (g_remote_debug.allow_run_command != 0U) {
                g_balance_debug.run_enable = 0U;
            }
            g_balance_debug.speed_target = 0.0f;
            g_balance_debug.turn_target = 0.0f;
            g_remote_state.speed_cmd = 0.0f;
            g_remote_state.turn_cmd = 0.0f;
            RemoteControl_MarkValid(line);
            return;
        }
    }

    if (strncmp(line, "SPD ", 4U) == 0) {
        float speed;
        if (RemoteControl_ParseFloat(&line[4], &speed) != 0U) {
            speed = RemoteControl_Clamp(speed, g_remote_debug.speed_limit);
            g_balance_debug.speed_target = speed;
            g_remote_state.speed_cmd = speed;
            RemoteControl_MarkValid(line);
            return;
        }
    }

    if (strncmp(line, "TURN ", 5U) == 0) {
        float turn;
        if (RemoteControl_ParseFloat(&line[5], &turn) != 0U) {
            turn = RemoteControl_Clamp(turn, g_remote_debug.turn_limit);
            g_balance_debug.turn_target = turn;
            g_remote_state.turn_cmd = turn;
            RemoteControl_MarkValid(line);
            return;
        }
    }

    if (strncmp(line, "PID ", 4U) == 0) {
        if (RemoteControl_SetPid(&line[4]) != 0U) {
            RemoteControl_MarkValid(line);
            return;
        }
    }

    RemoteControl_MarkInvalid(line);
}

static void RemoteControl_CheckTimeout(void)
{
    uint32_t now;
    uint32_t timeout_ms = g_remote_debug.timeout_ms;

    if (g_remote_debug.timeout_stop_enable == 0U || timeout_ms == 0U) {
        return;
    }
    if (g_remote_state.link_active == 0U) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - g_remote_state.last_rx_ms) > timeout_ms) {
        g_remote_state.link_active = 0U;
        g_remote_state.timeout_count++;
        g_balance_debug.speed_target = 0.0f;
        g_balance_debug.turn_target = 0.0f;
        g_remote_state.speed_cmd = 0.0f;
        g_remote_state.turn_cmd = 0.0f;
    }
}

void RemoteControl_Background(void)
{
    uint32_t now;

    if (s_line_ready != 0U) {
        __disable_irq();
        strncpy(s_parse_line, s_rx_line, sizeof(s_parse_line) - 1U);
        s_parse_line[sizeof(s_parse_line) - 1U] = '\0';
        s_line_ready = 0U;
        g_remote_state.command_ready = 0U;
        __enable_irq();

        RemoteControl_ProcessLine(s_parse_line);
    }

    RemoteControl_CheckTimeout();

    now = HAL_GetTick();
    if ((int32_t)(now - s_next_telemetry_ms) >= 0) {
        s_next_telemetry_ms = now + REMOTE_TELEMETRY_PERIOD_MS;
        RemoteControl_SendTelemetry();
    }
}

void RemoteControl_IRQHandler(void)
{
    HAL_UART_IRQHandler(&s_huart_remote);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ch;

    if (huart->Instance != REMOTE_UART) {
        return;
    }

    ch = s_rx_byte;
    g_remote_state.rx_count++;

    if (ch == '\r') {
        RemoteControl_StartReceive();
        return;
    }

    if (ch == '\n') {
        s_rx_line[s_rx_index] = '\0';
        s_rx_index = 0U;
        s_line_ready = 1U;
        g_remote_state.command_ready = 1U;
        RemoteControl_StartReceive();
        return;
    }

    if (s_rx_index < REMOTE_LINE_MAX) {
        s_rx_line[s_rx_index++] = (char)ch;
    } else {
        s_rx_index = 0U;
        s_rx_line[0] = '\0';
        RemoteControl_SetFault(REMOTE_FAULT_LINE_OVERFLOW);
    }

    RemoteControl_StartReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == REMOTE_UART) {
        RemoteControl_StartReceive();
    }
}
