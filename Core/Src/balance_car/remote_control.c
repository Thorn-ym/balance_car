#include "balance_car/remote_control.h"

#include "balance_car/app_sensors.h"
#include "balance_car/app_tasks.h"
#include "balance_car/balance_control.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define REMOTE_UART                         USART3
#define REMOTE_UART_BAUDRATE                9600U
#define REMOTE_UART_TX_PORT                 GPIOB
#define REMOTE_UART_TX_PIN                  GPIO_PIN_10
#define REMOTE_UART_RX_PORT                 GPIOB
#define REMOTE_UART_RX_PIN                  GPIO_PIN_11
#define REMOTE_LINE_MAX                     APP_REMOTE_LINE_MAX
#define REMOTE_TELEMETRY_PERIOD_MS          500U
#define REMOTE_DEBUG_PERIOD_MS              100U
#define REMOTE_RX_LINE_TIMEOUT_MS           120U

volatile RemoteControlState_t g_remote_state = {0};
volatile RemoteControlDebug_t g_remote_debug = {
    .enable = 1U,
    .allow_run_command = 1U,
    .timeout_stop_enable = 1U,
    .speed_limit = 3.0f,
    .turn_limit = 2.0f,
    .timeout_ms = 500U,
};

static UART_HandleTypeDef s_huart_remote;
static uint8_t s_rx_byte;
static char s_rx_line[REMOTE_LINE_MAX + 1U];
static volatile uint8_t s_rx_index;
static volatile uint8_t s_line_ready;
static char s_parse_line[REMOTE_LINE_MAX + 1U];
static char s_pending_line[REMOTE_LINE_MAX + 1U];
static char s_tx_line[96];
static volatile uint8_t s_tx_busy;
static volatile uint8_t s_line_queued;
static volatile uint32_t s_rx_line_start_ms;
static uint32_t s_next_telemetry_ms;
static uint32_t s_next_debug_ms;

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
    __HAL_RCC_USART3_CLK_ENABLE();

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

    HAL_NVIC_SetPriority(USART3_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    return HAL_OK;
}

HAL_StatusTypeDef RemoteControl_Init(void)
{
    HAL_StatusTypeDef status;

    memset((void *)&g_remote_state, 0, sizeof(g_remote_state));
    s_rx_index = 0U;
    s_line_ready = 0U;
    s_line_queued = 0U;
    s_rx_line_start_ms = 0U;
    s_tx_busy = 0U;
    s_next_telemetry_ms = HAL_GetTick() + REMOTE_TELEMETRY_PERIOD_MS;
    s_next_debug_ms = HAL_GetTick() + REMOTE_DEBUG_PERIOD_MS;
    status = RemoteControl_UartInit();
    if (status == HAL_OK) {
        RemoteControl_StartReceive();
    }
    return status;
}

static uint8_t RemoteControl_SendLine(const char *line, int len)
{
    if (len <= 0 || s_tx_busy != 0U) {
        return 0U;
    }
    if (len >= (int)sizeof(s_tx_line)) {
        len = (int)sizeof(s_tx_line) - 1;
    }
    if (Sem_BluetoothTxMutex != NULL &&
        xSemaphoreTake(Sem_BluetoothTxMutex, pdMS_TO_TICKS(2U)) != pdPASS) {
        return 0U;
    }
    memcpy(s_tx_line, line, (uint16_t)len);
    s_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&s_huart_remote, (uint8_t *)s_tx_line, (uint16_t)len) != HAL_OK) {
        s_tx_busy = 0U;
        if (Sem_BluetoothTxMutex != NULL) {
            (void)xSemaphoreGive(Sem_BluetoothTxMutex);
        }
        return 0U;
    }
    if (Sem_BluetoothTxMutex != NULL) {
        (void)xSemaphoreGive(Sem_BluetoothTxMutex);
    }
    return 1U;
}

static void RemoteControl_SendTelemetry(void)
{
    char line[96];
    char temp_text[20];
    char humi_text[20];
    AppSensorState_t sensor = {0};
    int temp10;
    int humi;
    int weight;
    int len;

    AppTasks_CopySensorState(&sensor);
    temp10 = (int)(sensor.temperature_c * 10.0f);
    humi = (int)(sensor.humidity_percent + 0.5f);
    weight = (int)(sensor.weight_g + 0.5f);

    if (sensor.dht_valid != 0U) {
        (void)snprintf(temp_text, sizeof(temp_text), "Temp:%d.%d C",
                       temp10 / 10,
                       temp10 < 0 ? -(temp10 % 10) : temp10 % 10);
        (void)snprintf(humi_text, sizeof(humi_text), "Humi:%d %%", humi);
    } else {
        (void)snprintf(temp_text, sizeof(temp_text), "DHT Err:%u", (unsigned)sensor.dht_fail_step);
        (void)snprintf(humi_text, sizeof(humi_text), "Humi:-- %%");
    }

    len = snprintf(line, sizeof(line),
                   "OLED %s|%s|Weight:%d g|Raw:%ld\n",
                   temp_text,
                   humi_text,
                   weight,
                   (long)sensor.hx711_raw);
    if (len <= 0) {
        return;
    }
    if (len >= (int)sizeof(line)) {
        len = (int)sizeof(line) - 1;
    }

    RemoteControl_SendLine(line, len);
}

static void RemoteControl_SendDebugTelemetry(void)
{
    char line[96];
    AppPidSnapshot_t pid = {0};
    int len;

    AppTasks_CopyPidSnapshot(&pid);
    len = snprintf(line, sizeof(line),
                   "DBG SPD %.3f %.3f ANG %.3f %.3f TURN %.3f %.3f\n",
                   pid.speed.Target,
                   pid.speed.Actual,
                   pid.angle.Target,
                   pid.angle.Actual,
                   pid.turn.Target,
                   pid.turn.Actual);
    if (len <= 0) {
        return;
    }
    if (len >= (int)sizeof(line)) {
        len = (int)sizeof(line) - 1;
    }
    (void)RemoteControl_SendLine(line, len);
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
    char *ki_text;
    char *kd_text;

    if (text == 0 || kp == 0 || ki == 0 || kd == 0) {
        return 0U;
    }

    while (*text == ' ') {
        text++;
    }
    ki_text = strchr(text, ' ');
    if (ki_text == 0) {
        return 0U;
    }
    *ki_text = '\0';
    ki_text++;
    while (*ki_text == ' ') {
        ki_text++;
    }

    kd_text = strchr(ki_text, ' ');
    if (kd_text == 0) {
        return 0U;
    }
    *kd_text = '\0';
    kd_text++;
    while (*kd_text == ' ') {
        kd_text++;
    }

    return (uint8_t)(RemoteControl_ParseFloat(text, kp) != 0U &&
                     RemoteControl_ParseFloat(ki_text, ki) != 0U &&
                     RemoteControl_ParseFloat(kd_text, kd) != 0U);
}

static void RemoteControl_SubmitPid(uint8_t target, float kp, float ki, float kd)
{
    AppPidUpdate_t update;

    update.source = APP_CMD_SOURCE_REMOTE;
    update.target = target;
    update.kp = kp;
    update.ki = ki;
    update.kd = kd;
    AppTasks_SubmitPidUpdateFromTask(&update);
}

static void RemoteControl_SubmitCommand(uint8_t run_valid,
                                        uint8_t run_enable,
                                        uint8_t reset_pid,
                                        float speed,
                                        float turn)
{
    AppControlCommand_t command;

    command.source = APP_CMD_SOURCE_REMOTE;
    command.run_valid = run_valid;
    command.run_enable = run_enable;
    command.reset_pid = reset_pid;
    command.clear_fault = 0U;
    command.speed_target = speed;
    command.turn_target = turn;
    AppTasks_SubmitControlCommandFromTask(&command);
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
        g_remote_state.speed_cmd = 0.0f;
        g_remote_state.turn_cmd = 0.0f;
        RemoteControl_SubmitCommand(0U, 0U, 0U, 0.0f, 0.0f);
        RemoteControl_MarkValid(line);
        return;
    }

    if (strcmp(line, "PING") == 0) {
        RemoteControl_MarkValid(line);
        return;
    }

    if (strcmp(line, "PIDRST") == 0) {
        RemoteControl_SubmitCommand(0U, 0U, 1U, g_remote_state.speed_cmd, g_remote_state.turn_cmd);
        RemoteControl_MarkValid(line);
        return;
    }

    if (strncmp(line, "RUN ", 4U) == 0) {
        if (strcmp(&line[4], "1") == 0) {
            RemoteControl_SubmitCommand(g_remote_debug.allow_run_command, 1U, 0U,
                                        g_remote_state.speed_cmd, g_remote_state.turn_cmd);
            RemoteControl_MarkValid(line);
            return;
        }
        if (strcmp(&line[4], "0") == 0) {
            g_remote_state.speed_cmd = 0.0f;
            g_remote_state.turn_cmd = 0.0f;
            RemoteControl_SubmitCommand(g_remote_debug.allow_run_command, 0U, 0U, 0.0f, 0.0f);
            RemoteControl_MarkValid(line);
            return;
        }
    }

    if (strncmp(line, "PID ", 4U) == 0) {
        char *values_text;
        float kp;
        float ki;
        float kd;

        values_text = strchr(&line[4], ' ');
        if (values_text != 0) {
            *values_text = '\0';
            values_text++;
            if (RemoteControl_ParsePidValues(values_text, &kp, &ki, &kd) != 0U) {
                if (strcmp(&line[4], "SPD") == 0) {
                    RemoteControl_SubmitPid(APP_PID_TARGET_SPEED, kp, ki, kd);
                    RemoteControl_MarkValid("PID SPD");
                    return;
                }
                if (strcmp(&line[4], "ANG") == 0) {
                    RemoteControl_SubmitPid(APP_PID_TARGET_ANGLE, kp, ki, kd);
                    RemoteControl_MarkValid("PID ANG");
                    return;
                }
                if (strcmp(&line[4], "TURN") == 0) {
                    RemoteControl_SubmitPid(APP_PID_TARGET_TURN, kp, ki, kd);
                    RemoteControl_MarkValid("PID TURN");
                    return;
                }
            }
        }
    }

    if (strncmp(line, "SPD ", 4U) == 0) {
        float speed;
        if (RemoteControl_ParseFloat(&line[4], &speed) != 0U) {
            speed = RemoteControl_Clamp(speed, g_remote_debug.speed_limit);
            g_remote_state.speed_cmd = speed;
            RemoteControl_SubmitCommand(0U, 0U, 0U, speed, g_remote_state.turn_cmd);
            RemoteControl_MarkValid(line);
            return;
        }
    }

    if (strncmp(line, "CTL ", 4U) == 0) {
        char *turn_text;
        float speed;
        float turn;

        turn_text = strchr(&line[4], ' ');
        if (turn_text != 0) {
            *turn_text = '\0';
            turn_text++;
            while (*turn_text == ' ') {
                turn_text++;
            }
            if (RemoteControl_ParseFloat(&line[4], &speed) != 0U &&
                RemoteControl_ParseFloat(turn_text, &turn) != 0U) {
                speed = RemoteControl_Clamp(speed, g_remote_debug.speed_limit);
                turn = RemoteControl_Clamp(turn, g_remote_debug.turn_limit);
                g_remote_state.speed_cmd = speed;
                g_remote_state.turn_cmd = turn;
                RemoteControl_SubmitCommand(0U, 0U, 0U, speed, turn);
                RemoteControl_MarkValid("CTL");
                return;
            }
        }
    }

    if (strncmp(line, "TURN ", 5U) == 0) {
        float turn;
        if (RemoteControl_ParseFloat(&line[5], &turn) != 0U) {
            turn = RemoteControl_Clamp(turn, g_remote_debug.turn_limit);
            g_remote_state.turn_cmd = turn;
            RemoteControl_SubmitCommand(0U, 0U, 0U, g_remote_state.speed_cmd, turn);
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
        g_remote_state.speed_cmd = 0.0f;
        g_remote_state.turn_cmd = 0.0f;
        RemoteControl_SubmitCommand(0U, 0U, 0U, 0.0f, 0.0f);
    }
}

static void RemoteControl_CheckLineTimeout(void)
{
    if (s_rx_index == 0U || s_rx_line_start_ms == 0U) {
        return;
    }
    if ((uint32_t)(HAL_GetTick() - s_rx_line_start_ms) <= REMOTE_RX_LINE_TIMEOUT_MS) {
        return;
    }

    __disable_irq();
    s_rx_index = 0U;
    s_rx_line[0] = '\0';
    s_pending_line[0] = '\0';
    s_line_ready = 0U;
    s_line_queued = 0U;
    s_rx_line_start_ms = 0U;
    __enable_irq();

    g_remote_state.partial_timeout_count++;
    g_remote_state.command_ready = 0U;
    g_remote_state.parser_error = 1U;
    RemoteControl_SetFault(REMOTE_FAULT_BAD_COMMAND);
}

void RemoteControl_Background(void)
{
    uint32_t now;
    AppRemoteLine_t queued_line;
    char latest_ctl_line[REMOTE_LINE_MAX + 1U];
    uint8_t processed_queued_line = 0U;
    uint8_t has_latest_ctl = 0U;

    while (Queue_BluetoothRxLine != NULL &&
           xQueueReceive(Queue_BluetoothRxLine, &queued_line, 0U) == pdPASS) {
        processed_queued_line = 1U;
        strncpy(s_parse_line, queued_line.text, sizeof(s_parse_line) - 1U);
        s_parse_line[sizeof(s_parse_line) - 1U] = '\0';
        g_remote_state.command_ready = (uint8_t)(uxQueueMessagesWaiting(Queue_BluetoothRxLine) != 0U);
        if (strncmp(s_parse_line, "CTL ", 4U) == 0) {
            strncpy(latest_ctl_line, s_parse_line, sizeof(latest_ctl_line) - 1U);
            latest_ctl_line[sizeof(latest_ctl_line) - 1U] = '\0';
            has_latest_ctl = 1U;
        } else {
            if (has_latest_ctl != 0U) {
                RemoteControl_ProcessLine(latest_ctl_line);
                has_latest_ctl = 0U;
            }
            RemoteControl_ProcessLine(s_parse_line);
        }
    }

    if (has_latest_ctl != 0U) {
        RemoteControl_ProcessLine(latest_ctl_line);
    }

    if (processed_queued_line != 0U) {
        __disable_irq();
        s_line_ready = 0U;
        s_line_queued = 0U;
        __enable_irq();
    }

    if (s_line_ready != 0U && s_line_queued == 0U) {
        __disable_irq();
        strncpy(s_parse_line, s_pending_line, sizeof(s_parse_line) - 1U);
        s_parse_line[sizeof(s_parse_line) - 1U] = '\0';
        s_line_ready = 0U;
        s_line_queued = 0U;
        g_remote_state.command_ready = 0U;
        __enable_irq();

        RemoteControl_ProcessLine(s_parse_line);
    }

    RemoteControl_CheckLineTimeout();
    RemoteControl_CheckTimeout();

    now = HAL_GetTick();
    if ((int32_t)(now - s_next_telemetry_ms) >= 0) {
        s_next_telemetry_ms = now + REMOTE_TELEMETRY_PERIOD_MS;
        RemoteControl_SendTelemetry();
    }
    if ((int32_t)(now - s_next_debug_ms) >= 0) {
        s_next_debug_ms = now + REMOTE_DEBUG_PERIOD_MS;
        RemoteControl_SendDebugTelemetry();
    }
}

void RemoteControl_IRQHandler(void)
{
    HAL_UART_IRQHandler(&s_huart_remote);
}

uint8_t RemoteControl_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ch;
    AppRemoteLine_t queued_line;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (huart->Instance != REMOTE_UART) {
        return 0U;
    }

    ch = s_rx_byte;
    g_remote_state.rx_count++;

    if (ch == '\r') {
        RemoteControl_StartReceive();
        return 1U;
    }

    if (ch == '\n') {
        s_rx_line[s_rx_index] = '\0';
        strncpy(s_pending_line, s_rx_line, sizeof(s_pending_line) - 1U);
        s_pending_line[sizeof(s_pending_line) - 1U] = '\0';
        s_line_queued = 0U;
        if (Queue_BluetoothRxLine != NULL &&
            xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            strncpy(queued_line.text, s_rx_line, sizeof(queued_line.text) - 1U);
            queued_line.text[sizeof(queued_line.text) - 1U] = '\0';
            if (xQueueSendToBackFromISR(Queue_BluetoothRxLine,
                                        &queued_line,
                                        &higher_priority_task_woken) != pdPASS) {
                AppRemoteLine_t dropped_line;

                (void)xQueueReceiveFromISR(Queue_BluetoothRxLine,
                                           &dropped_line,
                                           &higher_priority_task_woken);
                if (xQueueSendToBackFromISR(Queue_BluetoothRxLine,
                                            &queued_line,
                                            &higher_priority_task_woken) != pdPASS) {
                    RemoteControl_SetFault(REMOTE_FAULT_LINE_OVERFLOW);
                } else {
                    s_line_queued = 1U;
                }
            } else {
                s_line_queued = 1U;
            }
        }
        s_rx_index = 0U;
        s_rx_line_start_ms = 0U;
        s_line_ready = 1U;
        g_remote_state.command_ready = 1U;
        AppTasks_NotifyBluetoothRxFromISR(&higher_priority_task_woken);
        RemoteControl_StartReceive();
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return 1U;
    }

    if (s_rx_index < REMOTE_LINE_MAX) {
        if (s_rx_index == 0U) {
            s_rx_line_start_ms = HAL_GetTick();
        }
        s_rx_line[s_rx_index++] = (char)ch;
    } else {
        s_rx_index = 0U;
        s_rx_line[0] = '\0';
        s_rx_line_start_ms = 0U;
        RemoteControl_SetFault(REMOTE_FAULT_LINE_OVERFLOW);
    }

    RemoteControl_StartReceive();
    return 1U;
}

uint8_t RemoteControl_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != REMOTE_UART) {
        return 0U;
    }
    s_tx_busy = 0U;
    return 1U;
}

uint8_t RemoteControl_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == REMOTE_UART) {
        s_tx_busy = 0U;
        RemoteControl_StartReceive();
        return 1U;
    }
    return 0U;
}
