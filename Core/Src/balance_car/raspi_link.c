#include "balance_car/raspi_link.h"

#include "balance_car/app_tasks.h"
#include "balance_car/balance_control.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RASPI_UART                         USART1
#define RASPI_UART_BAUDRATE                115200U
#define RASPI_UART_TX_PORT                 GPIOB
#define RASPI_UART_TX_PIN                  GPIO_PIN_6
#define RASPI_UART_RX_PORT                 GPIOB
#define RASPI_UART_RX_PIN                  GPIO_PIN_7
#define RASPI_LINE_MAX                     APP_RASPI_LINE_MAX
#define RASPI_TX_LINE_MAX                  95U
#define RASPI_RX_LINE_TIMEOUT_MS           150U
#define RASPI_PI_F                         3.1415926f
#define RASPI_DEG_TO_RAD                   (RASPI_PI_F / 180.0f)

volatile RaspiLinkState_t g_raspi_link_state = {0};
volatile RaspiLinkDebug_t g_raspi_link_debug = {
    .enable = 1U,
    .timeout_stop_enable = 1U,
    .allow_run_enable = 1U,
    .odom_tx_enable = 1U,
    .odom_reset_request = 0U,
    .speed_limit = 3.0f,
    .turn_limit = 2.0f,
    .wheel_radius_m = 0.0325f,
    .odom_angular_deadband_rps = 0.02f,
    .timeout_ms = 500U,
    .odom_period_ms = 50U,
};

static UART_HandleTypeDef s_huart_raspi;
static uint8_t s_rx_byte;
static char s_rx_line[RASPI_LINE_MAX + 1U];
static volatile uint8_t s_rx_index;
static volatile uint8_t s_line_ready;
static char s_parse_line[RASPI_LINE_MAX + 1U];
static char s_pending_line[RASPI_LINE_MAX + 1U];
static char s_tx_line[RASPI_TX_LINE_MAX + 1U];
static volatile uint8_t s_line_queued;
static volatile uint32_t s_rx_line_start_ms;
static uint32_t s_next_odom_tx_ms;
static uint32_t s_last_odom_update_ms;

static void RaspiLink_SetFault(uint32_t fault)
{
    g_raspi_link_state.fault_flags |= fault;
}

static float RaspiLink_Clamp(float value, float limit)
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

static void RaspiLink_StopMotion(void)
{
    AppControlCommand_t command;

    g_raspi_link_state.linear_x_cmd = 0.0f;
    g_raspi_link_state.angular_z_cmd = 0.0f;
    g_raspi_link_state.speed_target_rps = 0.0f;

    command.source = APP_CMD_SOURCE_RASPI;
    command.run_valid = 1U;
    command.run_enable = 0U;
    command.reset_pid = 0U;
    command.clear_fault = 0U;
    command.speed_target = 0.0f;
    command.turn_target = 0.0f;
    AppTasks_SubmitControlCommandFromTask(&command);
}

static void RaspiLink_StartReceive(void)
{
    if (HAL_UART_Receive_IT(&s_huart_raspi, &s_rx_byte, 1U) != HAL_OK) {
        RaspiLink_SetFault(RASPI_LINK_FAULT_RX_RESTART);
    }
}

static uint8_t RaspiLink_ChecksumAscii(const char *payload)
{
    uint8_t checksum = 0U;

    while (*payload != '\0') {
        checksum ^= (uint8_t)(*payload);
        payload++;
    }
    return checksum;
}

static HAL_StatusTypeDef RaspiLink_UartInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_AFIO_REMAP_USART1_ENABLE();

    gpio.Pin = RASPI_UART_TX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RASPI_UART_TX_PORT, &gpio);

    gpio.Pin = RASPI_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RASPI_UART_RX_PORT, &gpio);

    s_huart_raspi.Instance = RASPI_UART;
    s_huart_raspi.Init.BaudRate = RASPI_UART_BAUDRATE;
    s_huart_raspi.Init.WordLength = UART_WORDLENGTH_8B;
    s_huart_raspi.Init.StopBits = UART_STOPBITS_1;
    s_huart_raspi.Init.Parity = UART_PARITY_NONE;
    s_huart_raspi.Init.Mode = UART_MODE_TX_RX;
    s_huart_raspi.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_huart_raspi.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&s_huart_raspi) != HAL_OK) {
        RaspiLink_SetFault(RASPI_LINK_FAULT_UART_INIT);
        return HAL_ERROR;
    }

    HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    return HAL_OK;
}

HAL_StatusTypeDef RaspiLink_Init(void)
{
    HAL_StatusTypeDef status;

    memset((void *)&g_raspi_link_state, 0, sizeof(g_raspi_link_state));
    s_rx_index = 0U;
    s_line_ready = 0U;
    s_line_queued = 0U;
    s_rx_line_start_ms = 0U;
    s_next_odom_tx_ms = HAL_GetTick() + g_raspi_link_debug.odom_period_ms;
    s_last_odom_update_ms = HAL_GetTick();
    status = RaspiLink_UartInit();
    if (status == HAL_OK) {
        RaspiLink_StartReceive();
    }
    return status;
}

static uint8_t RaspiLink_HexValue(char ch, uint8_t *value)
{
    if (ch >= '0' && ch <= '9') {
        *value = (uint8_t)(ch - '0');
        return 1U;
    }
    if (ch >= 'A' && ch <= 'F') {
        *value = (uint8_t)(ch - 'A' + 10);
        return 1U;
    }
    if (ch >= 'a' && ch <= 'f') {
        *value = (uint8_t)(ch - 'a' + 10);
        return 1U;
    }
    return 0U;
}

static uint8_t RaspiLink_ParseChecksum(const char *text, uint8_t *checksum)
{
    uint8_t high;
    uint8_t low;

    if (text == 0 || checksum == 0 || text[0] == '\0' || text[1] == '\0' || text[2] != '\0') {
        return 0U;
    }
    if (RaspiLink_HexValue(text[0], &high) == 0U || RaspiLink_HexValue(text[1], &low) == 0U) {
        return 0U;
    }
    *checksum = (uint8_t)((high << 4) | low);
    return 1U;
}

static uint8_t RaspiLink_ParseUint32(const char *text, uint32_t *value)
{
    char *endptr;
    unsigned long parsed;

    if (text == 0 || value == 0 || *text == '\0') {
        return 0U;
    }
    parsed = strtoul(text, &endptr, 10);
    if (endptr == text || *endptr != '\0') {
        return 0U;
    }
    *value = (uint32_t)parsed;
    return 1U;
}

static uint8_t RaspiLink_ParseFlag(const char *text, uint8_t *value)
{
    if (text == 0 || value == 0 || text[1] != '\0') {
        return 0U;
    }
    if (text[0] == '0') {
        *value = 0U;
        return 1U;
    }
    if (text[0] == '1') {
        *value = 1U;
        return 1U;
    }
    return 0U;
}

static uint8_t RaspiLink_ParseFloat(const char *text, float *value)
{
    char *endptr;
    float parsed;

    if (text == 0 || value == 0 || *text == '\0') {
        return 0U;
    }
    parsed = strtof(text, &endptr);
    if (endptr == text || *endptr != '\0') {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

static void RaspiLink_MarkInvalid(uint32_t fault)
{
    g_raspi_link_state.invalid_frame_count++;
    g_raspi_link_state.parser_error = 1U;
    RaspiLink_SetFault(fault);
}

static void RaspiLink_ProcessLine(char *line)
{
    char *star;
    char *payload;
    char *fields[6];
    uint8_t field_count = 0U;
    uint8_t actual_checksum = 0U;
    uint8_t expected_checksum;
    uint32_t seq;
    uint8_t enable_motion;
    uint8_t obstacle_stop;
    float linear_x;
    float angular_z;
    float wheel_radius_m;
    float speed_target_rps;
    char *cursor;

    if (line[0] == '\0') {
        return;
    }

    strncpy((char *)g_raspi_link_state.last_frame, line, sizeof(g_raspi_link_state.last_frame) - 1U);
    g_raspi_link_state.last_frame[sizeof(g_raspi_link_state.last_frame) - 1U] = '\0';

    if (g_raspi_link_debug.enable == 0U) {
        return;
    }
    if (line[0] != '$') {
        RaspiLink_MarkInvalid(RASPI_LINK_FAULT_BAD_FRAME);
        return;
    }

    star = strchr(line, '*');
    if (star == 0) {
        RaspiLink_MarkInvalid(RASPI_LINK_FAULT_BAD_FRAME);
        return;
    }
    *star = '\0';
    if (RaspiLink_ParseChecksum(star + 1, &expected_checksum) == 0U) {
        RaspiLink_MarkInvalid(RASPI_LINK_FAULT_BAD_FRAME);
        return;
    }

    for (cursor = line + 1; *cursor != '\0'; cursor++) {
        actual_checksum ^= (uint8_t)(*cursor);
    }
    if (actual_checksum != expected_checksum) {
        g_raspi_link_state.checksum_error_count++;
        RaspiLink_MarkInvalid(RASPI_LINK_FAULT_CHECKSUM);
        return;
    }

    payload = line + 1;
    cursor = payload;
    while (field_count < 6U) {
        fields[field_count++] = cursor;
        cursor = strchr(cursor, ',');
        if (cursor == 0) {
            break;
        }
        *cursor = '\0';
        cursor++;
    }

    if (field_count != 6U || cursor != 0 || strcmp(fields[0], "BB") != 0) {
        RaspiLink_MarkInvalid(RASPI_LINK_FAULT_BAD_FRAME);
        return;
    }
    if (RaspiLink_ParseUint32(fields[1], &seq) == 0U ||
        RaspiLink_ParseFlag(fields[2], &enable_motion) == 0U ||
        RaspiLink_ParseFlag(fields[3], &obstacle_stop) == 0U ||
        RaspiLink_ParseFloat(fields[4], &linear_x) == 0U ||
        RaspiLink_ParseFloat(fields[5], &angular_z) == 0U) {
        RaspiLink_MarkInvalid(RASPI_LINK_FAULT_BAD_FRAME);
        return;
    }

    linear_x = RaspiLink_Clamp(linear_x, g_raspi_link_debug.speed_limit);
    angular_z = RaspiLink_Clamp(angular_z, g_raspi_link_debug.turn_limit);
    wheel_radius_m = g_raspi_link_debug.wheel_radius_m;
    if (wheel_radius_m <= 0.0f) {
        wheel_radius_m = 0.0325f;
    }
    speed_target_rps = linear_x / (2.0f * RASPI_PI_F * wheel_radius_m);

    g_raspi_link_state.valid_frame_count++;
    g_raspi_link_state.parser_error = 0U;
    g_raspi_link_state.link_active = 1U;
    g_raspi_link_state.last_rx_ms = HAL_GetTick();
    g_raspi_link_state.last_seq = seq;
    g_raspi_link_state.obstacle_stop = obstacle_stop;
    g_raspi_link_state.linear_x_cmd = linear_x;
    g_raspi_link_state.angular_z_cmd = angular_z;
    g_raspi_link_state.speed_target_rps = speed_target_rps;

    if (obstacle_stop != 0U || enable_motion == 0U) {
        RaspiLink_StopMotion();
        return;
    }

    AppControlCommand_t command;
    command.source = APP_CMD_SOURCE_RASPI;
    command.run_valid = g_raspi_link_debug.allow_run_enable;
    command.run_enable = 1U;
    command.reset_pid = 0U;
    command.clear_fault = 0U;
    command.speed_target = speed_target_rps;
    command.turn_target = angular_z;
    AppTasks_SubmitControlCommandFromTask(&command);
}

static void RaspiLink_CheckTimeout(void)
{
    uint32_t now;
    uint32_t timeout_ms = g_raspi_link_debug.timeout_ms;

    if (g_raspi_link_debug.timeout_stop_enable == 0U || timeout_ms == 0U) {
        return;
    }
    if (g_raspi_link_state.link_active == 0U) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - g_raspi_link_state.last_rx_ms) > timeout_ms) {
        g_raspi_link_state.link_active = 0U;
        g_raspi_link_state.timeout_count++;
        RaspiLink_SetFault(RASPI_LINK_FAULT_TIMEOUT);
        RaspiLink_StopMotion();
    }
}

static void RaspiLink_CheckLineTimeout(void)
{
    if (s_rx_index == 0U || s_rx_line_start_ms == 0U) {
        return;
    }
    if ((uint32_t)(HAL_GetTick() - s_rx_line_start_ms) <= RASPI_RX_LINE_TIMEOUT_MS) {
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

    g_raspi_link_state.partial_timeout_count++;
    g_raspi_link_state.frame_ready = 0U;
    RaspiLink_MarkInvalid(RASPI_LINK_FAULT_BAD_FRAME);
}

static void RaspiLink_NormalizeYaw(void)
{
    while (g_raspi_link_state.odom_yaw_rad > RASPI_PI_F) {
        g_raspi_link_state.odom_yaw_rad -= 2.0f * RASPI_PI_F;
    }
    while (g_raspi_link_state.odom_yaw_rad < -RASPI_PI_F) {
        g_raspi_link_state.odom_yaw_rad += 2.0f * RASPI_PI_F;
    }
}

static void RaspiLink_UpdateOdometry(uint32_t now)
{
    uint32_t dt_ms;
    float dt_s;
    float wheel_radius_m = g_raspi_link_debug.wheel_radius_m;
    float linear_x_mps;
    float angular_z_rps;
    BalanceCarState_t balance = {0};

    if (s_last_odom_update_ms == 0U) {
        s_last_odom_update_ms = now;
        return;
    }

    dt_ms = now - s_last_odom_update_ms;
    if (dt_ms == 0U) {
        return;
    }
    s_last_odom_update_ms = now;

    if (wheel_radius_m <= 0.0f) {
        wheel_radius_m = 0.0325f;
    }

    dt_s = (float)dt_ms / 1000.0f;
    AppTasks_CopyBalanceState(&balance);
    linear_x_mps = balance.ave_speed * 2.0f * RASPI_PI_F * wheel_radius_m;
    angular_z_rps = balance.gyro_z_rate * RASPI_DEG_TO_RAD;
    if (angular_z_rps > -g_raspi_link_debug.odom_angular_deadband_rps &&
        angular_z_rps < g_raspi_link_debug.odom_angular_deadband_rps) {
        angular_z_rps = 0.0f;
    }

    g_raspi_link_state.speed_actual_rps = balance.ave_speed;
    g_raspi_link_state.speed_actual_mps = linear_x_mps;
    g_raspi_link_state.left_pwm_snapshot = balance.left_pwm;
    g_raspi_link_state.right_pwm_snapshot = balance.right_pwm;
    g_raspi_link_state.odom_linear_x_mps = linear_x_mps;
    g_raspi_link_state.odom_angular_z_rps = angular_z_rps;
    g_raspi_link_state.odom_yaw_rad += angular_z_rps * dt_s;
    RaspiLink_NormalizeYaw();
    g_raspi_link_state.odom_x_m += linear_x_mps * cosf(g_raspi_link_state.odom_yaw_rad) * dt_s;
    g_raspi_link_state.odom_y_m += linear_x_mps * sinf(g_raspi_link_state.odom_yaw_rad) * dt_s;
}

static void RaspiLink_FormatFixed4(char *buffer, uint8_t size, float value)
{
    int32_t scaled;
    int32_t whole;
    int32_t frac;
    const char *sign = "";

    if (buffer == 0 || size == 0U) {
        return;
    }

    if (value < 0.0f) {
        sign = "-";
        value = -value;
    }

    scaled = (int32_t)(value * 10000.0f + 0.5f);
    whole = scaled / 10000;
    frac = scaled % 10000;
    (void)snprintf(buffer, size, "%s%ld.%04ld", sign, (long)whole, (long)frac);
}

static void RaspiLink_SendOdometry(uint32_t now)
{
    char payload[80];
    char x_text[16];
    char y_text[16];
    char yaw_text[16];
    char vx_text[16];
    char wz_text[16];
    uint8_t checksum;
    int payload_len;
    int frame_len;

    if (g_raspi_link_debug.odom_tx_enable == 0U || g_raspi_link_debug.odom_period_ms == 0U) {
        return;
    }
    if ((int32_t)(now - s_next_odom_tx_ms) < 0) {
        return;
    }
    s_next_odom_tx_ms = now + g_raspi_link_debug.odom_period_ms;

    if (g_raspi_link_state.tx_busy != 0U) {
        g_raspi_link_state.tx_error_count++;
        return;
    }
    if (Sem_RaspberryTxMutex != NULL &&
        xSemaphoreTake(Sem_RaspberryTxMutex, pdMS_TO_TICKS(2U)) != pdPASS) {
        g_raspi_link_state.tx_error_count++;
        return;
    }

    g_raspi_link_state.odom_seq++;
    RaspiLink_FormatFixed4(x_text, sizeof(x_text), g_raspi_link_state.odom_x_m);
    RaspiLink_FormatFixed4(y_text, sizeof(y_text), g_raspi_link_state.odom_y_m);
    RaspiLink_FormatFixed4(yaw_text, sizeof(yaw_text), g_raspi_link_state.odom_yaw_rad);
    RaspiLink_FormatFixed4(vx_text, sizeof(vx_text), g_raspi_link_state.odom_linear_x_mps);
    RaspiLink_FormatFixed4(wz_text, sizeof(wz_text), g_raspi_link_state.odom_angular_z_rps);
    payload_len = snprintf(payload, sizeof(payload),
                           "BO,%lu,%lu,%s,%s,%s,%s,%s",
                           (unsigned long)g_raspi_link_state.odom_seq,
                           (unsigned long)now,
                           x_text,
                           y_text,
                           yaw_text,
                           vx_text,
                           wz_text);
    if (payload_len <= 0 || payload_len >= (int)sizeof(payload)) {
        g_raspi_link_state.tx_error_count++;
        if (Sem_RaspberryTxMutex != NULL) {
            (void)xSemaphoreGive(Sem_RaspberryTxMutex);
        }
        return;
    }

    checksum = RaspiLink_ChecksumAscii(payload);
    frame_len = snprintf(s_tx_line, sizeof(s_tx_line), "$%s*%02X\n", payload, checksum);
    if (frame_len <= 0 || frame_len >= (int)sizeof(s_tx_line)) {
        g_raspi_link_state.tx_error_count++;
        if (Sem_RaspberryTxMutex != NULL) {
            (void)xSemaphoreGive(Sem_RaspberryTxMutex);
        }
        return;
    }

    strncpy((char *)g_raspi_link_state.last_tx_frame,
            s_tx_line,
            sizeof(g_raspi_link_state.last_tx_frame) - 1U);
    g_raspi_link_state.last_tx_frame[sizeof(g_raspi_link_state.last_tx_frame) - 1U] = '\0';

    g_raspi_link_state.tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&s_huart_raspi, (uint8_t *)s_tx_line, (uint16_t)frame_len) != HAL_OK) {
        g_raspi_link_state.tx_busy = 0U;
        g_raspi_link_state.tx_error_count++;
        if (Sem_RaspberryTxMutex != NULL) {
            (void)xSemaphoreGive(Sem_RaspberryTxMutex);
        }
        return;
    }
    if (Sem_RaspberryTxMutex != NULL) {
        (void)xSemaphoreGive(Sem_RaspberryTxMutex);
    }
    g_raspi_link_state.tx_count++;
    g_raspi_link_state.last_tx_ms = now;
}

void RaspiLink_Background(void)
{
    uint32_t now = HAL_GetTick();
    AppRaspiLine_t queued_line;
    uint8_t processed_queued_line = 0U;

    if (g_raspi_link_debug.odom_reset_request != 0U) {
        g_raspi_link_debug.odom_reset_request = 0U;
        g_raspi_link_state.odom_x_m = 0.0f;
        g_raspi_link_state.odom_y_m = 0.0f;
        g_raspi_link_state.odom_yaw_rad = 0.0f;
    }

    while (Queue_RaspberryRxLine != NULL &&
           xQueueReceive(Queue_RaspberryRxLine, &queued_line, 0U) == pdPASS) {
        processed_queued_line = 1U;
        strncpy(s_parse_line, queued_line.text, sizeof(s_parse_line) - 1U);
        s_parse_line[sizeof(s_parse_line) - 1U] = '\0';
        g_raspi_link_state.frame_ready = (uint8_t)(uxQueueMessagesWaiting(Queue_RaspberryRxLine) != 0U);
        RaspiLink_ProcessLine(s_parse_line);
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
        g_raspi_link_state.frame_ready = 0U;
        __enable_irq();

        RaspiLink_ProcessLine(s_parse_line);
    }

    RaspiLink_UpdateOdometry(now);
    RaspiLink_CheckLineTimeout();
    RaspiLink_CheckTimeout();
    RaspiLink_SendOdometry(now);
}

void RaspiLink_IRQHandler(void)
{
    HAL_UART_IRQHandler(&s_huart_raspi);
}

uint8_t RaspiLink_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ch;
    AppRaspiLine_t queued_line;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (huart->Instance != RASPI_UART) {
        return 0U;
    }

    ch = s_rx_byte;
    g_raspi_link_state.rx_count++;

    if (ch == '\r') {
        RaspiLink_StartReceive();
        return 1U;
    }

    if (ch == '\n') {
        s_rx_line[s_rx_index] = '\0';
        strncpy(s_pending_line, s_rx_line, sizeof(s_pending_line) - 1U);
        s_pending_line[sizeof(s_pending_line) - 1U] = '\0';
        s_line_queued = 0U;
        if (Queue_RaspberryRxLine != NULL &&
            xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            strncpy(queued_line.text, s_rx_line, sizeof(queued_line.text) - 1U);
            queued_line.text[sizeof(queued_line.text) - 1U] = '\0';
            if (xQueueOverwriteFromISR(Queue_RaspberryRxLine,
                                       &queued_line,
                                       &higher_priority_task_woken) != pdPASS) {
                RaspiLink_SetFault(RASPI_LINK_FAULT_LINE_OVERFLOW);
            } else {
                s_line_queued = 1U;
            }
        }
        s_rx_index = 0U;
        s_rx_line_start_ms = 0U;
        s_line_ready = 1U;
        g_raspi_link_state.frame_ready = 1U;
        RaspiLink_StartReceive();
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return 1U;
    }

    if (s_rx_index < RASPI_LINE_MAX) {
        if (s_rx_index == 0U) {
            s_rx_line_start_ms = HAL_GetTick();
        }
        s_rx_line[s_rx_index++] = (char)ch;
    } else {
        s_rx_index = 0U;
        s_rx_line[0] = '\0';
        s_rx_line_start_ms = 0U;
        RaspiLink_SetFault(RASPI_LINK_FAULT_LINE_OVERFLOW);
    }

    RaspiLink_StartReceive();
    return 1U;
}

uint8_t RaspiLink_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != RASPI_UART) {
        return 0U;
    }
    g_raspi_link_state.tx_busy = 0U;
    return 1U;
}

uint8_t RaspiLink_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != RASPI_UART) {
        return 0U;
    }
    g_raspi_link_state.tx_busy = 0U;
    RaspiLink_StartReceive();
    return 1U;
}
