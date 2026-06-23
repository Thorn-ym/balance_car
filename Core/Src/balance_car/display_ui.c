#include "balance_car/display_ui.h"

#include "balance_car/app_sensors.h"
#include "balance_car/oled_ssd1306.h"
#include <stdio.h>

#define DISPLAY_TEXT_PERIOD_MS     500U
#define DISPLAY_PAGE_PERIOD_MS     20U

static uint32_t s_next_text_ms;
static uint32_t s_next_page_ms;

HAL_StatusTypeDef DisplayUi_Init(void)
{
    HAL_StatusTypeDef status = Oled_Init();

    if (status != HAL_OK) {
        AppSensors_SetFault(SENSOR_FAULT_OLED_INIT);
        g_sensor_state.oled_ready = 0U;
        g_sensor_state.oled_addr_7bit = Oled_GetAddress7Bit();
        g_sensor_state.oled_probe_mask = Oled_GetProbeMask();
        g_sensor_state.oled_fail_step = Oled_GetFailStep();
        return status;
    }

    AppSensors_ClearFault(SENSOR_FAULT_OLED_INIT);
    g_sensor_state.oled_ready = 1U;
    g_sensor_state.oled_addr_7bit = Oled_GetAddress7Bit();
    g_sensor_state.oled_probe_mask = Oled_GetProbeMask();
    g_sensor_state.oled_fail_step = Oled_GetFailStep();
    s_next_text_ms = HAL_GetTick();
    s_next_page_ms = HAL_GetTick();
    return HAL_OK;
}

static void DisplayUi_UpdateText(void)
{
    char line[22];
    int temp10 = (int)(g_sensor_state.temperature_c * 10.0f);
    int weight = (int)(g_sensor_state.weight_g + 0.5f);

    Oled_Clear();

    (void)snprintf(line, sizeof(line), "Temp: %d.%d C", temp10 / 10, temp10 < 0 ? -(temp10 % 10) : temp10 % 10);
    Oled_WriteString(0U, 0U, line);

    (void)snprintf(line, sizeof(line), "Humi: %d %%", (int)(g_sensor_state.humidity_percent + 0.5f));
    Oled_WriteString(0U, 2U, line);

    (void)snprintf(line, sizeof(line), "Weight: %d g", weight);
    Oled_WriteString(0U, 4U, line);

    (void)snprintf(line, sizeof(line), "HX: %ld", (long)g_sensor_state.hx711_raw);
    Oled_WriteString(0U, 6U, line);
}

void DisplayUi_Background(void)
{
    uint32_t now;

    if (g_sensor_state.oled_ready == 0U) {
        return;
    }

    now = HAL_GetTick();
    if ((int32_t)(now - s_next_text_ms) >= 0) {
        s_next_text_ms = now + DISPLAY_TEXT_PERIOD_MS;
        DisplayUi_UpdateText();
    }

    if ((int32_t)(now - s_next_page_ms) >= 0) {
        s_next_page_ms = now + DISPLAY_PAGE_PERIOD_MS;
        if (Oled_RefreshNextPage() != HAL_OK) {
            AppSensors_SetFault(SENSOR_FAULT_OLED_REFRESH);
            g_sensor_state.oled_ready = 0U;
            g_sensor_state.oled_fail_step = Oled_GetFailStep();
        } else {
            AppSensors_ClearFault(SENSOR_FAULT_OLED_REFRESH);
        }
    }
}
