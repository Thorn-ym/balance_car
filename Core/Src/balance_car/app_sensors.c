#include "balance_car/app_sensors.h"

#include "balance_car/dht11.h"
#include "balance_car/hx711.h"

#define HX711_SAMPLE_PERIOD_MS  100U
#define DHT_SAMPLE_PERIOD_MS    2000U
#define DHT_START_LOW_MS        18U
#define DEFAULT_G_PER_COUNT     1.0f

volatile AppSensorState_t g_sensor_state = {
    .temperature_c = 0.0f,
    .humidity_percent = 0.0f,
    .weight_g = 0.0f,
    .hx711_raw = 0,
    .hx711_zero_raw = 0,
    .hx711_g_per_count = DEFAULT_G_PER_COUNT,
    .hx711_weight_g = 0.0f,
    .sensor_fault_flags = SENSOR_FAULT_NONE,
};

static uint32_t s_next_hx711_ms;
static uint32_t s_next_dht_ms;
static uint32_t s_dht_start_ms;
static uint8_t s_dht_waiting;
static uint8_t s_hx711_zero_captured;

void AppSensors_SetFault(uint32_t fault)
{
    g_sensor_state.sensor_fault_flags |= fault;
}

void AppSensors_ClearFault(uint32_t fault)
{
    g_sensor_state.sensor_fault_flags &= ~fault;
}

HAL_StatusTypeDef AppSensors_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (Dht11_Init() != HAL_OK) {
        AppSensors_SetFault(SENSOR_FAULT_DHT_INIT);
        status = HAL_ERROR;
    }

    if (Hx711_Init() != HAL_OK) {
        AppSensors_SetFault(SENSOR_FAULT_HX711_INIT);
        status = HAL_ERROR;
    }

    s_next_hx711_ms = HAL_GetTick();
    s_next_dht_ms = HAL_GetTick() + 1000U;
    return status;
}

static void AppSensors_ReadHx711(void)
{
    int32_t raw;
    int32_t net;
    float weight;
    HAL_StatusTypeDef status;

    status = Hx711_ReadRaw(&raw);
    if (status == HAL_BUSY) {
        return;
    }
    if (status != HAL_OK) {
        AppSensors_SetFault(SENSOR_FAULT_HX711_READ);
        g_sensor_state.hx711_valid = 0U;
        return;
    }

    AppSensors_ClearFault(SENSOR_FAULT_HX711_READ);
    g_sensor_state.hx711_valid = 1U;
    g_sensor_state.hx711_raw = raw;

    if (s_hx711_zero_captured == 0U) {
        g_sensor_state.hx711_zero_raw = raw;
        s_hx711_zero_captured = 1U;
    }

    net = raw - g_sensor_state.hx711_zero_raw;
    weight = (float)net * g_sensor_state.hx711_g_per_count;
    if (weight < 0.0f) {
        weight = 0.0f;
    }
    g_sensor_state.hx711_weight_g = weight;
    g_sensor_state.weight_g = weight;
}

static void AppSensors_ServiceDht(void)
{
    Dht11Reading_t reading;

    if (s_dht_waiting == 0U) {
        if (Dht11_StartRead() == HAL_OK) {
            s_dht_start_ms = HAL_GetTick();
            s_dht_waiting = 1U;
        } else {
            AppSensors_SetFault(SENSOR_FAULT_DHT_READ);
            s_next_dht_ms = HAL_GetTick() + DHT_SAMPLE_PERIOD_MS;
        }
        return;
    }

    if ((HAL_GetTick() - s_dht_start_ms) < DHT_START_LOW_MS) {
        return;
    }

    s_dht_waiting = 0U;
    s_next_dht_ms = HAL_GetTick() + DHT_SAMPLE_PERIOD_MS;

    if (Dht11_FinishRead(&reading) != HAL_OK) {
        AppSensors_SetFault(SENSOR_FAULT_DHT_READ);
        g_sensor_state.dht_valid = 0U;
        g_sensor_state.dht_fail_step = Dht11_GetFailStep();
        return;
    }

    AppSensors_ClearFault(SENSOR_FAULT_DHT_READ);
    g_sensor_state.dht_valid = 1U;
    g_sensor_state.dht_fail_step = 0U;
    g_sensor_state.temperature_c = reading.temperature_c;
    g_sensor_state.humidity_percent = reading.humidity_percent;
}

void AppSensors_Background(void)
{
    uint32_t now = HAL_GetTick();

    if (s_dht_waiting != 0U) {
        AppSensors_ServiceDht();
        return;
    }

    if ((int32_t)(now - s_next_dht_ms) >= 0) {
        AppSensors_ServiceDht();
        return;
    }

    if ((int32_t)(now - s_next_hx711_ms) >= 0) {
        s_next_hx711_ms = now + HX711_SAMPLE_PERIOD_MS;
        AppSensors_ReadHx711();
    }
}

uint8_t AppSensors_NeedsServiceSoon(void)
{
    return s_dht_waiting;
}
