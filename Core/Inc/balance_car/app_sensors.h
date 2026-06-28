#ifndef BALANCE_CAR_APP_SENSORS_H
#define BALANCE_CAR_APP_SENSORS_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum {
    SENSOR_FAULT_NONE = 0x00000000UL,
    SENSOR_FAULT_DHT_INIT = 0x00000001UL,
    SENSOR_FAULT_DHT_READ = 0x00000002UL,
    SENSOR_FAULT_HX711_INIT = 0x00000004UL,
    SENSOR_FAULT_HX711_READ = 0x00000008UL,
    SENSOR_FAULT_OLED_INIT = 0x00000010UL,
    SENSOR_FAULT_OLED_REFRESH = 0x00000020UL
} SensorFault_t;

typedef struct {
    float temperature_c;
    float humidity_percent;
    float weight_g;
    int32_t hx711_raw;
    int32_t hx711_zero_raw;
    float hx711_g_per_count;
    float hx711_weight_g;
    uint32_t sensor_fault_flags;
    uint8_t dht_valid;
    uint8_t hx711_valid;
    uint8_t oled_ready;
    uint8_t oled_addr_7bit;
    uint8_t oled_probe_mask;
    uint8_t oled_fail_step;
    uint8_t dht_fail_step;
} AppSensorState_t;

extern volatile AppSensorState_t g_sensor_state;

HAL_StatusTypeDef AppSensors_Init(void);
void AppSensors_Background(void);
uint8_t AppSensors_NeedsServiceSoon(void);
void AppSensors_SetFault(uint32_t fault);
void AppSensors_ClearFault(uint32_t fault);

#endif
