#ifndef BALANCE_CAR_HX711_H
#define BALANCE_CAR_HX711_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef Hx711_Init(void);
HAL_StatusTypeDef Hx711_Read(int32_t *raw_count);

#endif
