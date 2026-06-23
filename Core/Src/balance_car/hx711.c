#include "balance_car/hx711.h"

#define HX711_DOUT_PORT       GPIOA
#define HX711_DOUT_PIN        GPIO_PIN_2
#define HX711_SCK_PORT        GPIOB
#define HX711_SCK_PIN         GPIO_PIN_12
#define HX711_READY_TIMEOUT   100U

static void Hx711_DelayShort(void)
{
    volatile uint32_t i;

    for (i = 0U; i < 16U; i++) {
        __NOP();
    }
}

HAL_StatusTypeDef Hx711_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = HX711_DOUT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HX711_DOUT_PORT, &gpio);

    gpio.Pin = HX711_SCK_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HX711_SCK_PORT, &gpio);
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);

    return HAL_OK;
}

HAL_StatusTypeDef Hx711_Read(int32_t *raw_count)
{
    uint32_t start_ms;
    uint32_t value = 0U;
    uint8_t i;

    if (raw_count == 0) {
        return HAL_ERROR;
    }

    start_ms = HAL_GetTick();
    while (HAL_GPIO_ReadPin(HX711_DOUT_PORT, HX711_DOUT_PIN) == GPIO_PIN_SET) {
        if ((uint32_t)(HAL_GetTick() - start_ms) > HX711_READY_TIMEOUT) {
            return HAL_TIMEOUT;
        }
    }

    __disable_irq();
    for (i = 0U; i < 24U; i++) {
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
        Hx711_DelayShort();
        value = (value << 1) |
                (HAL_GPIO_ReadPin(HX711_DOUT_PORT, HX711_DOUT_PIN) == GPIO_PIN_SET ? 1U : 0U);
        HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
        Hx711_DelayShort();
    }

    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
    Hx711_DelayShort();
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
    __enable_irq();

    if ((value & 0x00800000UL) != 0U) {
        value |= 0xFF000000UL;
    }

    *raw_count = (int32_t)value;
    return HAL_OK;
}
