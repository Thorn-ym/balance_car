#include "balance_car/i2c_bus.h"

static I2C_HandleTypeDef s_hi2c1;
static uint8_t s_i2c1_initialized;

HAL_StatusTypeDef BalanceI2C1_Init(void)
{
    if (s_i2c1_initialized != 0U) {
        return HAL_OK;
    }

    s_hi2c1.Instance = I2C1;
    s_hi2c1.Init.ClockSpeed = 400000U;
    s_hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    s_hi2c1.Init.OwnAddress1 = 0U;
    s_hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    s_hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    s_hi2c1.Init.OwnAddress2 = 0U;
    s_hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    s_hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&s_hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }

    s_i2c1_initialized = 1U;
    return HAL_OK;
}

I2C_HandleTypeDef *BalanceI2C1_GetHandle(void)
{
    return &s_hi2c1;
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    if (hi2c->Instance == I2C1) {
        __HAL_RCC_AFIO_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();
        __HAL_AFIO_REMAP_I2C1_ENABLE();

        gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        HAL_NVIC_SetPriority(I2C1_EV_IRQn, 4U, 0U);
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_SetPriority(I2C1_ER_IRQn, 4U, 0U);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    } else {
        return;
    }

    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}
