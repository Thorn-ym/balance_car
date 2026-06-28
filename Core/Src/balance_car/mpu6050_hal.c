#include "balance_car/mpu6050_hal.h"

#include "balance_car/i2c_bus.h"

#define MPU6050_ADDR              (0x68U << 1)
#define MPU6050_WHO_AM_I          0x75U
#define MPU6050_PWR_MGMT_1        0x6BU
#define MPU6050_PWR_MGMT_2        0x6CU
#define MPU6050_SMPLRT_DIV        0x19U
#define MPU6050_CONFIG            0x1AU
#define MPU6050_GYRO_CONFIG       0x1BU
#define MPU6050_ACCEL_CONFIG      0x1CU
#define MPU6050_ACCEL_XOUT_H      0x3BU
#define MPU6050_RAW_DATA_LEN      14U
#define MPU6050_ASYNC_TIMEOUT_MS  20U
#define MPU6050_STALE_TIMEOUT_MS  60U

static uint8_t s_last_id;
static volatile uint8_t s_async_busy;
static volatile uint8_t s_sample_valid;
static volatile uint32_t s_async_start_tick;
static volatile uint32_t s_last_sample_tick;
static volatile uint32_t s_async_timeout_count;
static volatile uint32_t s_rx_complete_count;
static volatile uint32_t s_rx_error_count;
static uint8_t s_async_data[MPU6050_RAW_DATA_LEN];
static Mpu6050Raw_t s_last_raw;

static void Mpu6050_ParseRaw(const uint8_t *data, Mpu6050Raw_t *raw)
{
    raw->ax = (int16_t)((uint16_t)data[0] << 8 | data[1]);
    raw->ay = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    raw->az = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    raw->gx = (int16_t)((uint16_t)data[8] << 8 | data[9]);
    raw->gy = (int16_t)((uint16_t)data[10] << 8 | data[11]);
    raw->gz = (int16_t)((uint16_t)data[12] << 8 | data[13]);
}

static HAL_StatusTypeDef Mpu6050_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(BalanceI2C1_GetHandle(), MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1U, 10U);
}

static HAL_StatusTypeDef Mpu6050_ReadReg(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(BalanceI2C1_GetHandle(), MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1U, 10U);
}

HAL_StatusTypeDef Mpu6050_Init(void)
{
    if (BalanceI2C1_Init() != HAL_OK) {
        return HAL_ERROR;
    }

    if (Mpu6050_ReadReg(MPU6050_WHO_AM_I, &s_last_id) != HAL_OK) {
        return HAL_ERROR;
    }
    if (s_last_id != 0x68U) {
        return HAL_ERROR;
    }

    if (Mpu6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (Mpu6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (Mpu6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (Mpu6050_WriteReg(MPU6050_CONFIG, 0x00U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (Mpu6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (Mpu6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18U) != HAL_OK) {
        return HAL_ERROR;
    }

    uint8_t data[MPU6050_RAW_DATA_LEN];
    if (HAL_I2C_Mem_Read(BalanceI2C1_GetHandle(), MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 10U) != HAL_OK) {
        return HAL_ERROR;
    }

    Mpu6050_ParseRaw(data, &s_last_raw);
    s_sample_valid = 1U;
    s_last_sample_tick = HAL_GetTick();
    s_async_busy = 0U;

    return HAL_OK;
}

HAL_StatusTypeDef Mpu6050_ReadRaw(Mpu6050Raw_t *raw)
{
    if (raw == NULL) {
        return HAL_ERROR;
    }

    if (s_async_busy != 0U && (uint32_t)(HAL_GetTick() - s_async_start_tick) > MPU6050_ASYNC_TIMEOUT_MS) {
        s_async_timeout_count++;
        s_async_busy = 0U;
    }

    if (s_async_busy == 0U && HAL_I2C_GetState(BalanceI2C1_GetHandle()) == HAL_I2C_STATE_READY) {
        s_async_busy = 1U;
        s_async_start_tick = HAL_GetTick();
        if (HAL_I2C_Mem_Read_IT(BalanceI2C1_GetHandle(), MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                                I2C_MEMADD_SIZE_8BIT, s_async_data, sizeof(s_async_data)) == HAL_OK) {
        } else {
            s_async_busy = 0U;
            s_rx_error_count++;
        }
    }

    if (s_sample_valid == 0U ||
        (uint32_t)(HAL_GetTick() - s_last_sample_tick) > MPU6050_STALE_TIMEOUT_MS) {
        return HAL_ERROR;
    }

    __disable_irq();
    *raw = s_last_raw;
    __enable_irq();

    return HAL_OK;
}

uint8_t Mpu6050_GetLastId(void)
{
    return s_last_id;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1 || s_async_busy == 0U) {
        return;
    }

    Mpu6050_ParseRaw(s_async_data, &s_last_raw);
    s_rx_complete_count++;
    s_last_sample_tick = HAL_GetTick();
    s_sample_valid = 1U;
    s_async_busy = 0U;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1 || s_async_busy == 0U) {
        return;
    }

    s_rx_error_count++;
    s_async_busy = 0U;
}
