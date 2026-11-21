#include "mpu6050.h"
#include "main.h"

#include <math.h>

extern I2C_HandleTypeDef hi2c1;

/* Each step consumes the result of the preceding interrupt-driven transfer. */
static enum { PROBE, CHECK_ID, SET_RANGES, SET_INTERRUPT, WAIT_READY, READ_SAMPLE } step;
static uint16_t address = 0x68u << 1;
static uint32_t transfer_ms;
static volatile uint32_t edge_ms;
static uint32_t sample_ms;
static uint8_t data[15]; /* INT_STATUS, accel XYZ, temperature, gyro XYZ */

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == IMU_INT_Pin) edge_ms = HAL_GetTick();
}

static void retry_probe(uint32_t now_ms)
{
    __HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
    HAL_I2C_Init(&hi2c1); /* F1 HAL resets the peripheral before reapplying its configuration. */
    address ^= 2u; /* AD0 selects 7-bit address 0x68 or 0x69. */
    step = PROBE;
    transfer_ms = now_ms;
}

static int32_t signed_word(const uint8_t *p)
{
    int32_t value = (int32_t)p[0] * 256 + p[1];
    return value >= 32768 ? value - 65536 : value;
}

bool mpu6050_poll(uint32_t now_ms, BaizeImu *sample)
{
    static uint8_t wake = 1;                    /* X gyro PLL, awake */
    static uint8_t ranges[] = {19, 4, 0, 0};   /* 1 kHz / 20, DLPF 4, ±250 °/s, ±2 g */
    static uint8_t interrupt[] = {0, 1};       /* Active-high pulse, DATA_RDY */
    uint32_t elapsed = now_ms - transfer_ms;
    if (step == PROBE && elapsed < 250u) return false;

    /* The F1 HAL IT entry points spin on BUSY. Only submit on an idle bus. */
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY || __HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {
        if (elapsed >= 5u) retry_probe(now_ms);
        return false;
    }
    if (step != PROBE && HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
        retry_probe(now_ms);
        return false;
    }

    HAL_StatusTypeDef result = HAL_ERROR;
    switch (step) {
    case PROBE:
        result = HAL_I2C_Mem_Read_IT(&hi2c1, address, 0x75, I2C_MEMADD_SIZE_8BIT, data, 1);
        step = CHECK_ID;
        break;
    case CHECK_ID:
        if (data[0] != 0x68) {
            retry_probe(now_ms);
            return false;
        }
        result = HAL_I2C_Mem_Write_IT(&hi2c1, address, 0x6B, I2C_MEMADD_SIZE_8BIT, &wake, 1);
        step = SET_RANGES;
        break;
    case SET_RANGES:
        if (elapsed < 50u) return false; /* Allow the gyro to settle after wake. */
        result = HAL_I2C_Mem_Write_IT(&hi2c1, address, 0x19, I2C_MEMADD_SIZE_8BIT, ranges, sizeof ranges);
        step = SET_INTERRUPT;
        break;
    case SET_INTERRUPT:
        result = HAL_I2C_Mem_Write_IT(&hi2c1, address, 0x37, I2C_MEMADD_SIZE_8BIT, interrupt, sizeof interrupt);
        sample_ms = edge_ms;
        step = WAIT_READY;
        break;
    case WAIT_READY: {
        uint32_t latest = edge_ms;
        if (latest == sample_ms) {
            if (elapsed >= 100u) retry_probe(now_ms);
            return false;
        }
        sample_ms = latest;
        result = HAL_I2C_Mem_Read_IT(&hi2c1, address, 0x3A, I2C_MEMADD_SIZE_8BIT, data, sizeof data);
        step = READ_SAMPLE;
        break;
    }
    case READ_SAMPLE:
        step = WAIT_READY;
        if (!(data[0] & 1u) || edge_ms != sample_ms) return false;
        sample->sample_ms = sample_ms;
        for (size_t i = 0; i < 3; ++i) {
            int32_t accel = signed_word(&data[1 + 2 * i]);
            int32_t gyro = signed_word(&data[9 + 2 * i]);
            sample->acceleration_mm_s2[i] = (int32_t)lroundf((float)accel * (9806.65f / 16384.0f));
            sample->gyro_mrad_s[i] = (int32_t)lroundf((float)gyro * (17.45329252f / 131.0f));
        }
        return true;
    }
    if (result != HAL_OK) retry_probe(now_ms);
    else transfer_ms = now_ms;
    return false;
}
