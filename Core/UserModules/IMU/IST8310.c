#include "IST8310.h"

#include <stddef.h>
#include <string.h>

#define IST8310_I2C_ADDRESS       (0x0Eu << 1u)
#define IST8310_WHO_AM_I_REG      0x00u
#define IST8310_WHO_AM_I_VALUE    0x10u
#define IST8310_STAT1_REG         0x02u
#define IST8310_DATA_REG          0x03u
#define IST8310_CNTL1_REG         0x0Au
#define IST8310_CNTL2_REG         0x0Bu
#define IST8310_AVGCNTL_REG       0x41u
#define IST8310_PDCNTL_REG        0x42u

#define IST8310_DRDY_BIT          0x01u
#define IST8310_SINGLE_MEASURE    0x01u
#define IST8310_DRDY_ENABLE       0x08u
#define IST8310_AVERAGE_16        0x24u
#define IST8310_PULSE_NORMAL      0xC0u
#define IST8310_UT_PER_LSB        0.3f
#define IST8310_I2C_TIMEOUT_MS    100u

static I2C_HandleTypeDef *IST8310_I2C;
static IST8310_Data_t IST8310_DATA;

static uint8_t IST8310_ReadRegs(uint8_t reg, uint8_t *data, uint16_t length)
{
    if (IST8310_I2C == NULL || data == NULL || length == 0u)
    {
        return IST8310_ERR_NULL;
    }

    return (HAL_I2C_Mem_Read(IST8310_I2C, IST8310_I2C_ADDRESS,
                             reg, I2C_MEMADD_SIZE_8BIT,
                             data, length, IST8310_I2C_TIMEOUT_MS) == HAL_OK) ?
           IST8310_OK : IST8310_ERR_I2C;
}

static uint8_t IST8310_WriteReg(uint8_t reg, uint8_t data)
{
    if (IST8310_I2C == NULL)
    {
        return IST8310_ERR_NULL;
    }

    return (HAL_I2C_Mem_Write(IST8310_I2C, IST8310_I2C_ADDRESS,
                              reg, I2C_MEMADD_SIZE_8BIT,
                              &data, 1u, IST8310_I2C_TIMEOUT_MS) == HAL_OK) ?
           IST8310_OK : IST8310_ERR_I2C;
}

static uint8_t IST8310_WriteAndVerify(uint8_t reg, uint8_t value)
{
    uint8_t readback;
    uint8_t status = IST8310_WriteReg(reg, value);

    if (status != IST8310_OK)
    {
        return status;
    }
    HAL_Delay(1u);
    status = IST8310_ReadRegs(reg, &readback, 1u);
    if (status != IST8310_OK)
    {
        return status;
    }
    return (readback == value) ? IST8310_OK : IST8310_ERR_CONFIG;
}

uint8_t IST8310_ReadID(uint8_t *id)
{
    uint8_t status;

    if (id == NULL)
    {
        return IST8310_ERR_NULL;
    }

    status = IST8310_ReadRegs(IST8310_WHO_AM_I_REG, id, 1u);
    if (status == IST8310_OK)
    {
        IST8310_DATA.id = *id;
    }
    return status;
}

uint8_t IST8310_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t id;
    uint8_t status;

    if (hi2c == NULL)
    {
        return IST8310_ERR_NULL;
    }

    IST8310_I2C = hi2c;
    memset(&IST8310_DATA, 0, sizeof(IST8310_DATA));

    HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_RESET);
    HAL_Delay(50u);
    HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_SET);
    HAL_Delay(50u);

    status = IST8310_ReadID(&id);
    if (status != IST8310_OK)
    {
        return status;
    }
    if (id != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_ERR_ID;
    }

    status = IST8310_WriteAndVerify(IST8310_CNTL2_REG, IST8310_DRDY_ENABLE);
    if (status != IST8310_OK)
    {
        return status;
    }
    status = IST8310_WriteAndVerify(IST8310_AVGCNTL_REG, IST8310_AVERAGE_16);
    if (status != IST8310_OK)
    {
        return status;
    }
    status = IST8310_WriteAndVerify(IST8310_PDCNTL_REG, IST8310_PULSE_NORMAL);
    if (status != IST8310_OK)
    {
        return status;
    }

    status = IST8310_WriteReg(IST8310_CNTL1_REG, IST8310_SINGLE_MEASURE);
    if (status != IST8310_OK)
    {
        return status;
    }

    IST8310_DATA.initialized = 1u;
    return IST8310_OK;
}

uint8_t IST8310_Update(void)
{
    uint8_t status_reg;
    uint8_t buffer[6];
    uint8_t status;

    if (IST8310_DATA.initialized == 0u)
    {
        return IST8310_ERR_NOT_INIT;
    }

    status = IST8310_ReadRegs(IST8310_STAT1_REG, &status_reg, 1u);
    if (status != IST8310_OK)
    {
        return status;
    }
    if ((status_reg & IST8310_DRDY_BIT) == 0u)
    {
        IST8310_DATA.data_ready = 0u;
        return IST8310_NOT_READY;
    }

    status = IST8310_ReadRegs(IST8310_DATA_REG, buffer, sizeof(buffer));
    if (status != IST8310_OK)
    {
        return status;
    }

    IST8310_DATA.raw[0] = (int16_t)(((uint16_t)buffer[1] << 8u) | buffer[0]);
    IST8310_DATA.raw[1] = (int16_t)(((uint16_t)buffer[3] << 8u) | buffer[2]);
    IST8310_DATA.raw[2] = (int16_t)(((uint16_t)buffer[5] << 8u) | buffer[4]);
    for (uint8_t i = 0u; i < 3u; i++)
    {
        IST8310_DATA.mag_uT[i] =
            ((float)IST8310_DATA.raw[i] * IST8310_UT_PER_LSB) -
            IST8310_DATA.offset_uT[i];
    }
    IST8310_DATA.data_ready = 1u;

    status = IST8310_WriteReg(IST8310_CNTL1_REG, IST8310_SINGLE_MEASURE);
    return status;
}

void IST8310_GetData(IST8310_Data_t *data)
{
    if (data != NULL)
    {
        *data = IST8310_DATA;
    }
}

void IST8310_SetOffset(float x_uT, float y_uT, float z_uT)
{
    IST8310_DATA.offset_uT[0] = x_uT;
    IST8310_DATA.offset_uT[1] = y_uT;
    IST8310_DATA.offset_uT[2] = z_uT;
}
