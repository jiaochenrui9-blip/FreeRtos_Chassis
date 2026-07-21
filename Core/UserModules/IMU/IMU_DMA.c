#include "IMU_DMA.h"

#include "BMI088.h"
#include <string.h>

#define BMI088_ACC_DATA_REG       0x12u
#define BMI088_GYRO_DATA_REG      0x02u
#define BMI088_SPI_READ           0x80u

#define IMU_DMA_ACC_TRANSFER_LEN  8u
#define IMU_DMA_GYRO_TRANSFER_LEN 7u

volatile uint8_t IMU_SAMPLE_READY;
volatile uint8_t IMU_DMA_BUSY;
volatile IMU_DMA_State_t IMU_DMA_STATE = IMU_DMA_IDLE;
volatile uint32_t IMU_DMA_ERROR_COUNT;
volatile uint32_t IMU_SAMPLE_SKIP_COUNT;
volatile uint32_t IMU_SAMPLE_COUNT;

static SPI_HandleTypeDef *IMU_SPI;
static TIM_HandleTypeDef *IMU_TIMER;

static uint8_t ACC_TX[IMU_DMA_ACC_TRANSFER_LEN];
static uint8_t GYRO_TX[IMU_DMA_GYRO_TRANSFER_LEN];
static volatile uint8_t ACC_RX[IMU_DMA_ACC_TRANSFER_LEN];
static volatile uint8_t GYRO_RX[IMU_DMA_GYRO_TRANSFER_LEN];
static volatile uint8_t SAMPLE_ACC_RX[IMU_DMA_ACC_TRANSFER_LEN];
static volatile uint8_t SAMPLE_GYRO_RX[IMU_DMA_GYRO_TRANSFER_LEN];
static volatile uint32_t SAMPLE_SEQUENCE;
static uint32_t LAST_PROCESSED_SEQUENCE;
static float IMU_SAMPLE_DT = 0.001f;

static void IMU_DMA_DeselectAll(void)
{
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}

static void IMU_DMA_Fail(void)
{
    IMU_DMA_DeselectAll();
    IMU_DMA_ERROR_COUNT++;
    IMU_DMA_BUSY = 0u;
    IMU_DMA_STATE = IMU_DMA_ERROR;
}

void IMU_DMA_Init(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim)
{
    IMU_SPI = hspi;
    IMU_TIMER = htim;

    memset(ACC_TX, 0xFF, sizeof(ACC_TX));
    memset(GYRO_TX, 0xFF, sizeof(GYRO_TX));
    ACC_TX[0] = BMI088_ACC_DATA_REG | BMI088_SPI_READ;
    GYRO_TX[0] = BMI088_GYRO_DATA_REG | BMI088_SPI_READ;

    IMU_DMA_DeselectAll();
    IMU_SAMPLE_READY = 0u;
    IMU_DMA_BUSY = 0u;
    IMU_DMA_STATE = IMU_DMA_IDLE;
    IMU_DMA_ERROR_COUNT = 0u;
    IMU_SAMPLE_SKIP_COUNT = 0u;
    IMU_SAMPLE_COUNT = 0u;
    SAMPLE_SEQUENCE = 0u;
    LAST_PROCESSED_SEQUENCE = 0u;
    IMU_SAMPLE_DT = 0.001f;
    
}

void IMU_DMA_StartTimer(void)
{
    if (IMU_TIMER != NULL)
    {
        (void)HAL_TIM_Base_Start_IT(IMU_TIMER);
    }
}

HAL_StatusTypeDef IMU_DMA_StartSample(void)
{
    HAL_StatusTypeDef status;

    if (IMU_SPI == NULL)
    {
        IMU_DMA_ERROR_COUNT++;
        return HAL_ERROR;
    }

    if (IMU_DMA_STATE == IMU_DMA_ERROR && IMU_DMA_BUSY == 0u)
    {
        IMU_DMA_STATE = IMU_DMA_IDLE;
    }

    if (IMU_DMA_BUSY != 0u || IMU_DMA_STATE != IMU_DMA_IDLE)
    {
        IMU_SAMPLE_SKIP_COUNT++;
        return HAL_BUSY;
    }

    IMU_DMA_BUSY = 1u;
    IMU_DMA_STATE = IMU_DMA_READING_ACC;
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_RESET);

    status = HAL_SPI_TransmitReceive_DMA(
        IMU_SPI, ACC_TX, (uint8_t *)ACC_RX, IMU_DMA_ACC_TRANSFER_LEN);
    if (status != HAL_OK)
    {
        IMU_DMA_Fail();
    }
    return status;
}

uint8_t IMU_DMA_IsSampleReady(void)
{
    return IMU_SAMPLE_READY;
}

void IMU_DMA_ClearSampleReady(void)
{
    IMU_SAMPLE_READY = 0u;
}

void IMU_DMA_ProcessSample(BMI088_Data_t *data)
{
    uint8_t acc_copy[IMU_DMA_ACC_TRANSFER_LEN];
    uint8_t gyro_copy[IMU_DMA_GYRO_TRANSFER_LEN];
    int16_t acc_raw[3];
    int16_t gyro_raw[3];
    uint32_t primask;
    uint32_t sequence;
    uint32_t sample_delta;

    if (IMU_SAMPLE_READY == 0u)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0u; i < IMU_DMA_ACC_TRANSFER_LEN; i++)
    {
        acc_copy[i] = SAMPLE_ACC_RX[i];
    }
    for (uint8_t i = 0u; i < IMU_DMA_GYRO_TRANSFER_LEN; i++)
    {
        gyro_copy[i] = SAMPLE_GYRO_RX[i];
    }
    sequence = SAMPLE_SEQUENCE;
    IMU_SAMPLE_READY = 0u;
    if (primask == 0u)
    {
        __enable_irq();
    }

    acc_raw[0] = (int16_t)(((uint16_t)acc_copy[3] << 8u) | acc_copy[2]);
    acc_raw[1] = (int16_t)(((uint16_t)acc_copy[5] << 8u) | acc_copy[4]);
    acc_raw[2] = (int16_t)(((uint16_t)acc_copy[7] << 8u) | acc_copy[6]);

    gyro_raw[0] = (int16_t)(((uint16_t)gyro_copy[2] << 8u) | gyro_copy[1]);
    gyro_raw[1] = (int16_t)(((uint16_t)gyro_copy[4] << 8u) | gyro_copy[3]);
    gyro_raw[2] = (int16_t)(((uint16_t)gyro_copy[6] << 8u) | gyro_copy[5]);

    sample_delta = sequence - LAST_PROCESSED_SEQUENCE;
    IMU_SAMPLE_DT = (LAST_PROCESSED_SEQUENCE == 0u || sample_delta == 0u) ?
                    0.001f : (float)sample_delta * 0.001f;
    if (IMU_SAMPLE_DT > 0.1f)
    {
        IMU_SAMPLE_DT = 0.1f;
    }
    LAST_PROCESSED_SEQUENCE = sequence;

    (void)BMI088_ProcessRawSample(acc_raw, gyro_raw);
    BMI088_GetData(data);
}

float IMU_DMA_GetDt(void)
{
    return IMU_SAMPLE_DT;
}

uint32_t IMU_DMA_GetErrorCount(void)
{
    return IMU_DMA_ERROR_COUNT;
}

uint32_t IMU_DMA_GetSkipCount(void)
{
    return IMU_SAMPLE_SKIP_COUNT;
}

uint32_t IMU_DMA_GetSampleCount(void)
{
    return IMU_SAMPLE_COUNT;
}

void IMU_DMA_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef status;

    if (IMU_SPI == NULL || hspi->Instance != IMU_SPI->Instance)
    {
        return;
    }

    if (IMU_DMA_STATE == IMU_DMA_READING_ACC)
    {
        HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
        IMU_DMA_STATE = IMU_DMA_READING_GYRO;
        HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin,
                          GPIO_PIN_RESET);
        status = HAL_SPI_TransmitReceive_DMA(
            IMU_SPI, GYRO_TX, (uint8_t *)GYRO_RX, IMU_DMA_GYRO_TRANSFER_LEN);
        if (status != HAL_OK)
        {
            IMU_DMA_Fail();
        }
    }
    else if (IMU_DMA_STATE == IMU_DMA_READING_GYRO)
    {
        HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin,
                          GPIO_PIN_SET);
        IMU_DMA_STATE = IMU_DMA_DONE;
        IMU_SAMPLE_COUNT++;
        if (IMU_SAMPLE_READY == 0u)
        {
            for (uint8_t i = 0u; i < IMU_DMA_ACC_TRANSFER_LEN; i++)
            {
                SAMPLE_ACC_RX[i] = ACC_RX[i];
            }
            for (uint8_t i = 0u; i < IMU_DMA_GYRO_TRANSFER_LEN; i++)
            {
                SAMPLE_GYRO_RX[i] = GYRO_RX[i];
            }
            SAMPLE_SEQUENCE = IMU_SAMPLE_COUNT;
            __DMB();
            IMU_SAMPLE_READY = 1u;
        }
        else
        {
            IMU_SAMPLE_SKIP_COUNT++;
        }
        IMU_DMA_BUSY = 0u;
        IMU_DMA_STATE = IMU_DMA_IDLE;
    }
    else
    {
        IMU_DMA_Fail();
    }
}

void IMU_DMA_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (IMU_SPI != NULL && hspi->Instance == IMU_SPI->Instance)
    {
        IMU_DMA_Fail();
    }
}
