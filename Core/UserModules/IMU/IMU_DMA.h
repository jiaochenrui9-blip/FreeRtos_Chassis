#ifndef IMU_DMA_H
#define IMU_DMA_H

#include "main.h"
#include "BMI088.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    IMU_DMA_IDLE = 0,
    IMU_DMA_READING_ACC,
    IMU_DMA_READING_GYRO,
    IMU_DMA_DONE,
    IMU_DMA_ERROR
} IMU_DMA_State_t;

extern volatile uint8_t IMU_SAMPLE_READY;
extern volatile uint8_t IMU_DMA_BUSY;
extern volatile IMU_DMA_State_t IMU_DMA_STATE;
extern volatile uint32_t IMU_DMA_ERROR_COUNT;
extern volatile uint32_t IMU_SAMPLE_SKIP_COUNT;
extern volatile uint32_t IMU_SAMPLE_COUNT;

void IMU_DMA_Init(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim);
void IMU_DMA_StartTimer(void);
HAL_StatusTypeDef IMU_DMA_StartSample(void);
uint8_t IMU_DMA_IsSampleReady(void);
void IMU_DMA_ClearSampleReady(void);
void IMU_DMA_ProcessSample(BMI088_Data_t *data);
float IMU_DMA_GetDt(void);
uint32_t IMU_DMA_GetErrorCount(void);
uint32_t IMU_DMA_GetSkipCount(void);
uint32_t IMU_DMA_GetSampleCount(void);

void IMU_DMA_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void IMU_DMA_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif
