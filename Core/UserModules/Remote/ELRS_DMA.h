#ifndef ELRS_ELRS_DMA_H
#define ELRS_ELRS_DMA_H

#include "main.h"
#include <stdint.h>

void ELRS_DMA_Reset(void);
void ELRS_DMA_RxEvent(const uint8_t ElrsRx[],
                      uint16_t ElrsLen,
                      uint16_t Position);
void ELRS_DMA_UpdateSize(UART_HandleTypeDef *huart,uint16_t ElrsSize);
void ELRS_DMA_Process(void);
HAL_StatusTypeDef ELRS_DMA_Start(UART_HandleTypeDef *huart);
#endif //ELRS_ELRS_DMA_H
