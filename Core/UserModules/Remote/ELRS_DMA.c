#include "ELRS_DMA.h"

#include "elrs_crsf.h"
#define ELRS_MAX_COUNT 256
static uint16_t ElrsLastPos;
static uint16_t ElrsPendingPos;
static UART_HandleTypeDef *Elrshuart;
static uint8_t ElrsRx[ELRS_MAX_COUNT];

void ELRS_DMA_Reset(void)
{
    ElrsLastPos = 0u;
}
HAL_StatusTypeDef ELRS_DMA_Start(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    Elrshuart = huart;
    ElrsLastPos = 0u;
    ElrsPendingPos = 0u;

    return HAL_UARTEx_ReceiveToIdle_DMA(
        huart,
        ElrsRx,
        ELRS_MAX_COUNT
    );
}
void ELRS_DMA_RxEvent(const uint8_t ElrsRx[],
                      uint16_t ElrsLen,
                      uint16_t Position)
{
    if (ElrsRx == NULL || ElrsLen == 0u || Position > ElrsLen)
    {
        return;
    }

    if (Position == ElrsLastPos)
    {
        return;
    }

    if (Position > ElrsLastPos)
    {
        for (uint16_t I = ElrsLastPos; I < Position; I++)
        {
            ELRS_CRSF_UART_RxCallback(ElrsRx[I]);
        }
    }
    else
    {
        for (uint16_t I = ElrsLastPos; I < ElrsLen; I++)
        {
            ELRS_CRSF_UART_RxCallback(ElrsRx[I]);
        }

        for (uint16_t I = 0u; I < Position; I++)
        {
            ELRS_CRSF_UART_RxCallback(ElrsRx[I]);
        }
    }

    ElrsLastPos = (Position == ElrsLen) ? 0u : Position;
}

void ELRS_DMA_Process(void)
{

    ELRS_DMA_RxEvent(ElrsRx,ELRS_MAX_COUNT,ElrsPendingPos);

}
void ELRS_DMA_UpdateSize(UART_HandleTypeDef *huart,uint16_t Size)
{
    if (huart == NULL || Size == 0u)
    {
        return;
    }
    if (huart != Elrshuart)
    {
        return;
    }
    ElrsPendingPos = Size;
}