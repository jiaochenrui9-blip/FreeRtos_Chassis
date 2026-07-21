/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088.h"
#include "chassis_control.h"
#include "ELRS_DMA.h"
#include "usart.h"
#include "IMU_DMA.h"
#include "IMU_ATTITUDE.h"
#include "IST8310.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "M3508.h"
#include "M3508_PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern M3508_ManagerTypeDef g_m3508_manager;
extern M3508_MotorTypeDef g_chassis_motors[4];
/* USER CODE END Variables */
/* Definitions for ELRS */
osThreadId_t ELRSHandle;
const osThreadAttr_t ELRS_attributes = {
  .name = "ELRS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BMI088_Parse */
osThreadId_t BMI088_ParseHandle;
const osThreadAttr_t BMI088_Parse_attributes = {
  .name = "BMI088_Parse",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Chassis_Remote */
osThreadId_t Chassis_RemoteHandle;
const osThreadAttr_t Chassis_Remote_attributes = {
  .name = "Chassis_Remote",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ELRSDataQuene */
osMessageQueueId_t ELRSDataQueneHandle;
const osMessageQueueAttr_t ELRSDataQuene_attributes = {
  .name = "ELRSDataQuene"
};
/* Definitions for ELRSSem */
osSemaphoreId_t ELRSSemHandle;
const osSemaphoreAttr_t ELRSSem_attributes = {
  .name = "ELRSSem"
};
/* Definitions for IMUSem */
osSemaphoreId_t IMUSemHandle;
const osSemaphoreAttr_t IMUSem_attributes = {
  .name = "IMUSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ELRS_Parse(void *argument);
void BMI088Task(void *argument);
void ChassisTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ELRSSem */
  ELRSSemHandle = osSemaphoreNew(1, 1, &ELRSSem_attributes);

  /* creation of IMUSem */
  IMUSemHandle = osSemaphoreNew(1, 1, &IMUSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ELRSDataQuene */
  ELRSDataQueneHandle = osMessageQueueNew (16, sizeof(Chassis_Command_t), &ELRSDataQuene_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ELRS */
  ELRSHandle = osThreadNew(ELRS_Parse, NULL, &ELRS_attributes);

  /* creation of BMI088_Parse */
  BMI088_ParseHandle = osThreadNew(BMI088Task, NULL, &BMI088_Parse_attributes);

  /* creation of Chassis_Remote */
  Chassis_RemoteHandle = osThreadNew(ChassisTask, NULL, &Chassis_Remote_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ELRS_Parse */
/**
  * @brief  Function implementing the ELRS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ELRS_Parse */
void ELRS_Parse(void *argument)
{
  /* USER CODE BEGIN ELRS_Parse */

  ELRS_DMA_Start(&huart1);
  Chassis_Command_t command;

  /* Infinite loop */
  for(;;)
  {
    if (osSemaphoreAcquire(ELRSSemHandle,osWaitForever) == osOK)
    {
      ELRS_DMA_Process();
      Chassis_Control_Update(&command);
      osMessageQueuePut(ELRSDataQueneHandle,&command,0,0);
    }
  }
  /* USER CODE END ELRS_Parse */
}

/* USER CODE BEGIN Header_BMI088Task */
/**
* @brief Function implementing the BMI088_Parse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMI088Task */
void BMI088Task(void *argument)
{
  /* USER CODE BEGIN BMI088Task */
  BMI088_Data_t bmi088_data;
  IST8310_Data_t ist8310_data;
  IMU_Attitude_t imu_attitude_data;
  uint32_t mag_Tick = 0;

  BMI088_Init(&hspi1);
  BMI088_Update();
  BMI088_GetData(&bmi088_data);

  IST8310_Init(&hi2c3);
  osDelay(10);
  IST8310_Update();
  IST8310_GetData(&ist8310_data);

  IMU_Attitude_Init(&bmi088_data,&ist8310_data);
  IMU_Attitude_Update(&bmi088_data,&ist8310_data,1);

  IMU_DMA_Init(&hspi1,&htim8);
  IMU_DMA_StartTimer();
  mag_Tick = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(IMUSemHandle,osWaitForever);
    IMU_DMA_ProcessSample(&bmi088_data);

    if (HAL_GetTick() - mag_Tick >= 10U)
    {
      mag_Tick = HAL_GetTick();
      IST8310_Update();
      IST8310_GetData(&ist8310_data);
    }

    IMU_Attitude_Update(&bmi088_data,&ist8310_data,IMU_DMA_GetDt());
    IMU_Attitude_GetData(&imu_attitude_data);

  }
  /* USER CODE END BMI088Task */
}

/* USER CODE BEGIN Header_ChassisTask */
/**
* @brief Function implementing the Chassis_Remote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisTask */
//2ms更新一次目标电流，控制PID
void ChassisTask(void *argument)
{
  /* USER CODE BEGIN ChassisTask */
  Chassis_Command_t command = {0};
  IMU_Attitude_t imu_attitude_data = {0};
  M3508_FeedbackTypeDef motor_feedback[CHASSIS_WHEEL_COUNT] = {0};
  float target_rpm[CHASSIS_WHEEL_COUNT] = {0};
  TickType_t last_wake_time = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    while (osMessageQueueGet(ELRSDataQueneHandle,&command,0,0) == osOK)
    {
      // 等待最新的command
    }
    IMU_Attitude_GetData(&imu_attitude_data);
    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
      M3508_Motor_GetFeedback(&g_chassis_motors[i], &motor_feedback[i]);
    }

    if (command.state == CHASSIS_STATE_ENABLED)
    {
      Chassis_CalculateMotorRPM(target_rpm, &command);
      for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
      {
        M3508_PID_Process(&g_chassis_motors[i],
                          target_rpm[i],
                          (float)motor_feedback[i].speed_rpm);
      }
    }
    else
    {
      for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
      {
        target_rpm[i] = 0.0f;
        M3508_PID_Stop(&g_chassis_motors[i]);
      }
    }

    M3508_Manager_SendCurrents(&g_m3508_manager);
    vTaskDelayUntil(&last_wake_time,2);
  }
  /* USER CODE END ChassisTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

