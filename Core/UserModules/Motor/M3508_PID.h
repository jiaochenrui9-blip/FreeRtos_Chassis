//
// Created by game on 2026/7/21.
//

#ifndef FREERTOS_ITEM_M3508_PID_H
#define FREERTOS_ITEM_M3508_PID_H

#include "M3508.h"

#define M3508_SPEED_PID_KP              2.0f
#define M3508_SPEED_PID_KI              0.0f
#define M3508_SPEED_PID_KD              0.0f
#define M3508_SPEED_PID_INTEGRAL_LIMIT 5000.0f

void M3508_PID_Init(M3508_MotorTypeDef *motor);
HAL_StatusTypeDef M3508_PID_Process(M3508_MotorTypeDef *motor,
                                    float target_rpm,
                                    float feedback_rpm);
void M3508_PID_Stop(M3508_MotorTypeDef *motor);

#endif //FREERTOS_ITEM_M3508_PID_H
