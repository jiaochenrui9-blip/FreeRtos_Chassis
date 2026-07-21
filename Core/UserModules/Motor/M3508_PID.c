//
// Created by game on 2026/7/21.
//
#include "M3508_PID.h"

void M3508_PID_Init(M3508_MotorTypeDef *motor)
{
    if (motor == NULL)
    {
        return;
    }

    PID_Init(&motor->MotorPID,
             M3508_SPEED_PID_KP,
             M3508_SPEED_PID_KI,
             M3508_SPEED_PID_KD,
             (float)M3508_CURRENT_MIN,
             (float)M3508_CURRENT_MAX,
             -M3508_SPEED_PID_INTEGRAL_LIMIT,
             M3508_SPEED_PID_INTEGRAL_LIMIT);
}

HAL_StatusTypeDef M3508_PID_Process(M3508_MotorTypeDef *motor,
                                    float target_rpm,
                                    float feedback_rpm)
{
    float current;

    if (motor == NULL)
    {
        return HAL_ERROR;
    }

    PID_SetTarget(&motor->MotorPID, target_rpm);
    current = PID_Calculate(&motor->MotorPID, feedback_rpm);
    return M3508_Motor_SetCurrent(motor, (int16_t)current);
}

void M3508_PID_Stop(M3508_MotorTypeDef *motor)
{
    if (motor == NULL)
    {
        return;
    }

    PID_SetTarget(&motor->MotorPID, 0.0f);
    PID_Reset(&motor->MotorPID);
    M3508_Motor_SetCurrent(motor, 0);
}
