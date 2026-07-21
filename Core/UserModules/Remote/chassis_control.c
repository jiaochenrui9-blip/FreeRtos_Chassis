#include "chassis_control.h"

#include "elrs_crsf.h"
#include <stddef.h>

#include "stm32f4xx_hal.h"

static float Chassis_Normalize(int16_t value)
{
    int32_t magnitude = (value >= 0) ? value : -(int32_t)value;

    if (magnitude <= CHASSIS_RC_DEADZONE)
    {
        return 0.0f;
    }

    if (magnitude > (int32_t)CHASSIS_COMMAND_MAX)
    {
        magnitude = (int32_t)CHASSIS_COMMAND_MAX;
    }

    float normalized = ((float)magnitude - (float)CHASSIS_RC_DEADZONE) /
                       (CHASSIS_COMMAND_MAX - (float)CHASSIS_RC_DEADZONE);

    return (value >= 0) ? normalized : -normalized;
}

static void Chassis_SetStopped(Chassis_Command_t *command)
{
    command->vx = 0.0f;
    command->vy = 0.0f;
    command->wz = 0.0f;
}

static void Chassis_UpdateGear(Chassis_Command_t *command, int16_t switch_value)
{
    if (switch_value < -CHASSIS_SWITCH_THRESHOLD)
    {
        command->gear = CHASSIS_GEAR_HIGH;
        command->speed_limit = CHASSIS_HIGH_LIMIT;
    }
    else if (switch_value > CHASSIS_SWITCH_THRESHOLD)
    {
        command->gear = CHASSIS_GEAR_LOW;
        command->speed_limit = CHASSIS_LOW_LIMIT;
    }
    else
    {
        command->gear = CHASSIS_GEAR_MEDIUM;
        command->speed_limit = CHASSIS_MEDIUM_LIMIT;
    }
}

void Chassis_Control_Update(Chassis_Command_t *command)
{
    int16_t sa_value;
    int16_t sb_value;

    if (command == NULL)
    {
        return;
    }

    if (ELRS_CRSF_IsOnline() == 0u)
    {
        command->state = CHASSIS_STATE_OFFLINE;
        command->gear = CHASSIS_GEAR_LOW;
        command->speed_limit = CHASSIS_LOW_LIMIT;
        Chassis_SetStopped(command);
        return;
    }

    sa_value = ELRS_CRSF_RawToSigned(
        ELRS_CRSF_GetChannel(CHASSIS_SA_CHANNEL));
    sb_value = ELRS_CRSF_RawToSigned(
        ELRS_CRSF_GetChannel(CHASSIS_SB_CHANNEL));
    Chassis_UpdateGear(command, sb_value);

    if (sa_value >= -CHASSIS_SWITCH_THRESHOLD)
    {
        command->state = CHASSIS_STATE_LOCKED;
        Chassis_SetStopped(command);
        return;
    }

    command->state = CHASSIS_STATE_ENABLED;

    command->vx = Chassis_Normalize(
        ELRS_CRSF_RawToSigned(ELRS_CRSF_GetChannel(CHASSIS_VX_CHANNEL))) *
        command->speed_limit;
    command->vy = Chassis_Normalize(
        ELRS_CRSF_RawToSigned(ELRS_CRSF_GetChannel(CHASSIS_VY_CHANNEL))) *
        command->speed_limit;
    command->wz = Chassis_Normalize(
        ELRS_CRSF_RawToSigned(ELRS_CRSF_GetChannel(CHASSIS_WZ_CHANNEL))) *
        command->speed_limit;
}
void Chassis_CalculateMotorRPM(float motor_rpm[CHASSIS_WHEEL_COUNT],
                               const Chassis_Command_t *command)
{
    float max_abs = 0.0f;

    motor_rpm[0] = command->vx - command->vy - command->wz;
    motor_rpm[1] = command->vx + command->vy + command->wz;
    motor_rpm[2] = command->vx + command->vy - command->wz;
    motor_rpm[3] = command->vx - command->vy + command->wz;

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        float abs_speed = (motor_rpm[i] >= 0.0f) ? motor_rpm[i] : -motor_rpm[i];
        if (abs_speed > max_abs)
        {
            max_abs = abs_speed;
        }
    }

    if (max_abs > command->speed_limit)
    {
        const float scale = command->speed_limit / max_abs;
        for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            motor_rpm[i] *= scale;
        }
    }

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        motor_rpm[i] *= CHASSIS_MOTOR_MAX_RPM;
    }
}
const char *Chassis_Control_GetStateName(const Chassis_Command_t *command)
{
    if (command == NULL || command->state == CHASSIS_STATE_OFFLINE)
    {
        return "OFFLINE";
    }

    return (command->state == CHASSIS_STATE_ENABLED) ? "ENABLED" : "LOCKED";
}

const char *Chassis_Control_GetGearName(const Chassis_Command_t *command)
{
    if (command == NULL)
    {
        return "LOW";
    }

    if (command->gear == CHASSIS_GEAR_HIGH)
    {
        return "HIGH";
    }
    if (command->gear == CHASSIS_GEAR_MEDIUM)
    {
        return "MEDIUM";
    }
    return "LOW";
}

const char *Chassis_Control_GetDirection(const Chassis_Command_t *command)
{
    if (command == NULL || command->state != CHASSIS_STATE_ENABLED ||
        (command->vx == 0.0f && command->vy == 0.0f && command->wz == 0.0f))
    {
        return "STOP";
    }

    if (command->vy > 0)
    {
        if (command->vx > 0)
        {
            return "FORWARD_RIGHT";
        }
        if (command->vx < 0)
        {
            return "FORWARD_LEFT";
        }
        return "FORWARD";
    }

    if (command->vy < 0)
    {
        if (command->vx > 0)
        {
            return "BACKWARD_RIGHT";
        }
        if (command->vx < 0)
        {
            return "BACKWARD_LEFT";
        }
        return "BACKWARD";
    }

    if (command->vx > 0)
    {
        return "RIGHT";
    }
    if (command->vx < 0)
    {
        return "LEFT";
    }

    return (command->wz > 0) ? "ROTATE_RIGHT" : "ROTATE_LEFT";
}
