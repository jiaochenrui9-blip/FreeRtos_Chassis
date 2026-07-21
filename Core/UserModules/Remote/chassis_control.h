#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_WZ_CHANNEL        2u
#define CHASSIS_VY_CHANNEL        3u
#define CHASSIS_VX_CHANNEL        4u
#define CHASSIS_SA_CHANNEL        5u
#define CHASSIS_SB_CHANNEL        6u
#define CHASSIS_RC_DEADZONE      80
#define CHASSIS_SWITCH_THRESHOLD 500
#define CHASSIS_COMMAND_MAX    1000.0f
#define CHASSIS_HIGH_LIMIT        1.0f
#define CHASSIS_MEDIUM_LIMIT      0.65f
#define CHASSIS_LOW_LIMIT         0.35f
#define CHASSIS_MOTOR_MAX_RPM  8000.0f
#define CHASSIS_WHEEL_COUNT         4U

typedef enum
{
    CHASSIS_STATE_OFFLINE = 0,
    CHASSIS_STATE_LOCKED,
    CHASSIS_STATE_ENABLED
} Chassis_State_t;

typedef enum
{
    CHASSIS_GEAR_LOW = 0,
    CHASSIS_GEAR_MEDIUM,
    CHASSIS_GEAR_HIGH
} Chassis_Gear_t;

typedef struct
{
    Chassis_State_t state;
    Chassis_Gear_t gear;
    float speed_limit;
    float vx;
    float vy;
    float wz;
} Chassis_Command_t;

void Chassis_Control_Update(Chassis_Command_t *command);
void Chassis_CalculateMotorRPM(float motor_rpm[CHASSIS_WHEEL_COUNT],
                               const Chassis_Command_t *command);
const char *Chassis_Control_GetStateName(const Chassis_Command_t *command);
const char *Chassis_Control_GetGearName(const Chassis_Command_t *command);
const char *Chassis_Control_GetDirection(const Chassis_Command_t *command);

#ifdef __cplusplus
}
#endif

#endif
