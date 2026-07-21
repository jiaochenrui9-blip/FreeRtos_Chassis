#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include "BMI088.h"
#include "IST8310.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_COMPLEMENTARY_TAU         0.5f
#define IMU_YAW_COMPLEMENTARY_TAU     0.5f
#define IMU_YAW_STATIONARY_TAU        0.08f
#define IMU_YAW_STATIONARY_DPS        0.5f

/* Measured while the board is placed horizontally. */
#define IMU_ROLL_TRIM_DEG             0.8f
#define IMU_PITCH_TRIM_DEG            1.2f

/* Suppress tiny residual Z-axis gyro bias after stationary calibration. */
#define IMU_YAW_GYRO_DEADBAND_DPS     0.15f

typedef struct
{
    float accel_roll;
    float accel_pitch;
    float mag_yaw;

    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;

    float fused_roll;
    float fused_pitch;
    float fused_yaw;
} IMU_Attitude_t;

void IMU_Attitude_Init(const BMI088_Data_t *imu, const IST8310_Data_t *mag);
void IMU_Attitude_Update(const BMI088_Data_t *imu,
                         const IST8310_Data_t *mag,
                         float dt_s);
void IMU_Attitude_GetData(IMU_Attitude_t *attitude);

#ifdef __cplusplus
}
#endif

#endif
