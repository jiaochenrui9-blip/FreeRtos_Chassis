#include "IMU_ATTITUDE.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_RAD_TO_DEG 57.295779513f
#define IMU_DEG_TO_RAD 0.01745329252f

static IMU_Attitude_t IMU_ATTITUDE_DATA;

static float IMU_Wrap180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static void IMU_CalculateAccelAngles(const BMI088_Data_t *imu,
                                     float *roll, float *pitch)
{
    float ax = imu->acc_g[0];
    float ay = imu->acc_g[1];
    float az = imu->acc_g[2];

    *roll = atan2f(ay, az) * IMU_RAD_TO_DEG + IMU_ROLL_TRIM_DEG;
    *pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * IMU_RAD_TO_DEG +
             IMU_PITCH_TRIM_DEG;
}

static float IMU_CalculateMagYaw(const IST8310_Data_t *mag,
                                 float roll_deg, float pitch_deg)
{
    float roll = roll_deg * IMU_DEG_TO_RAD;
    float pitch = pitch_deg * IMU_DEG_TO_RAD;
    float mx = mag->mag_uT[0];
    float my = mag->mag_uT[1];
    float mz = mag->mag_uT[2];
    float mx_horizontal;
    float my_horizontal;

    mx_horizontal = mx * cosf(pitch) + mz * sinf(pitch);
    my_horizontal = mx * sinf(roll) * sinf(pitch) +
                    my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

    return IMU_Wrap180(atan2f(-my_horizontal, mx_horizontal) * IMU_RAD_TO_DEG);
}
//姿态角初始化
void IMU_Attitude_Init(const BMI088_Data_t *imu, const IST8310_Data_t *mag)
{
    if (imu == NULL)
    {
        return;
    }
//成员清零
    memset(&IMU_ATTITUDE_DATA, 0, sizeof(IMU_ATTITUDE_DATA));
    IMU_CalculateAccelAngles(imu,
                             &IMU_ATTITUDE_DATA.accel_roll,
                             &IMU_ATTITUDE_DATA.accel_pitch);
//初步计算
    IMU_ATTITUDE_DATA.gyro_roll = IMU_ATTITUDE_DATA.accel_roll;
    IMU_ATTITUDE_DATA.gyro_pitch = IMU_ATTITUDE_DATA.accel_pitch;
    IMU_ATTITUDE_DATA.fused_roll = IMU_ATTITUDE_DATA.accel_roll;
    IMU_ATTITUDE_DATA.fused_pitch = IMU_ATTITUDE_DATA.accel_pitch;

    if (mag != NULL && mag->data_ready != 0u)
    {
        IMU_ATTITUDE_DATA.mag_yaw = IMU_CalculateMagYaw(
            mag, IMU_ATTITUDE_DATA.fused_roll, IMU_ATTITUDE_DATA.fused_pitch);
    }
    IMU_ATTITUDE_DATA.gyro_yaw = IMU_ATTITUDE_DATA.mag_yaw;
    IMU_ATTITUDE_DATA.fused_yaw = IMU_ATTITUDE_DATA.mag_yaw;
}
//更新各个姿态角0.0.
void IMU_Attitude_Update(const BMI088_Data_t *imu,
                         const IST8310_Data_t *mag,
                         float dt_s)
{
    float alpha;
    float yaw_alpha;
    float yaw_tau;
    float predicted_yaw;
    float gyro_z_dps;

    if (imu == NULL || dt_s <= 0.0f)
    {
        return;
    }

    if (dt_s > 0.1f)
    {
        dt_s = 0.01f;
    }

    IMU_CalculateAccelAngles(imu,
                             &IMU_ATTITUDE_DATA.accel_roll,
                             &IMU_ATTITUDE_DATA.accel_pitch);

    IMU_ATTITUDE_DATA.gyro_roll += imu->gyro_dps[0] * dt_s;
    IMU_ATTITUDE_DATA.gyro_pitch += imu->gyro_dps[1] * dt_s;
    gyro_z_dps = imu->gyro_dps[2];
    if (fabsf(gyro_z_dps) < IMU_YAW_GYRO_DEADBAND_DPS)
    {
        gyro_z_dps = 0.0f;
    }

    IMU_ATTITUDE_DATA.gyro_yaw = IMU_Wrap180(
        IMU_ATTITUDE_DATA.gyro_yaw + gyro_z_dps * dt_s);

    alpha = IMU_COMPLEMENTARY_TAU / (IMU_COMPLEMENTARY_TAU + dt_s);
    IMU_ATTITUDE_DATA.fused_roll =
        alpha * (IMU_ATTITUDE_DATA.fused_roll + imu->gyro_dps[0] * dt_s) +
        (1.0f - alpha) * IMU_ATTITUDE_DATA.accel_roll;
    IMU_ATTITUDE_DATA.fused_pitch =
        alpha * (IMU_ATTITUDE_DATA.fused_pitch + imu->gyro_dps[1] * dt_s) +
        (1.0f - alpha) * IMU_ATTITUDE_DATA.accel_pitch;

    predicted_yaw = IMU_Wrap180(
        IMU_ATTITUDE_DATA.fused_yaw + gyro_z_dps * dt_s);
    if (mag != NULL && mag->data_ready != 0u)
    {
        IMU_ATTITUDE_DATA.mag_yaw = IMU_CalculateMagYaw(
            mag, IMU_ATTITUDE_DATA.fused_roll, IMU_ATTITUDE_DATA.fused_pitch);
        yaw_tau = (fabsf(imu->gyro_dps[2]) < IMU_YAW_STATIONARY_DPS) ?
                  IMU_YAW_STATIONARY_TAU : IMU_YAW_COMPLEMENTARY_TAU;
        yaw_alpha = yaw_tau / (yaw_tau + dt_s);
        IMU_ATTITUDE_DATA.fused_yaw = IMU_Wrap180(
            predicted_yaw + (1.0f - yaw_alpha) *
            IMU_Wrap180(IMU_ATTITUDE_DATA.mag_yaw - predicted_yaw));
    }
    else
    {
        IMU_ATTITUDE_DATA.fused_yaw = predicted_yaw;
    }
}

void IMU_Attitude_GetData(IMU_Attitude_t *attitude)
{
    if (attitude != NULL)
    {
        *attitude = IMU_ATTITUDE_DATA;
    }
}
