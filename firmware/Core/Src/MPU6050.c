/*
 * mpu6050.c - FIXED VERSION for STM32F030
 */

#include "mpu6050.h"
#include <math.h>


#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG      0x75
#define PWR_MGMT_1_REG   0x6B
#define SMPLRT_DIV_REG   0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG   0x41
#define GYRO_CONFIG_REG  0x1B
#define GYRO_XOUT_H_REG  0x43
#define CONFIG_REG       0x1A  // DLPF config

// MPU6050 I2C address (7-bit << 1) — REQUIRED for STM32 HAL
#define MPU6050_ADDR (0x68 << 1)

static uint32_t timer;
static const uint16_t i2c_timeout = 100;
static const double Accel_Z_corrector = 14418.0;

// Kalman filter instances
Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t data;

    // Read WHO_AM_I register (should return 0x68)
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 0x68)
    {
        // Wake up MPU6050
        data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, i2c_timeout);

        // Sample rate = Gyro rate
        data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, i2c_timeout);

        // Digital Low Pass Filter ≈ 44 Hz (important for motors)
        data = 0x03;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, i2c_timeout);

        // Accelerometer ±2g
        data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, i2c_timeout);

        // Gyroscope ±500 deg/s
        data = 0x08;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, i2c_timeout);

        timer = HAL_GetTick();
        return 0; // success
    }
    return 1; // failure
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t recData[14];
    int16_t temp;

    // Read accel, temp, gyro in one burst
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, recData, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(recData[0] << 8 | recData[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(recData[2] << 8 | recData[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(recData[4] << 8 | recData[5]);
    temp = (int16_t)(recData[6] << 8 | recData[7]);
    DataStruct->Gyro_X_RAW  = (int16_t)(recData[8] << 8 | recData[9]);
    DataStruct->Gyro_Y_RAW  = (int16_t)(recData[10] << 8 | recData[11]);
    DataStruct->Gyro_Z_RAW  = (int16_t)(recData[12] << 8 | recData[13]);

    // Convert raw data
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)(temp / 340.0 + 36.53);

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 65.5;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 65.5;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 65.5;

    // Time difference
    double dt = (double)(HAL_GetTick() - timer) / 1000.0;
    timer = HAL_GetTick();
    if (dt <= 0) dt = 0.01;

    // Calculate roll and pitch from accelerometer
    double roll = atan2(
        DataStruct->Accel_Y_RAW,
        sqrt(DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW +
             DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW)
    ) * RAD_TO_DEG;

    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;

    // Kalman filter fusion
    DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll,  DataStruct->Gx, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1]
                        - Kalman->P[0][1]
                        - Kalman->P[1][0]
                        + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K0 = Kalman->P[0][0] / S;
    double K1 = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K0 * y;
    Kalman->bias  += K1 * y;

    double P00 = Kalman->P[0][0];
    double P01 = Kalman->P[0][1];

    Kalman->P[0][0] -= K0 * P00;
    Kalman->P[0][1] -= K0 * P01;
    Kalman->P[1][0] -= K1 * P00;
    Kalman->P[1][1] -= K1 * P01;

    return Kalman->angle;
}
