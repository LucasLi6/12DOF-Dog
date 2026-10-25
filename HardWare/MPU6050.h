#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"
#include <stdint.h>

// MPU6050 I2C??
#define MPU6050_ADDR        0xD0  // AD0?????,???0x68,??????0xD0

// ???????
#define MPU6050_PWR_MGMT_1  0x6B  // ???????1
#define MPU6050_PWR_MGMT_2  0x6C  // ???????2
#define MPU6050_SMPLRT_DIV  0x19  // ??????
#define MPU6050_CONFIG      0x1A  // ?????
#define MPU6050_GYRO_CONFIG 0x1B  // ????????
#define MPU6050_ACCEL_CONFIG 0x1C // ?????????
#define MPU6050_INT_ENABLE  0x38  // ???????
#define MPU6050_ACCEL_XOUT_H 0x3B // ????X????
#define MPU6050_ACCEL_XOUT_L 0x3C // ????X????
#define MPU6050_ACCEL_YOUT_H 0x3D // ????Y????
#define MPU6050_ACCEL_YOUT_L 0x3E // ????Y????
#define MPU6050_ACCEL_ZOUT_H 0x3F // ????Z????
#define MPU6050_ACCEL_ZOUT_L 0x40 // ????Z????
#define MPU6050_TEMP_OUT_H  0x41  // ????????
#define MPU6050_TEMP_OUT_L  0x42  // ????????
#define MPU6050_GYRO_XOUT_H 0x43  // ???X????
#define MPU6050_GYRO_XOUT_L 0x44  // ???X????
#define MPU6050_GYRO_YOUT_H 0x45  // ???Y????
#define MPU6050_GYRO_YOUT_L 0x46  // ???Y????
#define MPU6050_GYRO_ZOUT_H 0x47  // ???Z????
#define MPU6050_GYRO_ZOUT_L 0x48  // ???Z????

// ??????
typedef struct {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
} MPU6050_AccelData;

typedef struct {
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} MPU6050_GyroData;

typedef struct {
    float pitch;
    float roll;
    float yaw;
} MPU6050_AngleData;

typedef struct {
    MPU6050_AccelData accel;
    MPU6050_GyroData gyro;
    int16_t temp;
    float tempC;
    MPU6050_AngleData angle;
    float yawIntegral; // ??YAW?????
} MPU6050_Data;

// ????
void MPU6050_Init(void);
void MPU6050_I2C_Init(void);
void MPU6050_WriteReg(uint8_t regAddr, uint8_t data);
uint8_t MPU6050_ReadReg(uint8_t regAddr);
void MPU6050_ReadAccel(MPU6050_AccelData *accel);
void MPU6050_ReadGyro(MPU6050_GyroData *gyro);
void MPU6050_ReadTemp(int16_t *temp, float *tempC);
void MPU6050_ReadAll(MPU6050_Data *data);
void MPU6050_CalcAngle(MPU6050_Data *data);

#endif
