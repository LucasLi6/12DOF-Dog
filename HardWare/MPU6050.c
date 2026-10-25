#include "MPU6050.h"
#include "Delay.h"
#include <math.h>

// ??I2C????(PB3-SCL, PB4-SDA)
#define I2C_SCL_PIN   GPIO_Pin_3
#define I2C_SDA_PIN   GPIO_Pin_4
#define I2C_GPIO_PORT GPIOB

// ?????
#define I2C_SCL_HIGH()   GPIO_SetBits(I2C_GPIO_PORT, I2C_SCL_PIN)
#define I2C_SCL_LOW()    GPIO_ResetBits(I2C_GPIO_PORT, I2C_SCL_PIN)
#define I2C_SDA_HIGH()   GPIO_SetBits(I2C_GPIO_PORT, I2C_SDA_PIN)
#define I2C_SDA_LOW()    GPIO_ResetBits(I2C_GPIO_PORT, I2C_SDA_PIN)
#define I2C_SDA_READ()   GPIO_ReadInputDataBit(I2C_GPIO_PORT, I2C_SDA_PIN)

#define I2C_Delay_us(x)  Delay_us(x)

// ??I2C???
static void SoftI2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // ??JTAG,??PB3/PB4????IO
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);
    
    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
    I2C_Delay_us(5);
}

// ??I2C????
static void SoftI2C_Start(void)
{
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    I2C_Delay_us(5);
    I2C_SDA_LOW();
    I2C_Delay_us(5);
    I2C_SCL_LOW();
}

// ??I2C????
static void SoftI2C_Stop(void)
{
    I2C_SDA_LOW();
    I2C_SCL_HIGH();
    I2C_Delay_us(5);
    I2C_SDA_HIGH();
    I2C_Delay_us(5);
}

// ??I2C??????,???????(0:ACK, 1:NACK)
static uint8_t SoftI2C_SendByte(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
            I2C_SDA_HIGH();
        else
            I2C_SDA_LOW();
        data <<= 1;
        I2C_Delay_us(2);
        I2C_SCL_HIGH();
        I2C_Delay_us(2);
        I2C_SCL_LOW();
    }
    I2C_SDA_HIGH();
    I2C_Delay_us(2);
    I2C_SCL_HIGH();
    I2C_Delay_us(2);
    uint8_t ack = I2C_SDA_READ();
    I2C_SCL_LOW();
    return ack;
}

// ??I2C??????,?????/???
static uint8_t SoftI2C_RecvByte(uint8_t ack)
{
    uint8_t i, data = 0;
    I2C_SDA_HIGH();
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        I2C_SCL_HIGH();
        I2C_Delay_us(2);
        if (I2C_SDA_READ())
            data |= 0x01;
        I2C_SCL_LOW();
        I2C_Delay_us(2);
    }
    if (ack)
        I2C_SDA_HIGH();
    else
        I2C_SDA_LOW();
    I2C_Delay_us(2);
    I2C_SCL_HIGH();
    I2C_Delay_us(2);
    I2C_SCL_LOW();
    I2C_SDA_HIGH();
    return data;
}

// ??I2C????
static void SoftI2C_WaitIdle(void)
{
    I2C_Delay_us(10);
}

// I2C???
void MPU6050_I2C_Init(void)
{
    SoftI2C_Init();
}

// ????:addr??????,data???????
void MPU6050_WriteReg(uint8_t regAddr, uint8_t data)
{
    SoftI2C_WaitIdle();
    SoftI2C_Start();
    // ?????? + ??(??????8????)
    if (SoftI2C_SendByte(MPU6050_ADDR) != 0)  // ?????
    {
        SoftI2C_Stop();
        return;
    }
    if (SoftI2C_SendByte(regAddr) != 0)
    {
        SoftI2C_Stop();
        return;
    }
    SoftI2C_SendByte(data);
    SoftI2C_Stop();
}

// ????:addr??????,??????
uint8_t MPU6050_ReadReg(uint8_t regAddr)
{
    uint8_t data = 0;
    SoftI2C_WaitIdle();
    
    // ???:?????? + ?????
    SoftI2C_Start();
    if (SoftI2C_SendByte(MPU6050_ADDR) != 0)  // ?????
    {
        SoftI2C_Stop();
        return 0;
    }
    if (SoftI2C_SendByte(regAddr) != 0)
    {
        SoftI2C_Stop();
        return 0;
    }
    
    // ??????
    SoftI2C_Start();
    
    // ?????? + ??(??? = ??? | 1)
    if (SoftI2C_SendByte(MPU6050_ADDR | 1) != 0)
    {
        SoftI2C_Stop();
        return 0;
    }
    
    // ????(?????)
    data = SoftI2C_RecvByte(1);  // 1?????
    
    SoftI2C_Stop();
    return data;
}

// MPU6050???
void MPU6050_Init(void)
{
    MPU6050_I2C_Init();
    Delay_ms(100);  // ??MPU6050??
    
    // ??MPU6050,??????
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
    Delay_ms(10);
    
    // ????????,??? = 1kHz/(1+SMPLRT_DIV)
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);
    
    // ??DLPF(???????),????
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);  // 5Hz??
    
    // ????????±500°/s
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x08);  // 500dps
    
    // ?????????±4g
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x08);  // 4g
    
    // ?????
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
}

// ????????
void MPU6050_ReadAccel(MPU6050_AccelData *accel)
{
    uint8_t buf[6];
    buf[0] = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    buf[1] = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    buf[2] = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    buf[3] = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    buf[4] = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    buf[5] = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    
    accel->accelX = (int16_t)((buf[0] << 8) | buf[1]);
    accel->accelY = (int16_t)((buf[2] << 8) | buf[3]);
    accel->accelZ = (int16_t)((buf[4] << 8) | buf[5]);
}

// ???????
void MPU6050_ReadGyro(MPU6050_GyroData *gyro)
{
    uint8_t buf[6];
    buf[0] = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    buf[1] = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    buf[2] = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    buf[3] = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    buf[4] = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    buf[5] = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    
    gyro->gyroX = (int16_t)((buf[0] << 8) | buf[1]);
    gyro->gyroY = (int16_t)((buf[2] << 8) | buf[3]);
    gyro->gyroZ = (int16_t)((buf[4] << 8) | buf[5]);
}

// ??????
void MPU6050_ReadTemp(int16_t *temp, float *tempC)
{
    uint8_t buf[2];
    buf[0] = MPU6050_ReadReg(MPU6050_TEMP_OUT_H);
    buf[1] = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);
    
    *temp = (int16_t)((buf[0] << 8) | buf[1]);
    *tempC = (*temp) / 340.0f + 36.53f;
}

// ?????(??????)
void MPU6050_CalcAngle(MPU6050_Data *data)
{
    float ax = data->accel.accelX;
    float ay = data->accel.accelY;
    float az = data->accel.accelZ;
    
    // ??? pitch = atan2(-ax, sqrt(ay*ay + az*az))
    data->angle.pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / 3.14159265f;
    
    // ??? roll = atan2(ay, az)
    data->angle.roll = atan2f(ay, az) * 180.0f / 3.14159265f;
    
    // ??? yaw ??????????,??0
    data->angle.yaw = 0;
}

// ??????
void MPU6050_ReadAll(MPU6050_Data *data)
{
    MPU6050_ReadAccel(&data->accel);
    MPU6050_ReadGyro(&data->gyro);
    MPU6050_ReadTemp(&data->temp, &data->tempC);
    MPU6050_CalcAngle(data);
}
