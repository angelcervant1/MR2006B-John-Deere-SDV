#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_
#include "main.h"


#define MPU6050_ADDRESS 0xD0

extern I2C_HandleTypeDef hi2c1;

void MPU6050_Init(void);
uint8_t MPU6050_Read_Data(void);
float MPU6050_Get_Ax(void);
float MPU6050_Get_Ay(void);
float MPU6050_Get_Az(void);
float MPU6050_Get_Gx(void);
float MPU6050_Get_Gy(void);
float MPU6050_Get_Gz(void);
float MPU6050_Get_Temperature(void);

#endif
