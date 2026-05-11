/*
 * mpu6050.c
 *
 *  Created on: Apr 9, 2026
 *      Author: test1
 */


#include "main.h"
#include "mpu6050.h"

void MPU6050_Init()
{
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, 0xD0, 1, 100);
	if(ret == HAL_OK)
	{
		printf("MPU6050 is ready for use. \n");
	}
	else
	{
		printf("MPU6050 is not ready for use. Check cables. \n");
	}


	uint8_t tempData = 0x08;
	ret = HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x1B, 1, &tempData, 1, 100);
	if(ret == HAL_OK)
  {
  	printf("MPU6050 Gyroscope test passed. \n");
  }
  else
  {
  	printf("MPU6050 Gyroscope test failed. \n");
  }

 ret = HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x1C, 1, &tempData, 1, 100);
	if(ret == HAL_OK)
	  {
		printf("MPU6050 Accelerometer test passed. \n");
	  }
	  else
	  {
		printf("MPU6050 Accelerometer test failed. \n");
	  }

	tempData = 0;
	ret = HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x6B, 1, &tempData, 1, 100);
	if(ret == HAL_OK)
	  {
		printf("MPU6050 is not in sleep mode. \n");
	  }
	  else
	  {
		printf("MPU6050 is in sleep mode. \n");
	  }
}

void MPU6050_Read()
{
	uint8_t data[2];
	uint16_t x_acc;
	HAL_I2C_Mem_Read(&hi2c1, 0xD1, 0x3B, 1, data, 2, 100);
	x_acc = ((uint16_t) data[0] << 8) + data[1];
	printf("x axis acceleration: %d \n", x_acc);
}


