/*
 * mpu6050.h
 *
 *  Created on: Apr 9, 2026
 *      Author: test1
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>

void MPU6050_Init();
void MPU6050_Read();

extern I2C_HandleTypeDef hi2c1;


#endif /* INC_MPU6050_H_ */
