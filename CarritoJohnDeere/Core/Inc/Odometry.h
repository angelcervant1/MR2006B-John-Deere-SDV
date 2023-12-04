/*
 * Odometry.h
 *
 *  Created on: Nov 24, 2023
 *      Author: angel
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Motor.h"
#include "mpu6050.h"
#include "Servo.h"
#include "stm32f1xx_hal.h"

//#include "mpu6050.h"
extern Servo myServo;
extern Motor backLeft;
extern Motor backRight;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;




void initMovement();
void setRobotAngle(const uint8_t angle);
void stop();
void goStraight();
void checkPID(const double setpoint);
void MoverServo(float degree, float setpoint);
float moveDistance(Motor *motor, const long setpoint);
#endif /* INC_ODOMETRY_H_ */
