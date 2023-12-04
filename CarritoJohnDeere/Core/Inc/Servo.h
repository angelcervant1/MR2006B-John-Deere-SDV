/*
 * Servo.h
 *
 *  Created on: Nov 28, 2023
 *      Author: angel
 */

#ifndef SERVO_H
#define SERVO_H

#include "PID.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"

typedef struct{
	uint8_t kMaxRightAngle;
	uint8_t kMaxLeftAngle;
	uint8_t kStraightAngle;
    TIM_HandleTypeDef *timer;
	PID *pid;

}Servo;

extern TIM_HandleTypeDef htim1;

void servoInit(Servo *servo, uint8_t kMaxRightAngle, uint8_t kMaxLeftAngle, uint8_t StraighAngle, TIM_HandleTypeDef *timer);
void moveToDegrees(float currentAngle, float desiredAngle);
void moveToDegreesT(float currentAngle, float desiredAngle, const int tolerance);
void stopServo(void);
float getError();
void moveMaxRight(void);
void moveMaxLeft(void);
void moveStraight(void);


#endif /* INC_SERVO_H_ */
