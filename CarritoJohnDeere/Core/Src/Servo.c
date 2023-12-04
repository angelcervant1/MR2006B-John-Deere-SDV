/*
 * Servo.c
 *
 *  Created on: Nov 28, 2023
 *      Author: angel
 */
#include "Servo.h"
#include "PID.h"


Servo myServo;
float input = 0;
float error = 0;
int servoIzq = 230;
int servoDer = 90;
int servo0 = 155;
float tolerance = 45;

void initServo(Servo *servo, uint8_t kMaxRightAngle, uint8_t kMaxLeftAngle, uint8_t kStraighAngle, TIM_HandleTypeDef *timer){
	servo->kMaxRightAngle = kMaxRightAngle;
	servo->kMaxLeftAngle = kMaxLeftAngle;
	servo->kStraightAngle = kStraighAngle;
	servo->timer = timer;
}

void moveToDegrees(float currentAngle, float setpointAngle) {
    // Calculate the error between the current angle and setpoint angle
    float error = setpointAngle - currentAngle;

    // Limit the error to ensure it stays within a reasonable range
    if (error > tolerance) {
        error = tolerance;
    } else if (error < -tolerance) {
        error = -tolerance;
    }

    // Scale the error to match servo angles
    float scaledError = error * (myServo.kMaxRightAngle - myServo.kStraightAngle) / tolerance;

    // Calculate the new servo position
    float newServoPosition = myServo.kStraightAngle + scaledError;

    // Ensure the new servo position is within the PWM range

    // Set the servo position
    myServo.timer->Instance->CCR1 = (int)newServoPosition;
}

void moveToDegreesT(float currentAngle, float setpointAngle, const int tolerance) {
    // Calculate the error between the current angle and setpoint angle
    float error = setpointAngle - currentAngle;

    // Limit the error to ensure it stays within a reasonable range
    if (error > tolerance) {
        error = tolerance;
    } else if (error < -tolerance) {
        error = -tolerance;
    }

    // Scale the error to match servo angles
    float scaledError = error * (myServo.kMaxRightAngle - myServo.kStraightAngle) / tolerance;

    // Calculate the new servo position
    float newServoPosition = myServo.kStraightAngle + scaledError;

    // Ensure the new servo position is within the PWM range

    // Set the servo position
    myServo.timer->Instance->CCR1 = (int)newServoPosition;
}



float getError(){
	return input;
}



void stopServo(){

	myServo.timer->Instance->CCR1 = 0;

}

void moveMaxRight(){

	myServo.timer->Instance->CCR1 = myServo.kMaxRightAngle;

}

void moveMaxLeft(){

	myServo.timer->Instance->CCR1 = myServo.kMaxLeftAngle;

}

void moveStraight(){

	myServo.timer->Instance->CCR1 = myServo.kStraightAngle;

}
