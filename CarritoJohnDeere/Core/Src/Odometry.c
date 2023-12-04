/*
 * Odometry.c
 *
 *  Created on: Nov 24, 2023
 *      Author: angel
 */
#include "Odometry.h"

long errorA;
long errorB;
long inputA = 1;
long inputB = 0;
long VueltasA = 0;
long DespA = 0;
uint16_t pulsosEncoderAnterior;

void initMovement(){
	initMotor(&backRight, 2, &htim2, TIM_CHANNEL_1, GPIO_PIN_12, GPIO_PIN_13, &htim3);
	initMotor(&backLeft, 1, &htim2, TIM_CHANNEL_2, GPIO_PIN_14, GPIO_PIN_15, &htim4);
	initServo(&myServo, 240, 95, 155, &htim1);

}

void goStraight(){
	changePwm(&backLeft, 50);
	changePwm(&backRight, 50);
	forward(&backLeft);
	forward(&backRight);
}

void stop(){
	changePwm(&backLeft, 0);
	changePwm(&backRight, 0);
	motorStop(&backLeft);
	motorStop(&backRight);
}

void setRobotAngle(const uint8_t angle){
	return;
}

void checkPID(const double setpoint){
	constantRPM(&backRight, setpoint);
	constantRPM(&backLeft, setpoint);
}

void MoverServo(float degree, float setpoint){
	moveToDegrees(degree, setpoint);
}

float moveDistance(Motor *motor, const long SetpointD){

	  errorA = SetpointD-(int)(DespA*0.0733)- motor->encoderPulses*100;

	  inputA = errorA;

	  if (inputA>100){
		  inputA = 100;
	  }

	  inputA = inputA/50*60;

	  if (errorA<15){
		  inputA = 0;
	  }
	//155 0 grados normal
	//230 60 grados giro derecha
	//90 -60 grados giro izquierda

	  DespA += motor->encoderPulses = pulsosEncoderAnterior;

	  if (motor->encoderPulses>=1364){
		  VueltasA++;
		  DespA -= 1364;

	  }

	  pulsosEncoderAnterior = motor->encoderPulses;
	  return inputA;

}




