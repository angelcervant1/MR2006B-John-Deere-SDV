/*
 * Motor.c
 *
 *  Created on: Nov 24, 2023
 *      Author: angel
 */

#include "Motor.h"



Motor backLeft;
Motor backRight;

void initMotor(Motor *motor, uint8_t ID, TIM_HandleTypeDef *timer, uint32_t channel,
               uint16_t digital_one, uint16_t digital_two, TIM_HandleTypeDef *encoderChannel) {
    // Initialize the Motor structure
	motor->timer = timer;
	motor->encoderChannel = encoderChannel;
    motor->channel = channel;
    motor->digital_one = digital_one;
    motor->digital_two = digital_two;
    motor->pwm = 0;
    motor->details.motorId = ID;
    motor->details.kPulsesPerRevolution = motor->details.motorId == 1 ? 50 : 50; // PONER AQUI LOS TICKS DEL ENCODER POR VUELTA
    motor->details.kWheelDiameter = 0.090;
    motor->encoderTicks = 0;
    motor->details.kReductionFactor = 34;
    motor->details.MotorKP = 2.55;
    motor->details.MotorKI = 3.5;
    motor->details.MotorKD = 0;
    motor->prevMillis = 0;
    motor->interval = 100;
    initPID(&motor->motorPid, motor->details.MotorKP, motor->details.MotorKI, motor->details.MotorKD, 100);

}

void forward(Motor *motor){
	HAL_GPIO_WritePin(GPIOB, motor->digital_one, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, motor->digital_two, GPIO_PIN_SET);
}

void backward(Motor *motor){
	HAL_GPIO_WritePin(GPIOB, motor->digital_one, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, motor->digital_two, GPIO_PIN_RESET);
}

void motorStop(Motor *motor){
	HAL_GPIO_WritePin(GPIOB, motor->digital_one, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, motor->digital_two, GPIO_PIN_RESET);
}

void changePwm(Motor *motor, const uint16_t pwm){
	motor->pwm = pwm;
	//motor->current_speed = motor->pwm
	switch(motor->channel){
	case TIM_CHANNEL_1:
		motor->timer->Instance->CCR1 = motor->pwm;
		break;
	case TIM_CHANNEL_2:
		motor->timer->Instance->CCR2 = motor->pwm;
		break;
	default:
		break;
	}
}


int getEncoderPulses(Motor *motor){
	return motor->encoderTicks;
}

void constantRPM(Motor *motor, const double rpm){
	//changePwm(motor->, 200);
	//motorPID
	forward(motor);
	float current_speed = getRPM(motor);
	double speed = compute(&motor->motorPid, rpm, current_speed, 0.1);
	speed = RpmToPwm(speed);
	changePwm(motor, speed);
}

float PwmToRpm(const double pwm){
    float rpm = pwm / (255.0 / 350.0);
    return rpm;
}

float RpmToPwm(const double rpm){
	double pwm = rpm * (255.0 / 350.0);
	return pwm;
}

float getRPM(Motor *motor) {
    float currentMillis = HAL_GetTick();
    if ((currentMillis - motor->prevMillis) >= motor->interval) {
        motor->prevMillis = currentMillis;
        motor->current_speed = 10 * motor->encoderTicks * (60 / (motor->details.kReductionFactor * motor->details.kPulsesPerRevolution));
        motor->encoderTicks = 0;
        //backLeft.encoderTicks = 0;
    }

    return motor->current_speed;
}


