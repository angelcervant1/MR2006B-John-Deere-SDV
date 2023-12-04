#ifndef MOTOR_H
#define MOTOR_H

#include "PID.h"
#include <stdint.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
// Enum for motor state

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
typedef enum {
    FORWARD = 1,
    BACKWARD = 2,
    STOP = 3
} MotorState;

// Structure for motor characteristics
typedef struct {
    double kPulsesPerRevolution;
    double kWheelDiameter;
    float kReductionFactor;
    double MotorKP;
    double MotorKI;
    double MotorKD;
    uint8_t motorId;
    // ... add other characteristics as needed
} MotorDetails;

// Structure for motor
typedef struct {
    MotorState current_state;
    MotorDetails details;
    // Pins
    uint16_t digital_one;
    uint16_t digital_two;
    // Velocity
    uint8_t pwm;
    TIM_HandleTypeDef *timer;
    TIM_HandleTypeDef *encoderChannel;
    uint16_t channel;
    float current_speed;
    float target_speed;
    int encoderPulses;
    int encoderTicks;
    float prevMillis;
	float interval;

    // PID
    PID motorPid;
    // ... add PID parameters and limits
} Motor;

// Function prototypes

void initMotor(Motor *motor, uint8_t ID, TIM_HandleTypeDef *timer, uint32_t channel,
		uint16_t digital_one, uint16_t digital_two, TIM_HandleTypeDef *encoderChannel);

void forward(Motor *motor);
void backward(Motor *motor);
void motorStop(Motor *motor);
void changePwm(Motor *motor, const uint16_t pwm);
void startEncoders(Motor *motor, TIM_HandleTypeDef *timer);
void constantRPM(Motor *motor, const double rpm);
double getTargetSpeed(const Motor *motor);
double getCurrentSpeed(const Motor *motor);
uint8_t getPWM(const Motor *motor);
float PwmToRpm(const double pwm);
float RpmToPwm(const double rpm);
float getRPM(Motor *motor);
int getEncoderPulses(Motor *motor);




#endif // MOTOR_H
