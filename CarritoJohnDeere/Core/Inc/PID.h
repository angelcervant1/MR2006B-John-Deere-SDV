#ifndef PID_H
#define PID_H

#include "math.h"
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))
#include <stdint.h>
#include "stm32f1xx_hal.h"  // Include the HAL header for your STM32 series

// Structure for PID controller
typedef struct {
	double cv;
	double cv1;
	float error;
	float error1;
	float error2;

	float kP;
	float kI;
	float kD;

	float sample_time;
	float current_millis;
	float previous_millis;
    float pV;
    double pidTicks;
    double dt;
} PID;


// Function prototypes
void initPID(PID *pid, const double kP, const double kI, const double kD, const double sample_time);

double compute(PID *pid, const double setpoint, const double input, const double dt);

void setTunings(PID *pid, const double kP, const double kI, const double kD);

void PIDreset(PID *pid);

#endif // PID_H
