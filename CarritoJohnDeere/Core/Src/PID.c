/*
 * PID.c
 *
 *  Created on: Nov 24, 2023
 *      Author: angel
 */
#include "PID.h"

void initPID(PID *pid, const double kP, const double kI, const double kD, const double sample_time){
	pid->cv = 0;
	pid->cv1 = 0;
    pid->error = 0;
    pid->error1 = 0;
    pid->error2 = 0;
    pid->kP = kP;
    pid->kI = kI;
    pid->kD = kD;
    pid->sample_time = sample_time;
    pid->previous_millis = 0;
    pid->current_millis = 0;
    pid->pV=0;
}

double compute(PID *pid, const double setpoint, const double input, const double dt) {
	pid->current_millis = HAL_GetTick();
	pid->dt = dt;
	pid->pV = input;

    pid->error = setpoint - pid->pV;

    pid->cv = pid->cv1 + (pid->kP + pid->kD / pid->dt) * pid->error +
              ((pid->kP) * (-1) + pid->kI * pid->dt - 2 * pid->kD / pid->dt) * pid->error1 +
              (pid->kD / pid->dt) * pid->error2;

    pid->cv1 = pid->cv;
    pid->error2 = pid->error1;
    pid->error1 = pid->error;

    if(pid->cv > 350.0)
    	pid->cv = 350.0;
    if(pid->cv < 30.0)
    	pid->cv = 30.0;

    return pid->cv;
}

void setTunings(PID *pid, const double kP, const double kI, const double kD){
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
}

void PIDreset(PID *pid){
	pid->cv = 0;
	pid->cv1 = 0;
    pid->error = 0;
    pid->error1 = 0;
    pid->error2 = 0;
}

