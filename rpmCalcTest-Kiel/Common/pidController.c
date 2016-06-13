#include "pidController.h"

float errorIntegral[maxPIDControllers];
float prevError[maxPIDControllers];

float Kp[maxPIDControllers] = {0.42,0.42};
float Ki[maxPIDControllers] = {0.115,0.115};
float Kd[maxPIDControllers] = {0.0,0.0};

int antiIntegralWindup[maxPIDControllers];
float maxIntegralError[maxPIDControllers];

void initPIDController(int i, float Kp_, float Ki_, float Kd_) {
	Kp[i] = Kp_;
	Ki[i] = Ki_;
	Kd[i] = Kd_;
	errorIntegral[i] = 0.0;
	prevError[i] = 0.0;
	antiIntegralWindup[i] = 0;
}

void enableAntiIntegralWindup(int i, float maxIntegralError_) {
	maxIntegralError[i] = maxIntegralError_;
	antiIntegralWindup[i] = 1;
}

float PID(int i, float error) {
	float pid = (Kp[i] * error) + (Ki[i] * errorIntegral[i]) + (Kd[i] * (error - prevError[i]));
	errorIntegral[i] += error;
		if(antiIntegralWindup[i]) {
		if(errorIntegral[i] > maxIntegralError[i]) {
			errorIntegral[i] = maxIntegralError[i];
		} else if(errorIntegral[i] < maxIntegralError[i]) {
			errorIntegral[i] = maxIntegralError[i];
		}
	}
	prevError[i] = error;
	return pid;
}
