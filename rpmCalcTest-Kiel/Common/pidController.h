#ifndef pidController
#define pidController

#define maxPIDControllers 2

void initPIDController(int i, float Kp_, float Ki_, float Kd_);
void enableAntiIntegralWindup(int i, float maxIntegralError_);
float PID(int i, float error);

#endif
