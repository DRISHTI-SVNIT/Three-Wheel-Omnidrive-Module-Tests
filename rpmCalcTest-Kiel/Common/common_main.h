#ifndef common_main
#define common_main

/** Motor Pin Definition **/
#define motorDirectionRegister GPIO_PORTF_DATA_R
#define Back1 PF2
#define Back2 PF3
#define Front1 PF0
#define Front2 PF1
#define FrontPWM PWM0_3_CMPA_R //PB5; maxPWM to 0
#define BackPWM PWM0_3_CMPB_R // PB4; maxPWM to 0

/** Variable Limits **/
#define maxPWM 998
#define minPWM -998
#define maxMV 349
#define minMV -349

/** PID Variables **/
#define maxIntegral 10
#define minIntegral -10

#define debug 1
enum {Front, Back};

void setup(void);
void whileLoop(void);

void testMotors(int uartNo);
void linearize_Front(int uartNo);
void linearize_Back(int uartNo);
void testLinearizer(int uartNo);
void recordData(int uartNo);
void setPWM(int pwm,int i);
void setLinearizers(int *frontLinearizer, int *backLinearizer);
void changeDesiredRPM(int rpm, int i);
	
#endif
