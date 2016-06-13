#include "tm4c123gh6pm.h"
#include "PLL.h" 
#include "SysTick.h"
#include "UART.h"
#include "init.h"
#include "pidController.h"
#include "common_main.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

void EnableInterrupts(void);
void DisableInterrupts(void);
void WaitForInterrupt(void);

volatile int desiredRPM[2] = {0,0};
float currentMV[2] = {0, 0};
int currentRPM[2] = {0, 0};

int count = 0;
int recordingData = 0;

int *FrontLinearizer_;
int *BackLinearizer_;

unsigned int absolute(int x) {
	return ((x < 0) ? -x : x);
}

void setLinearizers(int *frontLinearizer, int *backLinearizer) {
	FrontLinearizer_ = frontLinearizer;
	BackLinearizer_ = backLinearizer;
}

void setPWM(int pwm, int i) {
	if(i == Back) {
		if(pwm > 0) {
			motorDirectionRegister |= (1 << Back1);
			motorDirectionRegister &=~ (1 << Back2);
			BackPWM = maxPWM - pwm;
		} else {
			motorDirectionRegister |= (1 << Back2);
			motorDirectionRegister &=~ (1 << Back1);
			BackPWM = maxPWM + pwm;
		}
	} else {
		if(pwm > 0) {

			motorDirectionRegister |= (1 << Front1);
			motorDirectionRegister &=~ (1 << Front2);
			FrontPWM = maxPWM - pwm;
		} else {
			motorDirectionRegister |= (1 << Front2);
			motorDirectionRegister &=~ (1 << Front1);
			FrontPWM =  maxPWM + pwm;
		}
	}
}

void setMV(int rpm, int i) {
	if(i == Back) {
		setPWM((*((BackLinearizer_ + (rpm >0)* maxMV) + absolute(rpm))), i);
	} else {
		setPWM((*((FrontLinearizer_ + (rpm >0)* maxMV) + absolute(rpm))), i);
	}
}

void changeDesiredRPM(int rpm, int i) {
			desiredRPM[i] = rpm;
}
void setDesiredRPM() {
	int i;
	for(i = 0; i < 2; i++) {
		float pid = PID(i, desiredRPM[i] - currentRPM[i]);
		currentMV[i] += pid;
		if(currentMV[i] >= maxMV) {
			currentMV[i] = maxMV; 
		} else if(currentMV[i] <= -maxMV){
			currentMV[i] = -maxMV;
		}
		setMV(currentMV[i], i);
	}
}

void calculateRPM() {
	currentRPM[Front] = ((((QEI0_STAT_R) & (0x00000002)) ? 1 : -1) * ((float)(QEI0_SPEED_R * 0.029296875)));
	currentRPM[Back] = ((((QEI1_STAT_R) & (0x00000002)) ? 1 : -1) * ((float)(QEI1_SPEED_R * 0.029296875)));
}


void testMotors(int uart) {
	int i = 0;
	
	motorDirectionRegister |= (1 << Back1) | (1 << Front1) | (1 << Back2) | (1 << Front2);
	FrontPWM = maxPWM - 100;
	BackPWM = maxPWM - 100;
	
	UART_TransmitString("***Checking Direction Pins of Motors and Encoders *** \r\n", uart);
	SysTick_Wait10ms(100);
	
	UART_TransmitString("Clockwise Front... : ", uart);
	motorDirectionRegister |= (1 << Front1);
	motorDirectionRegister &= ~(1 << Front2);
	SysTick_Wait10ms(200);
	calculateRPM();
	UART_OutDec(currentRPM[Front], uart);
	Print_Space(2, uart);
	UART_OutDec(currentRPM[Back], uart);
	New_Line(uart);
	
	UART_TransmitString("Anti Clockwise Front... : ", uart);
	motorDirectionRegister |= (1 << Front2);
	motorDirectionRegister &= ~(1 << Front1);
	SysTick_Wait10ms(200);
	calculateRPM();
	UART_OutDec(currentRPM[Front], uart);
	Print_Space(2, uart);
	UART_OutDec(currentRPM[Back], uart);
	New_Line(uart);
	
	motorDirectionRegister |= (1 << Back1) | (1 << Front1) | (1 << Back2) | (1 << Front2);
	
	UART_TransmitString("Clockwise Back... : ", uart);
	motorDirectionRegister |= (1 << Back1);
	motorDirectionRegister &= ~(1 << Back2);
	SysTick_Wait10ms(200);
	calculateRPM();
	UART_OutDec(currentRPM[Back], uart);
	Print_Space(2, uart);
	UART_OutDec(currentRPM[Front], uart);
	New_Line(uart);
	
	UART_TransmitString("AntiClockwise Back... : ", uart);
	motorDirectionRegister |= (1 << Back2);
	motorDirectionRegister &= ~(1 << Back1);
	SysTick_Wait10ms(200);
	calculateRPM();
	UART_OutDec(currentRPM[Back], uart);
	Print_Space(2, uart);
	UART_OutDec(currentRPM[Front], uart);
	New_Line(uart);
	
	motorDirectionRegister |= (1 << Back1) | (1 << Front1) | (1 << Back2) | (1 << Front2);
	
	UART_TransmitString("***Checking PWM *** \r\n", uart);
	SysTick_Wait10ms(300);
	
	UART_TransmitString("Front... \r\n", uart);
	for (i = minPWM/3; i < maxPWM/3; i += 70) {
		setPWM(i, Front);
		SysTick_Wait10ms(100);
	}
	setPWM(0, Front);
	
	UART_TransmitString("Back... \r\n", uart);
	for (i = minPWM/3; i < maxPWM/3; i += 70) {
		setPWM(i, Back);
		SysTick_Wait10ms(100);
	}
	setPWM(0, Back);
	
	UART_TransmitString("***Done*** \n", uart);
}

void linearize_Front(int uart) {
	int i = 0, prevRPM = 0, count = 0, prevPWM = 0;
	float step = 0.0;
	SysTick_Wait10ms(200);
	UART_TransmitString("const int FrontLinearizer[2][350] = {{\r", uart);
	
	for (i = 0; i >= minPWM; i-=1) {
		setPWM(i, Front);
		SysTick_Wait10ms(100);
		calculateRPM();
		if(prevRPM > currentRPM[Front]) {
			step = ((i - prevPWM) * 1.0)/(currentRPM[Front] - prevRPM);
			count = 0;
			while(prevRPM >= currentRPM[Front]) {
				UART_OutDec((prevPWM-(int)(step*count)), uart);
				count++;
				if(prevRPM == -349) break;
				UART_TransmitString(", ", uart);
				prevRPM--;
			}
			prevPWM = i;
			count = 0;
		}
		if(prevRPM == -349) break;
	}
	UART_TransmitString("} , \r{", uart);
	
	prevRPM = 0, count = 0, prevPWM = 0;
	for (i = 0; i <= maxPWM; i+=1) {
		setPWM(i, Front);
		SysTick_Wait10ms(100);
		calculateRPM();
		if(prevRPM < currentRPM[Front]) {
			step = ((i - prevPWM) * 1.0)/(currentRPM[Front] - prevRPM);
			count = 0;
			while(prevRPM <= currentRPM[Front]) {
				UART_OutDec((prevPWM+(int)(step*count)), uart);
				count++;
				if(prevRPM == 349) break;
				UART_TransmitString(", ", uart);
				prevRPM++;
			}
			prevPWM = i;
			count = 0;
		}
		if(prevRPM == 349) break;
	}
	UART_TransmitString("}};\r", uart);
	setPWM(0, Front);
}

void linearize_Back(int uart) {
	int i = 0, prevRPM = 0, count = 0, prevPWM = 0;
	float step = 0.0;
	SysTick_Wait10ms(200);
	UART_TransmitString("const int BackLinearizer[2][350] = {{\r", uart);
	
	for (i = 0; i >= minPWM; i-=1) {
		setPWM(i, Back);
		SysTick_Wait10ms(100);
		calculateRPM();
		if(prevRPM > currentRPM[Back]) {
			step = ((i - prevPWM) * 1.0)/(currentRPM[Back] - prevRPM);
			count = 0;
			while(prevRPM >= currentRPM[Back]) {
				UART_OutDec((prevPWM-(int)(step*count)), uart);
				count++;
				if(prevRPM == -349) break;
				UART_TransmitString(", ", uart);
				prevRPM--;
			}
			prevPWM = i;
			count = 0;
		}
		if(prevRPM == -349) break;
	}
	UART_TransmitString("} , \r{", uart);
	
	prevRPM = 0, count = 0, prevPWM = 0;
	for (i = 0; i <= maxPWM; i+=1) {
		setPWM(i, Back);
		SysTick_Wait10ms(100);
		calculateRPM();
		if(prevRPM < currentRPM[Back]) {
			step = ((i - prevPWM) * 1.0)/(currentRPM[Back] - prevRPM);
			count = 0;
			while(prevRPM <= currentRPM[Back]) {
				UART_OutDec((prevPWM+(int)(step*count)), uart);
				count++;
				if(prevRPM == 349) break;
				UART_TransmitString(", ", uart);
				prevRPM++;
			}
			prevPWM = i;
			count = 0;
		}
		if(prevRPM == 349) break;
	}
	UART_TransmitString("}};\r", uart);
	setPWM(0, Back);
}

void testLinearizer(int uart) {
	int i = 0;
	for(i = -345; i < 350; i+=10) {
		setMV(i, Back);
		setMV(i, Front);
		SysTick_Wait10ms(100);
		calculateRPM();
		UART_OutDec(i, uart);
		Print_Space(1,uart);
		UART_OutDec(currentRPM[Front], uart);
		Print_Space(1, uart);
		UART_OutDec(currentRPM[Back], uart);
		New_Line(uart);
	}
	setPWM(0, Front);
	setPWM(0, Back);
}

void recordData(int uart) {
	int H[2], deltaPV[2], deltaMV[2] = {50, 50};
	SysTick_Wait10ms(200);
	UART_TransmitString("** Measuring Hysterisis **\r", uart);
	
	setMV(100, Front);
	setMV(100, Back);
	Timer0_Init(80000000*0.25);
	recordingData = 1;
	SysTick_Wait10ms(500);
	calculateRPM();
	H[Front] = currentRPM[Front];
	H[Back] = currentRPM[Back];
	deltaPV[Front] = currentRPM[Front];
	deltaPV[Back] = currentRPM[Back];
	
	count = 0;
	setMV(150, Front);
	setMV(150, Back);
	SysTick_Wait10ms(500);
	calculateRPM();
	deltaPV[Front] = currentRPM[Front] - deltaPV[Front];
	deltaPV[Back] = currentRPM[Back] - deltaPV[Back];
	
	setMV(100, Front);
	setMV(100, Back);
	SysTick_Wait10ms(500);
	calculateRPM();
	H[Front] = currentRPM[Front] - H[Front];
	H[Back] = currentRPM[Back] - H[Back];
	
	UART_TransmitString("Front : ", uart);
	UART_OutDec(H[Front], uart);
	Print_Space(2, uart);
	UART_OutDec(deltaMV[Front], uart);
	Print_Space(2, uart);
	UART_OutDec(deltaPV[Front], uart);
	New_Line(uart);
	UART_TransmitString("Back : ", uart);
	UART_OutDec(H[Back], uart);
	Print_Space(2, uart);
	UART_OutDec(deltaMV[Back], uart);
	Print_Space(2, uart);
	UART_OutDec(deltaPV[Back], uart);
	New_Line(uart);

	setPWM(0, Front);
	setPWM(0, Back);
	recordingData = 0;

}



void setup() {
	int i;
	PLL_Init();
	SysTick_Init();
	PortF_Init();
	UART5_Init();
	PWM_Init();
	setPWM(0,Front);
	setPWM(0,Back);
	//QEI0_Init();
	QEI1_Init();
	EnableInterrupts();
}

void whileLoop() {
}

