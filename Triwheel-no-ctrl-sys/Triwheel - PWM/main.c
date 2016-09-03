/*
 * Triwheel - PWM.c
 *
 * Created: 6/9/2016 11:45:26 AM
 * Author : Aniket
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <stdbool.h>
#include "PS2.h"
#include "USART_128.h"
#include "macros.h"

enum {select, leftStick, rightStick, start, up, right, down, left}; //3rd byte
enum {leftFront2, rightFront2, leftFront1, rightFront1, triangle_up, circle_right, cross_down, square_left}; // 4th byte
volatile bool UP,DOWN,LEFT,RIGHT;

struct omniDriveState {
	float rpmA;
	float rpmB;
	float rpmC;
	} wheel;

struct uniCycleState {
	float vx;
	float vy;
	float w;
	} kinematic;
	
struct omniDriveState triWheelEquation(struct uniCycleState botKinematic) {
	struct omniDriveState wheel_;
	wheel_.rpmA = -(botKinematic.vx/2) + (botKinematic.vy*0.8660/*sqrt(3)/2*/) + (wheelRadius*botKinematic.w);
	wheel_.rpmB = -(botKinematic.vx/2) - (botKinematic.vy*0.8660/*sqrt(3)/2*/) + (wheelRadius*botKinematic.w);
	wheel_.rpmC = botKinematic.vx + (wheelRadius*botKinematic.w);
	return (wheel_);
}

void PWMupdate(void) {
	kinematic.vx = 0;
	kinematic.vy = 0;
	if(UP) {
		kinematic.vy = 200;
	}
	if(DOWN) {
		kinematic.vy = -200;
	}
	if(RIGHT) {
		kinematic.vx = 200;
	}
	if(LEFT) {
		kinematic.vx = -200;
	}
	kinematic.w = 0;
	wheel  = triWheelEquation(kinematic);
	if(wheel.rpmA >= 0) {
		motorOutPort |= 1<<motorA1;
		motorOutPort &= ~(1<<motorA2);
	} else {
		motorOutPort |= 1<<motorA2;
		motorOutPort &= ~(1<<motorA1);		
		wheel.rpmA = -wheel.rpmA;
	}
	PWMA = wheel.rpmA * 29.2;
	if(wheel.rpmB >= 0) {
		motorOutPort |= 1<<motorB1;
		motorOutPort &= ~(1<<motorB2);
	} else {
		motorOutPort |= 1<<motorB2;
		motorOutPort &= ~(1<<motorB1);
		wheel.rpmB = -wheel.rpmB;
	}
	PWMB = wheel.rpmB * 30;
	if(wheel.rpmC >= 0) {
		motorOutPort |= 1<<motorC1;
		motorOutPort &= ~(1<<motorC2);
	} else {
		motorOutPort |= 1<<motorC2;
		motorOutPort &= ~(1<<motorC1);
		wheel.rpmC = -wheel.rpmC;
	}
	PWMC = wheel.rpmC * 33.33;
}

int isPressed(uint8_t dataByte, uint8_t dataBit) {
	return ((dataByte & (1 << dataBit)) ? 1 : 0);
}

void motorDirInit(void) {
	motorDirPort = 0xFF;
	motorOutPort = 0x00;
}

void PWMinit(void) {
	PWMPort |= 1<<PWMApin | 1<<PWMBpin | 1<<PWMCpin;
	TCCR1A |= 1<<COM1A1 | 1<< COM1B1 | 1<<COM1C1 | 1<<WGM11;
	TCCR1B |= 1<<WGM12 | 1<<WGM13 | 1<<CS10;
	ICR1 = 20000;
}

int main(void) {
	USART_Init(12,0);
	motorDirInit();
	PWMinit();
	USART_TransmitString("Pin Config done.\n",0);
	init_PS2();
	USART_TransmitString("PS2 Config done.\n",0);
	uint8_t x;
    while (1) {
		scan_PS2();
		x = ~(data_array[3]);
		if(isPressed(x, up)) {
			USART_TransmitString("Up ",0);
			UP = true;
		} else {
			UP = false;
		}
		if(isPressed(x, down)) {
			USART_TransmitString("Down ",0);
			DOWN = true;
		} else {
			DOWN = false;
		}
		if(isPressed(x, right)) {
			USART_TransmitString("Right ",0);
			RIGHT = true;
		} else {
			RIGHT = false;
		}
		if(isPressed(x, left)) {
			USART_TransmitString("Left ",0);
			LEFT = true;
		} else {
			LEFT = false;
		}
		USART_Transmitchar(0x0D,0);		
		PWMupdate();
	}
}