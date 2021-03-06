/*
 * main.c
 */
#include "userLib/common.h"
#include "userLib/init.h"
#include "userLib/pidController.h"
#include "userLib/movingArray.h"

#define KP 0.02
#define KI 0.0
#define KD 0.0

volatile float desiredRPM = 100.0, currentRPM = 0.0, out = 0.0;

int main(void) {
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	initPIDController(0,KP,KI,KD);
	motorDirInit();
	uart1Init();
	pwmInit();
	qeiInit();
	IntMasterEnable();
	while(1) {
	//	GraphPlot0((int)currentRPM,(int)desiredRPM,0,0);
	}
}

void Timer0IntHandler(void) {
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	// Clear the timer interrupt

}

void UARTIntHandler(void) {

}

void QEIIntHandler(void) {
	QEIIntClear(QEI0_BASE, QEI_INTTIMER);
	currentRPM = movingArrayOut(calculateRPM());
	UART_OutDec(currentRPM,1);
	UARTCharPut(UART1_BASE,',');
	out += PID(0,desiredRPM - currentRPM);
	setPWM(out);
	UART_OutDec(out,1);
	UARTCharPut(UART1_BASE,',');
}
