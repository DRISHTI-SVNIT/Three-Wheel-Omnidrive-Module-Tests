#include "userLib/common.h"
#include "userLib/init.h"
#include "userLib/pidController.h"
#include "userLib/movingArray.h"

#define KP_motorA 0.0
#define KI_motorA 0.0
#define KD_motorA 0.0
#define KP_motorB 0.0
#define KI_motorB 0.0
#define KD_motorB 0.0

volatile float desiredRPM[2] = {100.0,100.0}, currentRPM[2] = {0.0,0.0};

int main() {
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	initPIDController(A,KP_motorA,KI_motorA,KD_motorA);
	initPIDController(B,KP_motorB,KI_motorB,KD_motorB);
	motorDirInit();
	uart0Init();
	pwmInit();
	qeiInit();
	//setPWM(55,A);
	//setPWM(57,B);
	IntMasterEnable();
	while(1) {
	}
}

void Timer0IntHandler(void) {
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	// Clear the timer interrupt
}

void UARTIntHandler(void) {
}

void QEI0IntHandler(void) {
	QEIIntClear(QEI0_BASE, QEI_INTTIMER);
	float rpm = movingArrayOut(A,calculateRPM(A));
}

void QEI1IntHandler(void) {
	QEIIntClear(QEI1_BASE, QEI_INTTIMER);
	float rpm = movingArrayOut(B,calculateRPM(B));
}
