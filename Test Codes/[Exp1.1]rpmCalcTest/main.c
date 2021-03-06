/*
 * main.c
 */
#include "userLib/common.h"
#include "userLib/init.h"
#include "userLib/pidController.h"

int main(void) {
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	motorDirInit();
	uart0Init();
	qeiInit();
	pwmInit();
	UART_TransmitString("Up and running",0);
	setPWM(100);
//	timerInit();
	while(1) {
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
	int rpm = calculateRPM();
	UART_OutDec(rpm,0);
	UARTCharPut(UART0_BASE,'+');
}
