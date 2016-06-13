#include "../Common/tm4c123gh6pm.h"
#include "../Common/PLL.h" 
#include "../Common/SysTick.h"
#include "../Common/UART.h"
#include "../Common/init.h"
#include "../Common/pidController.h"
#include "../Common/common_main.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

int uartReceiveCount = 0;

float getRPM(void) {
	return ((((QEI1_STAT_R) & (0x00000002)) ? 1 : -1) * ((float)(QEI1_SPEED_R * 0.06)))*40/4;
}

int main(void) {
	setup();
	UART0_Init();
	//UART1_Init();
	UART_TransmitString("Up And Running!\r", 0);
	setPWM(998,Back);
	Timer0_Init(80000000/40); 
	while(1)	{
		whileLoop();
	}
}

void Timer0A_Handler() {
	TIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER0A timeout
	UART_OutDec(-(int)getRPM(),0);
	UART0_OutChar(',');
}

void UART5_Handler(){                // ISR loop(defined in startup file)
	UART5_ICR_R = UART_ICR_RXIC;        // Acknowldging interrupt: clears the RXRIS bit in the UARTRIS register and		
}