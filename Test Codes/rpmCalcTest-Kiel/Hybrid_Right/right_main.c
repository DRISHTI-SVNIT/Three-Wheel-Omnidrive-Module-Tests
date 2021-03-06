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

#define Front_Kp 0.42
#define Front_Ki 0 //0.115
#define Back_Kp 0.42
#define Back_Ki 0 //0.115

int uartReceiveCount = 0;

//Last Updated: 4 feb 2016 (maxon)
const int FrontLinearizer[2][350] = {{
0, -1, -2, -4, -5, -6, -8, -9, -11, -12, -13, -15, -16, -17, -19, -20, -22, -23, -24, -26, -27, -29, -29, -29, -29, -29, -30, -30, -30, -31, -31, -31, -32, -32, -32, -33, -33, -33, -33, -34, -34, -34, -35, -35, -35, -36, -36, -36, -37, -37, -37, -38, -38, -38, -38, -39, -39, -39, -40, -40, -40, -41, -41, -41, -42, -42, -42, -43, -43, -43, -43, -44, -44, -44, -45, -45, -45, -46, -46, -46, -47, -47, -47, -48, -48, -48, -48, -49, -49, -49, -50, -50, -50, -51, -51, -52, -52, -52, -53, -53, -54, -54, -55, -55, -56, -56, -58, -61, -61, -61, -61, -61, -61, -61, -61, -61, -62, -62, -63, -65, -67, -67, -67, -67, -67, -67, -67, -67, -67, -67, -67, -67, -67, -67, -67, -67, -68, -68, -68, -68, -68, -68, -69, -69, -71, -71, -71, -71, -71, -72, -72, -72, -72, -73, -73, -73, -74, -74, -75, -75, -76, -77, -77, -77, -77, -77, -77, -77, -78, -78, -79, -79, -80, -81, -82, -82, -82, -82, -82, -82, -82, -82, -83, -83, -83, -83, -83, -84, -84, -84, -85, -85, -86, -86, -86, -86, -87, -87, -87, -88, -88, -89, -89, -89, -90, -90, -91, -91, -92, -92, -92, -92, -92, -92, -92, -92, -92, -93, -93, -94, -94, -95, -95, -96, -96, -97, -97, -97, -97, -98, -98, -98, -98, -98, -99, -99, -100, -101, -101, -102, -103, -103, -104, -104, -104, -104, -104, -104, -104, -105, -105, -105, -106, -106, -107, -107, -108, -108, -108, -108, -108, -108, -108, -109, -109, -109, -110, -110, -110, -111, -112, -113, -113, -113, -114, -115, -115, -116, -116, -117, -118, -118, -118, -118, -118, -118, -118, -118, -119, -119, -119, -120, -120, -121, -121, -124, -124, -124, -125, -126, -127, -127, -127, -127, -127, -128, -128, -130, -130, -131, -131, -131, -131, -132, -132, -133, -134, -135, -135, -135, -135, -135, -136, -136, -137, -139, -139, -139, -140, -141, -141, -141, -141, -141, -142, -142, -142, -143, -143, -143, -143, -144, -144, -144, -145, -145, -146, -146, -148, -150} , 
{0, 1, 2, 3, 5, 6, 7, 9, 10, 11, 12, 14, 15, 16, 18, 19, 20, 21, 23, 24, 25, 27, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 38, 38, 39, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 41, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 43, 43, 43, 43, 44, 44, 44, 45, 45, 45, 46, 46, 46, 47, 47, 47, 48, 48, 51, 51, 53, 56, 56, 56, 56, 56, 57, 57, 57, 58, 58, 58, 58, 59, 59, 59, 60, 60, 61, 61, 61, 61, 62, 62, 63, 63, 63, 64, 64, 65, 65, 65, 65, 66, 66, 66, 67, 67, 67, 67, 67, 67, 68, 68, 68, 68, 68, 68, 69, 69, 69, 69, 69, 70, 70, 70, 70, 70, 71, 71, 71, 72, 73, 73, 74, 74, 75, 75, 75, 75, 75, 75, 75, 75, 76, 76, 76, 77, 78, 78, 79, 80, 81, 81, 82, 82, 82, 82, 82, 83, 83, 83, 84, 84, 84, 85, 85, 85, 86, 86, 86, 86, 86, 86, 87, 87, 87, 87, 87, 88, 88, 88, 89, 90, 90, 90, 90, 90, 91, 91, 91, 92, 92, 92, 93, 93, 94, 96, 96, 96, 96, 97, 97, 97, 97, 98, 98, 98, 99, 99, 99, 100, 100, 101, 101, 102, 103, 103, 103, 103, 103, 103, 103, 103, 104, 104, 104, 104, 104, 105, 105, 105, 106, 107, 107, 107, 107, 107, 107, 107, 108, 108, 108, 109, 109, 110, 110, 110, 111, 112, 112, 113, 114, 115, 115, 115, 115, 116, 116, 118, 118, 118, 119, 120, 120, 121, 122, 122, 122, 122, 122, 123, 123, 123, 123, 123, 124, 124, 124, 125, 125, 125, 126, 126, 127, 127, 128, 128, 128, 129, 129, 129, 129, 130, 130, 130, 131, 132, 132, 133, 134, 134, 134, 135, 135, 139, 139, 139, 139, 139, 139, 140, 140, 140, 141, 141, 142, 142, 143, 144, 144, 144, 145, 145, 146, 146}};

//Last Updated: 4 feb 2016 (maxon)
const int BackLinearizer[2][350] = {{
0, -1, -3, -5, -6, -8, -10, -11, -13, -15, -16, -18, -20, -22, -23, -25, -27, -28, -30, -32, -33, -35, -37, -38, -40, -42, -44, -45, -47, -49, -50, -52, -54, -55, -57, -59, -60, -62, -64, -66, -66, -66, -66, -66, -66, -67, -67, -67, -67, -67, -67, -67, -68, -68, -68, -68, -68, -69, -69, -69, -69, -70, -70, -71, -71, -71, -72, -73, -73, -73, -73, -74, -74, -75, -75, -75, -76, -76, -77, -78, -78, -78, -79, -79, -80, -80, -80, -81, -81, -82, -82, -82, -82, -83, -83, -83, -84, -84, -85, -86, -86, -86, -87, -87, -88, -88, -88, -89, -89, -91, -91, -91, -91, -92, -92, -93, -94, -94, -94, -94, -94, -94, -94, -94, -95, -95, -96, -98, -98, -98, -98, -99, -99, -99, -100, -100, -101, -103, -103, -103, -103, -103, -103, -103, -104, -104, -104, -105, -106, -106, -106, -107, -108, -109, -109, -109, -109, -110, -110, -110, -111, -112, -112, -112, -113, -114, -114, -114, -115, -116, -116, -116, -117, -118, -119, -119, -119, -120, -121, -121, -121, -122, -122, -123, -124, -124, -126, -126, -126, -126, -126, -126, -126, -127, -127, -127, -128, -128, -129, -129, -129, -130, -131, -131, -131, -131, -131, -132, -132, -132, -132, -133, -133, -133, -134, -134, -135, -135, -135, -136, -136, -137, -138, -138, -142, -142, -142, -143, -143, -143, -144, -144, -145, -145, -145, -146, -147, -148, -148, -148, -149, -149, -150, -150, -150, -150, -150, -150, -151, -151, -151, -152, -153, -153, -155, -155, -155, -155, -156, -156, -157, -158, -159, -159, -159, -160, -160, -161, -161, -161, -162, -163, -163, -163, -164, -165, -166, -167, -167, -169, -169, -169, -170, -170, -170, -171, -172, -172, -172, -173, -173, -173, -174, -175, -176, -176, -182, -182, -182, -182, -183, -183, -183, -184, -185, -185, -185, -186, -187, -188, -188, -189, -190, -192, -192, -194, -196, -196, -196, -196, -196, -197, -197, -197, -198, -198, -199, -201, -203, -203, -205, -205, -206, -206, -207, -207, -207, -208, -208, -209, -209, -209, -209, -209, -209, -210, -210, -215, -215, -216} , 
{0, 3, 6, 10, 13, 16, 20, 23, 26, 30, 33, 36, 40, 40, 40, 40, 41, 41, 41, 42, 42, 42, 43, 43, 44, 44, 44, 45, 45, 45, 46, 46, 47, 47, 47, 48, 48, 48, 49, 49, 50, 50, 50, 50, 50, 50, 50, 51, 51, 51, 51, 51, 51, 52, 52, 52, 53, 53, 56, 60, 64, 64, 64, 64, 65, 65, 66, 66, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 69, 69, 70, 71, 71, 71, 71, 72, 72, 72, 73, 73, 73, 74, 74, 75, 75, 75, 76, 76, 77, 78, 78, 81, 81, 81, 81, 81, 81, 81, 81, 82, 82, 82, 82, 83, 83, 83, 84, 85, 85, 86, 86, 86, 87, 88, 88, 89, 91, 91, 91, 91, 91, 91, 91, 92, 92, 92, 93, 93, 94, 94, 95, 95, 96, 97, 97, 97, 98, 98, 99, 99, 99, 100, 101, 101, 101, 102, 102, 103, 103, 104, 104, 107, 107, 107, 108, 108, 108, 108, 108, 109, 109, 110, 110, 110, 110, 111, 111, 111, 112, 112, 112, 113, 114, 115, 115, 115, 116, 116, 116, 117, 117, 117, 118, 118, 119, 120, 120, 121, 123, 123, 123, 123, 123, 124, 124, 124, 124, 124, 125, 125, 127, 127, 127, 127, 128, 128, 129, 129, 130, 130, 132, 132, 133, 133, 134, 134, 135, 135, 135, 136, 136, 137, 138, 138, 139, 139, 139, 140, 140, 140, 141, 142, 142, 142, 143, 143, 144, 144, 145, 147, 147, 148, 149, 149, 149, 149, 150, 150, 150, 151, 151, 152, 152, 155, 155, 155, 156, 156, 157, 157, 157, 157, 158, 158, 158, 159, 159, 160, 161, 161, 162, 162, 162, 163, 163, 163, 164, 164, 167, 167, 168, 168, 169, 170, 171, 171, 171, 171, 171, 172, 172, 173, 175, 175, 175, 175, 176, 176, 177, 177, 178, 179, 180, 180, 180, 181, 182, 182, 182, 182, 182, 183, 183, 184, 185, 185, 185, 186, 187, 187, 189, 189, 189, 190, 190, 191, 192, 193, 193, 193, 194, 194, 195}};

int main(void) {
	setLinearizers((int*)FrontLinearizer, (int*)BackLinearizer);
	initPIDController(Front, Front_Kp, Front_Ki, 0.0);
	enableAntiIntegralWindup(Front, maxIntegral);
	initPIDController(Back, Back_Kp, Back_Ki, 0.0);
	enableAntiIntegralWindup(Back, maxIntegral);
	setup();
	//Timer0_Init(80000000*0.25); 
	UART0_Init();
	testMotors(0);
	//UART1_Init();
	UART_TransmitString("Up And Running!\r", 0);
	while(1)	{
		whileLoop();
	}
}


void UART5_Handler(){                // ISR loop(defined in startup file)
	char data = UART5_DR_R;
	UART5_ICR_R = UART_ICR_RXIC;        // Acknowldging interrupt: clears the RXRIS bit in the UARTRIS register and
	//UART5_TransmitString("In the Handler\r", 5);
	if(data == 0x0A) {
		uartReceiveCount = 0;
	} else {
		switch(uartReceiveCount) {
			case 0: // Ignore FrontLeft
				break;
			case 1: changeDesiredRPM((data * ((data & 0x01) ? -1 : 1)), Front);
				break;
			case 2: changeDesiredRPM((data * ((data & 0x01) ? -1 : 1)), Back);
				break;
			case 3: // Ignore BackLeft
			  break;
		}
		uartReceiveCount++;
	}		
}	

		

