#ifndef F_CPU
	#define F_CPU 8000000UL
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "PS2.h"

void init_PS2() {
// All pins o/p except MISO, Pull up enabled on MISO
	SPI_DDR |= (_BV(MOSI_PIN)) | _BV(SCK_PIN) | _BV(SS_PIN);
	SPI_DDR &= ~(_BV(MISO_PIN));
	SPI_PORT |= _BV(MISO_PIN);
// SPI interrupt disabled, SPI enable, will be enabled after initialization
// LSB first, Master Mode,
// First edge of Clock- Falling
// Sample on Trailing Edge
// Clock Frequency 500kHz for 8MHZ crystal
	SPCR= 0b01111101;

// PS2 initialization
	char  PS2_CONFIGMODE[5]= {0x01, 0x43, 0x00, 0x01, 0x00};
	char  PS2_ANALOGMODE[9]= {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
	char  PS2_SETUPMOTOR[9]= {0x01, 0x4D, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff};
//	char  PS2_RETURNPRES[9]= {0x01, 0x4f, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};
	char  PS2_EXITCONFIG[9]= {0x01, 0x43, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a};

// Enter Config mode
	SPI_PORT &= ~(_BV(SS_PIN));	//ATT held low	
	a=0;
	b=0;
	c=0;
	SPI_send_array(PS2_CONFIGMODE,5);		
	SPI_PORT |= _BV(SS_PIN);	// Driving Attention Low
	_delay_ms(1);

// Enter Analog Mode
	SPI_PORT &= ~(_BV(SS_PIN));	//ATT held low	
	a=0;
	b=0;
	c=0;
	SPI_send_array(PS2_ANALOGMODE,9);		
	SPI_PORT |= _BV(SS_PIN);	// Driving Attention Low
	_delay_ms(1);

// Setup Motor
	SPI_PORT &= ~(_BV(SS_PIN));	//ATT held low	
	a=0;
	b=0;
	c=0;
	SPI_send_array(PS2_SETUPMOTOR,9);		
	SPI_PORT |= _BV(SS_PIN);	// Driving Attention Low
	_delay_ms(1);

// Exit Config
	SPI_PORT &= ~(_BV(SS_PIN));	//ATT held low	
	a=0;
	b=0;
	c=0;
	SPI_send_array(PS2_EXITCONFIG,9);
	SPI_PORT |= _BV(SS_PIN);	// Driving Attention Low
	_delay_ms(1);

// Enter values in PS2_POLLBUTTON
	volatile char ps2_init_array[9]= {0x01, 0x42, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
	for(uint8_t i = 0; i < 10; i++) {
		PS2_POLLBUTTON[i] = ps2_init_array[i];
	}
}

void scan_PS2() {
	SPI_PORT &= ~(_BV(SS_PIN));	//ATT held low	
	a=0;	
    b=0;
	c=0;
	SPI_send_array(PS2_POLLBUTTON,9);
	SPI_PORT |= _BV(SS_PIN);	// Driving Attention Low
	_delay_ms(2);	
}

void PS2_start_vibrate() {
	PS2_POLLBUTTON[4]= 0xff;
	PS2_POLLBUTTON[3]= 0xff;
	SPI_PORT &= ~(_BV(SS_PIN));	//ATT held low	
	a=0;
	b=0;
	c=0;
	SPI_send_array(PS2_POLLBUTTON,9);
	SPI_PORT |= _BV(SS_PIN);	// Driving Attention Low
	_delay_ms(1);	
}

void PS2_stop_vibrate() {
	PS2_POLLBUTTON[3]= 0x00;
	PS2_POLLBUTTON[4]= 0x00;
	SPI_PORT &= ~(_BV(SS_PIN));	//ATT held low	
	a=0;
	b=0;
	c=0;
	SPI_send_array(PS2_POLLBUTTON,9);
	SPI_PORT |= _BV(SS_PIN);	// Driving Attention Low
	_delay_ms(1);	
}

char SPI_send(char send_x) {
	SPDR = send_x;
	while(!(SPSR & _BV(SPIF)));
	_delay_us(30);
	send_x = SPDR;
	return(send_x);
}

void SPI_send_array(char *a, char cmd_length) {
	for(uint8_t i=0; i < cmd_length; i++) {
		data_array[i]= SPI_send(*a);
		a++;
	}
}
