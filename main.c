#include <avr/io.h>
#include <string.h>
#include <util/delay.h>

#define SPEED 4800
#define F_CPU 1000000

// from https://www.mainframe.cx/~ckuethe/avr-c-tutorial/
// Copyright (c) 2008 Chris Kuethe <chris.kuethe@gmail.com>
void serial_init(unsigned int bittimer)
{
	/* Set the baud rate */
	UBRR0H = (unsigned char) (bittimer >> 8);
	UBRR0L = (unsigned char) bittimer;
	/* set the framing to 8N1 */
	UCSR0C = (3 << UCSZ00);
	/* Engage! */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	return;
}

void serial_write(unsigned char c)
{
	while ( !(UCSR0A & (1 << UDRE0)) )
		;
	UDR0 = c;
}

// write null terminated string
void serial_write_str(const char* str)
{
	int len = strlen(str);
	int i;
	for (i = 0; i < len; i++) {
		serial_write(str[i]);
	}		
}

int main (void)
{
	char *str = "the quick brown fox jumps over the lazy dog. 1234567890\r\n";

	/* let the preprocessor calculate this */
	serial_init( ( F_CPU / SPEED / 16 ) - 1);

	// LED
	// PD4 as output
	DDRD |= (1<<4);
	// initialize 
	
	// i2c
	serial_write_str("\n\n***\nstarting i2c...\n");
	uint8_t status;

	TWIStart();
	status = TWIGetStatus();
	char msg[128];
	sprintf(msg,"status = %x\n", status);
	serial_write_str(msg);
	if (status != 0x08)
		serial_write_str("start failed!\n");
	else {
		serial_write_str("start succeeded\n");
	}
	///	TWIWrite((()|(u8paddr>>3))&(~1));
	//	if (TWIGetStatus() != 0x18)
	//return ERROR;

	// Load SLA_W into TWDR Register. 
	// Clear TWINT bit in TWCR to start transmission of address
	
	// see fig. 11 in MMA7660 data sheet
	TWDR = (0x4C << 1) | 0x0;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// Wait for TWINT Flag set. This indicates that the SLA+W 
	// has been transmitted, and ACK/NACK has been received.
	while (!(TWCR & (1<<TWINT))) ;
	// Check value of TWI status register. Mask prescaler bits. 
	// If status different from MT_SLA_ACK go to ERROR
	status = TWIGetStatus();
	sprintf(msg,"status = %x\n", status);
	serial_write_str(msg);
	if (status != 0x18) {
		serial_write_str("address failed\n");
	}
	else {
		serial_write_str("address success!\n");
	}

	TWIStop();
	 

	while (1) {
		serial_write_str(str);
		
		//Set high
		PORTD |= (1<<4); 
		_delay_ms(200);
		//Set low
		PORTD ^= (1<<4); 
		_delay_ms(200);
	}
	return 0;
}

