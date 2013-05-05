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
	char i = 0;
	char *str = "the quick brown fox jumps over the lazy dog. 1234567890\r\n";

	/* let the preprocessor calculate this */
	serial_init( ( F_CPU / SPEED / 16 ) - 1);

	// LED
	// PD4 as output
	DDRD |= (1<<4);
	// initialize 

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

