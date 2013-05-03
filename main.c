#include <avr/io.h>
#include <util/delay.h>

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

#define SPEED 9600
#define F_CPU 1000000


int main (void)
{
	char i = 0;
	char *str = "the quick brown fox jumps over the lazy dog. 1234567890\r\n";

	/* let the preprocessor calculate this */
	serial_init( ( F_CPU / SPEED / 16 ) - 1);

	while (1) {
		serial_write(str[i++]);
		if (str[i] == '\0') {
			i = 0;
			_delay_ms(500);
		}
	}
	return 0;
}

