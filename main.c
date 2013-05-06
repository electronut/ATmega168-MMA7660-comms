#include <avr/io.h>
#include <string.h>
#include <util/delay.h>

#define SPEED 9600
#define F_CPU 8000000

// 6-bit value to g value lookup table
// From APPENDIX C - MMA7660FC ACQUISITION CODE TABLE
float gLUT[] = {
0.000,0.047,0.094,0.141,0.188,0.234,0.281,0.328,0.375,0.422,0.469,0.516,0.563,0.609,0.656,0.703,0.750,0.797,0.844,0.891,0.938,0.984,1.031,1.078,1.125,1.172,1.219,1.266,1.313,1.359,1.406,1.453,-1.500,-1.453,-1.406,-1.359,-1.313,-1.266,-1.219,-1.172,-1.125,-1.078,-1.031,-0.984,-0.938,-0.891,-0.844,-0.797,-0.750,-0.703,-0.656,-0.609,-0.563,-0.516,-0.469,-0.422,-0.375,-0.328,-0.281,-0.234,-0.188,-0.141,-0.094,-0.047};

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

// MMA7660
// set data to given register
void mma7660_set_data(uint8_t reg, uint8_t data)
{
	// generate START 
	TWIStart();

	uint8_t status = TWIGetStatus();
	char msg[128];
	sprintf(msg,"status = %x\n", status);
	serial_write_str(msg);
	if (status != 0x08)
		serial_write_str("start failed!\n");
	else {
		serial_write_str("start succeeded\n");
	}

	// see fig. 11 in MMA7660 data sheet
	TWIWrite((0x4C << 1) | 0x0);

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

	// send register
	TWIWrite(reg);
	status = TWIGetStatus();
	sprintf(msg,"status = %x\n", status);
	serial_write_str(msg);
	// send data
	TWIWrite(data);
	status = TWIGetStatus();
	sprintf(msg,"status = %x\n", status);
	serial_write_str(msg);

	TWIStop();
}

// MMA7660
// set data to given register
void mma7660_get_data(uint8_t reg, uint8_t* data)
{
		// generate START 
	TWIStart();

	uint8_t status = TWIGetStatus();
	char msg[128];
	sprintf(msg,"status = %x\n", status);
	serial_write_str(msg);
	if (status != 0x08)
		serial_write_str("start failed!\n");
	else {
		serial_write_str("start succeeded\n");
	}

	// see fig. 11 in MMA7660 data sheet
	TWIWrite((0x4C << 1) | 0x0);

	// Check value of TWI status register. Mask prescaler bits. 
	// If status different from MT_SLA_ACK go to ERROR
	status = TWIGetStatus();
	sprintf(msg,"status = %x\n", status);
	serial_write_str(msg);
	if (status != 0x18) {
		serial_write_str("read address failed\n");
	}
	else {
		serial_write_str("read address success!\n");
	}

	// send register
	TWIWrite(reg);
	status = TWIGetStatus();
	sprintf(msg,"status = %x\n", status);
	serial_write_str(msg);

	// restart
	TWIStart();
	status = TWIGetStatus();
	sprintf(msg,"status = %x\n", status);
	serial_write_str(msg);

	// see fig. 14 in MMA7660 data sheet
	TWIWrite((0x4C << 1) | 0x1);
	status = TWIGetStatus();
	sprintf(msg,"status = %x\n", status);
	serial_write_str(msg);

	*data = TWIReadNACK();

	TWIStop();
}


int main (void)
{
	char *str = "the quick brown fox jumps over the lazy dog. 1234567890\r\n";
	char msg[128];

	/* let the preprocessor calculate this */
	serial_init( ( F_CPU / SPEED / 16 ) - 1);

	// LED
	// PD4 as output
	DDRD |= (1<<4);
	// initialize 
	
	// i2c
	TWIInit();

	serial_write_str("\n\n***\nstarting i2c...\n");
	uint8_t status;

	// set MODE to active
	mma7660_set_data(0x07,0x01);

	// read a byte
	uint8_t val;
	mma7660_get_data(0x00, &val);
	sprintf(msg, "X: %x\n", val);
	serial_write_str(msg);

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

