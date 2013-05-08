#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h> 

#define SPEED 9600
#define F_CPU 8000000

// from: http://stackoverflow.com/questions/111928/is-there-a-printf-converter-to-print-in-binary-format
#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"
#define BYTETOBINARY(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0) 

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
	if (status != 0x08)
		serial_write_str("start failed!\n");

	// see fig. 11 in MMA7660 data sheet
	TWIWrite((0x4C << 1) | 0x0);

	// Check value of TWI status register. Mask prescaler bits. 
	// If status different from MT_SLA_ACK go to ERROR
	status = TWIGetStatus();
	if (status != 0x18) {
		serial_write_str("read address failed\n");
	}

	// send register
	TWIWrite(reg);
	status = TWIGetStatus();
	if (status != 0x28) {
		serial_write_str("send reg failed!");
	}

	// restart
	TWIStart();
	status = TWIGetStatus();
	if (status != 0x10) {
		serial_write_str("repeated start failed!");
	}

	// see fig. 14 in MMA7660 data sheet
	TWIWrite((0x4C << 1) | 0x1);
	status = TWIGetStatus();
	if (status != 0x40) {
		serial_write_str("SLA + R failed!");
	}

	// set data
	*data = TWIReadNACK();

	TWIStop();
}

// for sleep
volatile int sleeping = 0;

int main (void)
{
	char msg[128];

	/* let the preprocessor calculate this */
	serial_init( ( F_CPU / SPEED / 16 ) - 1);

	// LED
	// PD4 as output
	DDRD |= (1<<4);
	// initialize 

	// turn off interrupts
	cli();

#if 0
	// ATmega168 interrupt - INT0 is pin 4
	EICRA |= (1 << ISC00);    // set INT0 to trigger on falling edge
	EIMSK |= (1 << INT0);     // Turns on INT0
#endif
	
	// Pin change interrupt enable 0
	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT0);

	// 16 bit timer - every 3 seconds
	TCCR1B |= (1<<CS12) | (1<<CS10);  //Divide by 1024
	OCR1A = 23437;        // Count cycles - 3*8000000/1024 
	TCCR1B |= 1<<WGM12;     //Put Timer/Counter1 in CTC mode
	TIMSK1 |= 1<<OCIE1A;

	sei();                    // turn on interrupts
	
	// i2c
	TWIInit();

	serial_write_str("\n\n***\nstarting i2c...\n");

	// set MODE to stand by
	mma7660_set_data(0x07,0x00);

	// set up SR register
	mma7660_set_data(0x08,0x00);

	// set up interrupt register
	mma7660_set_data(0x06,0b00000100);

	// tap detection reg
	mma7660_set_data(0x09,11);
		
	// tap debounce reg
	mma7660_set_data(0x0a,11);
	
	// set MODE to active
	mma7660_set_data(0x07,0x01);


	while (1) {
		//serial_write_str(str);

		// read a byte
		uint8_t x, y, z;
		mma7660_get_data(0x00, &x);
		mma7660_get_data(0x01, &y);
		mma7660_get_data(0x02, &z);
		//sprintf(msg, "%d %d %d\n", x, y, z);
		//serial_write_str(msg);
		sprintf(msg, "%f, %f, %f\n", gLUT[x], gLUT[y], gLUT[z]);
		//serial_write_str(msg);
		
		// tilt register
		uint8_t tReg;
		mma7660_get_data(0x03, &tReg);
		sprintf(msg, BYTETOBINARYPATTERN"\n", BYTETOBINARY(tReg));
		//sprintf(msg, "%x\n", tReg);
		serial_write_str(msg);

		_delay_ms(100);
#if 0
		serial_write_str("going to sleep...\n");
		
		set_sleep_mode(SLEEP_MODE_PWR_SAVE);
		cli();
		
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
		sei();
#endif

	}

	return 0;
}

// pin change interrupt
ISR (PCINT0_vect)
{
	serial_write_str("pin chnage interrupt!\n");
}

// interrupt handler
ISR (INT0_vect)
{
	serial_write_str("interrupt!\nWaking up...\n");
	sleep_disable();

	// flash LED:

	//Set high
	PORTD |= (1<<4); 
	_delay_ms(50);
	//Set low
	PORTD ^= (1<<4); 
}

// 16-bit timer CTC handler
ISR(TIMER1_COMPA_vect)      //Interrupt Service Routine
{
	serial_write_str("3 seconds up!\n");
}
