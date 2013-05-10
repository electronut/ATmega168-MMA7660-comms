#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h> 

#define SPEED 9600
#define F_CPU 8000000

// uncomment to enable ATmega168 sleep mode
// #define ENABLE_SLEEP_MODE

// from: http://stackoverflow.com/questions/111928/is-there-a-printf-converter-to-print-in-binary-format
#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"
#define BYTETOBINARY(byte)                      \
  (byte & 0x80 ? 1 : 0),                        \
    (byte & 0x40 ? 1 : 0),                      \
    (byte & 0x20 ? 1 : 0),                      \
    (byte & 0x10 ? 1 : 0),                      \
    (byte & 0x08 ? 1 : 0),                      \
    (byte & 0x04 ? 1 : 0),                      \
    (byte & 0x02 ? 1 : 0),                      \
    (byte & 0x01 ? 1 : 0) 

// 6-bit value to g value lookup table
// From APPENDIX C - MMA7660FC ACQUISITION CODE TABLE
float gLUT[] = {
  0.000,0.047,0.094,0.141,0.188,0.234,0.281,0.328,0.375,0.422,0.469,0.516,0.563,0.609,0.656,0.703,0.750,0.797,0.844,0.891,0.938,0.984,1.031,1.078,1.125,1.172,1.219,1.266,1.313,1.359,1.406,1.453,-1.500,-1.453,-1.406,-1.359,-1.313,-1.266,-1.219,-1.172,-1.125,-1.078,-1.031,-0.984,-0.938,-0.891,-0.844,-0.797,-0.750,-0.703,-0.656,-0.609,-0.563,-0.516,-0.469,-0.422,-0.375,-0.328,-0.281,-0.234,-0.188,-0.141,-0.094,-0.047};


//
// BEGIN: serial comms
//
// from data sheet
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

void USART_Init(unsigned int ubrr) 
{
  /*Set baud rate */
  UBRR0H = (unsigned char)(ubrr>>8); 
  UBRR0L = (unsigned char)ubrr;
  /*Enable receiver and transmitter */ 
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  /* Set frame format: 8data, 1 stop bit */ 
  UCSR0C = (1<<UCSZ00) | (1 << UCSZ01);
}

void USART_Transmit(unsigned char data ) 
{
  /* Wait for empty transmit buffer */ 
  while ( !( UCSR0A & (1<<UDRE0)) )
    ;
  /* Put data into buffer, sends the data */ 
  UDR0 = data;
}

// write null terminated string
void serial_write_str(const char* str)
{
  int len = strlen(str);
  int i;
  for (i = 0; i < len; i++) {
    USART_Transmit(str[i]);
  }   
}

//
// END: serial comms
//

//
// BEGIN: I2C comms
//

void TWI_init()
{
  // status register
  TWSR = 0x00;

  // set TWI bit rate register:
  // SCL_freq = CPU_freq/(16 + 2*(TWBR)*(PrescalarValue)
  // In this case = 8000000/(16+2*12*1) = 200 kHz
  TWBR = 0x0C;

  //enable TWI
  TWCR = (1<<TWEN);
}

void TWI_start()
{
  // Send START condition
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  // Wait for TWINT Flag set. 
  // This indicates that the START condition has been transmitted
  while (!(TWCR & (1<<TWINT))) ;
}

void TWI_stop()
{
  // Transmit STOP condition
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void TWI_write(uint8_t data)
{
  // Load DATA into TWDR register. 
  // Clear TWINT bit in TWCR to start transmission of data
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  // Wait for TWINT flag set. 
  // This indicates that the DATA has been transmitted, 
  // and ACK/NACK has been received.
  while (!(TWCR & (1<<TWINT))) ;
}

// read byte with ACK
uint8_t TWI_readACK(void)
{
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
  while (!(TWCR & (1<<TWINT))) ;
  return TWDR;
}

// read byte with NACK
uint8_t TWI_readNACK(void)
{
  TWCR = (1<<TWINT)|(1<<TWEN);
  while (!(TWCR & (1<<TWINT))) ;
  return TWDR;
}

uint8_t TWI_status(void)
{
  return (TWSR & 0xF8);
}

//
// END: I2C comms
//

// MMA7660
// set data to given register
void mma7660_set_data(uint8_t reg, uint8_t data)
{
  // generate START 
  TWI_start();

  uint8_t status = TWI_status();
  char msg[128];
  sprintf(msg,"status = %x\n", status);
  serial_write_str(msg);
  if (status != 0x08)
    serial_write_str("start failed!\n");
  else {
    serial_write_str("start succeeded\n");
  }

  // see fig. 11 in MMA7660 data sheet
  TWI_write((0x4C << 1) | 0x0);

  // Check value of TWI status register. Mask prescaler bits. 
  // If status different from MT_SLA_ACK go to ERROR
  status = TWI_status();
  sprintf(msg,"status = %x\n", status);
  serial_write_str(msg);
  if (status != 0x18) {
    serial_write_str("address failed\n");
  }
  else {
    serial_write_str("address success!\n");
  }

  // send register
  TWI_write(reg);
  status = TWI_status();
  sprintf(msg,"status = %x\n", status);
  serial_write_str(msg);
  // send data
  TWI_write(data);
  status = TWI_status();
  sprintf(msg,"status = %x\n", status);
  serial_write_str(msg);

  TWI_stop();
}

// MMA7660
// set data to given register
void mma7660_get_data(uint8_t reg, uint8_t* data)
{
  // generate START 
  TWI_start();

  uint8_t status = TWI_status();
  if (status != 0x08)
    serial_write_str("start failed!\n");

  // see fig. 11 in MMA7660 data sheet
  TWI_write((0x4C << 1) | 0x0);

  // Check value of TWI status register. Mask prescaler bits. 
  // If status different from MT_SLA_ACK go to ERROR
  status = TWI_status();
  if (status != 0x18) {
    serial_write_str("read address failed\n");
  }

  // send register
  TWI_write(reg);
  status = TWI_status();
  if (status != 0x28) {
    serial_write_str("send reg failed!");
  }

  // restart
  TWI_start();
  status = TWI_status();
  if (status != 0x10) {
    serial_write_str("repeated start failed!");
  }

  // see fig. 14 in MMA7660 data sheet
  TWI_write((0x4C << 1) | 0x1);
	status = TWI_status();
	if (status != 0x40) {
		serial_write_str("SLA + R failed!");
	}

	// set data
	*data = TWI_readNACK();

	TWI_stop();
}

// for sleep
volatile int goToSleep = 0;

int main (void)
{
  USART_Init(MYUBRR);

	// LED for testing
	// PD4 as output
	DDRD |= (1<<4);
	// initialize 

	// turn off interrupts
	cli();

	// 16 bit timer - every 3 seconds
	TCCR1B |= (1<<CS12) | (1<<CS10);  //Divide by 1024
	OCR1A = 23437;        // Count cycles - 3*8000000/1024 
	TCCR1B |= 1<<WGM12;     //Put Timer/Counter1 in CTC mode

	sei();                    // turn on interrupts
	
	// i2c
	TWI_init();

	// set MODE to stand by
	mma7660_set_data(0x07,0x00);

	// set up SR register
	mma7660_set_data(0x08,0x00);

	// set up interrupt register
	mma7660_set_data(0x06,0b11100100);

	// tap detection reg
	mma7660_set_data(0x09,11);
		
	// tap debounce reg
	mma7660_set_data(0x0a,11);

	// count
	mma7660_set_data(0x05, 0xff);
 
	// set MODE to active
	mma7660_set_data(0x07,0b00011001);


	while (1) {

		// read ax/ay/az
		uint8_t x, y, z;
    char strA[128];
		mma7660_get_data(0x00, &x);
		mma7660_get_data(0x01, &y);
		mma7660_get_data(0x02, &z);
    sprintf(strA, "%f %f %f", gLUT[x], gLUT[y], gLUT[z]);
		
		// tilt register
		uint8_t tReg;
    char strT[16];
		mma7660_get_data(0x03, &tReg);
		sprintf(strT, BYTETOBINARYPATTERN, BYTETOBINARY(tReg));

		// SRST
		uint8_t srstReg;
    char strS[16];
		mma7660_get_data(0x04, &srstReg);
		sprintf(strS, BYTETOBINARYPATTERN, BYTETOBINARY(srstReg));

    // output 
    char msg[256];
    sprintf(msg, "DATA %s %s %s\n", strA, strT, strS);
		serial_write_str(msg);

    // delay
		_delay_ms(100);

    // sleep mode code
#ifdef ENABLE_SLEEP_MODE

    // is MMA7660 asleep?
    if(srstReg & 0b00000010) {

      // start ATmega168 sleep timer if not already started
      TIMSK1 |= 1<<OCIE1A;
    }
    else {

      // cancel ATmega168 sleep timer if already started
      TIMSK1 &= ~(1<<OCIE1A);
    }

		if(goToSleep) {
			serial_write_str("going to sleep...\n");
		
			set_sleep_mode(SLEEP_MODE_PWR_SAVE);
			cli();

      // enable Pin change interrupt enable 0
      PCICR |= (1 << PCIE0);
      PCMSK0 |= (1 << PCINT0);

      // cancel ATmega168 sleep timer if already started
      TIMSK1 &= ~(1<<OCIE1A);

      // enable sleep flag
			sleep_enable();
			sei();
			// sleep
			sleep_cpu();
      // ...
			// just awake
      // ...
      // unset sleep flag
			goToSleep = 0;
      // disable sleep flag
			sleep_disable();
      
      // disable Pin change interrupt enable 0
      PCICR &= ~(1 << PCIE0);

			sei();
		}
#endif // ENABLE_SLEEP_MODE

	}

	return 0;
}

// pin change interrupt
ISR (PCINT0_vect)
{
	sleep_disable();
	serial_write_str("\npin change interrupt!\n");

#if 0
	// flash LED:

	//Set high
	PORTD |= (1<<4); 
	_delay_ms(50);
	//Set low
	PORTD ^= (1<<4); 
#endif

}

// 16-bit timer CTC handler
ISR(TIMER1_COMPA_vect)      //Interrupt Service Routine
{
	serial_write_str("3 seconds up!\n");
	goToSleep = 1;
}
