/*
 * VanStalk.c
 *
 * Created: 25.10.2015 20:23:05
 * Author:	George Lazarov
 * email:	lazarov.g@gmail.com
 */ 

#define F_CPU 16000000L

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define ROKE	(1)
#define RNOKE	(0)

#define MOTOROLA_MODE   0x00
#define WRITE           0xE0
#define READ            0x60
#define ADDR_ANSW       0xAA
#define CMD_ANSW        0x55

#define LINECONTROL  0x00      // r/w - 0x00
#define TRANSMITCTR  0x01      // r/w - 0x02
#define DIAGNCTRL    0x02      // r/w - 0x00
#define COMMANDREG   0x03      // w   - 0x00
#define LINESTAT     0x04      // r   - 0bx01xxx00
#define TRANSTAT     0x05      // r   - 0x00
#define LASTMSGSTAT  0x06      // r   - 0x00
#define LASTERRSTAT  0x07      // r   - 0x00
#define INTSTATUS    0x09      // r   - 0x80
#define INTENABLE    0x0A      // r/w - 0x80
#define INTRESET     0x0B      // w

// Channels
#define CHANNEL_ADDR( x)	( 0x10 + ( 0x08 * x))
#define CHANNELS     14
// Mailbox - data register
#define GETMAIL( x)         ( 0x80 + x)

// Buttons
/*   7  6  5  4  3  2  1  0   */
/*  su sd wu wd vu  vd sr ?   */
#define SRC   1
#define VDWN  2
#define VUP   3
#define WDWN  4
#define WUP   5
#define SDWN  6
#define SUP   7
#define CMD_WAIT_MS	(50)

typedef unsigned char byte;

// Resistance references for MCP4100
const byte source = 0xfd;      //  1.20 k
const byte att    = 0xf8;      //  3.50 k
const byte displ  = 0xf1;      //  5.75 k
const byte tup    = 0xeb;      //  8.00 k
const byte tdwn   = 0xe2;      // 11.25 k
const byte volup  = 0xd5;      // 16.00 k
const byte voldwn = 0xbe;      // 24.00 k
const byte band   = 0x4d;      // 62.75 k
const byte nocmd  = 0x00;      // max

// Global variables
volatile int error   = 0x00;   // TSS463C out of sync error
volatile byte interr = 0x00;

// Button states
volatile byte oldwheel   = 0x80;
volatile byte wheel      = 0x80;
volatile byte button     = 0x00;
volatile byte wuloop     = 0x00;
volatile byte wdwnloop   = 0x00;

// Functions declaration
byte spi_transfer( volatile byte data);
void tss_init();
void execCmd( const byte cmd);
void register_set( byte address, byte value);
byte register_get( byte address);
void motorolla_mode();

void setup()
{
	TIMSK0 = 0;
	DDRB |= (1 << DDB3) | (1 << DDB5) | (1 << DDB2) | (1 << DDB1) | ( 1 << DDB0);
	
	PORTB |= ( 0 << PINB0);
	PORTB |= ( 1 << PINB2);
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA);
	byte clr = SPSR;
	clr = SPDR;
	(void)clr;
	//_delay_ms( 10);
	
	// Enable ATMega Interrupts on INT0
	EICRA |= (1 << ISC01); // The falling edge of INT0 generates an interrupt request
	EIMSK |= (1 << INT0);  // Enable INT0
	tss_init();
};

void tss_init()
{
	motorolla_mode();
	_delay_ms( 10);

	// disable all channels
	for ( int i = 0 ; i < CHANNELS ; i++)
	{
		register_set( CHANNEL_ADDR( i) + 3, 0x0F);
		_delay_ms( 10);
	}

	// set Channel 0 to receive messages from ID 9C4
	// and receive data in address 0x80
	/*
		ID_MASK         0x07 ID_M [3:0] x x x x
		ID_MASK         0x06 ID_M [11:4]
		(No register)   0x05 x x x x x x x x
		(No register)   0x04 x x x x x x x x
		MESS_L/STA      0x03 M_L [4:0] CHER CHTx CHRx
		MESS_PTR        0x02 DRACK M_P [6:0]
		ID_TAG/CMD      0x01 ID_T [3: 0] EXT RAK RNW RTR
		ID_TAG          0x00 ID_T [11:4]
	*/
	register_set( CHANNEL_ADDR( 0) + 0, 0x9C); //  ID_TAG 9C4 - Radio Control
	register_set( CHANNEL_ADDR( 0) + 1, 0x41); //  ID_TAG, RNW = 0, RTR = 1
	register_set( CHANNEL_ADDR( 0) + 2, 0x00); //  MESS_PTR 0 ( 0x80 + 0) - MAILBOX ADDRESS
	register_set( CHANNEL_ADDR( 0) + 3, 0xf8); //  M_L [4:0] = 0x1F Frame with 30 DATA bytes , CHER = 0, CHTx = 0, CHRx = 0
	register_set( CHANNEL_ADDR( 0) + 6, 0x9C); //  ID_MASK 9C4
	register_set( CHANNEL_ADDR( 0) + 7, 0x40); //  ID_MASK 

	register_set( LINECONTROL, 0x20); // Clock Divider 0010 - 0x02 ( TSCLK = XTAL1 / n * 16) or ( TSCLK = 16000000 / 2 * 16) or 500000
	_delay_ms( 10);
	register_set( TRANSMITCTR, 0x32); // MT: 0011 ( Maximum Retries = 0x03) VER 001 fixed
	_delay_ms( 10);
	
	// Enable TSS Interrupts
	byte intEnable = 0x80; // Default value reset: 1xx0 0000
	intEnable |= ( 1 << ROKE) | ( 1 << ROKE);
	register_set( INTENABLE, intEnable);
	_delay_ms( 10);
	
	register_set( COMMANDREG, 0x10);  // ACTI - activate line
	_delay_ms( 10);
	error = 0;
};

void execCmd( const byte cmd)
{
	PORTB &= ~( 1 << PINB1);
	spi_transfer( 0x11);
	spi_transfer( cmd);
	PORTB |= ( 1 << PINB1);
	_delay_ms( CMD_WAIT_MS);
	PORTB &= ~( 1 << PINB1);
	spi_transfer( 0x11);
	spi_transfer( nocmd);
	PORTB |= ( 1 << PINB1);
};

void register_set( byte address, byte value)
{
	byte res = 0;

	PORTB &= ~( 1 << PINB2);

	_delay_ms( 4);
	res = spi_transfer( address);
	if ( res != ADDR_ANSW)
		error++;
	_delay_ms( 8);
	res = spi_transfer( WRITE);
	if ( res != CMD_ANSW)
		error++;
	_delay_ms( 15);
	spi_transfer( value);
	_delay_ms( 12);

	PORTB |= ( 1 << PINB2);
};

byte register_get( byte address)
{
	byte value = 0;

	PORTB &= ~( 1 << PINB2);

	_delay_ms( 4);
	value = spi_transfer( address);
	if ( value != ADDR_ANSW)
		error++;
	_delay_ms( 8);
	value = spi_transfer( READ);
	if ( value != CMD_ANSW)
		error++;
	_delay_ms( 15);
	value = spi_transfer( 0xff);
	_delay_ms( 12);

	PORTB |= ( 1 << PINB2);

	return value;
};

void motorolla_mode()
{
	byte value;

	PORTB &= ~( 1 << PINB2);

	_delay_ms( 4);
	value = spi_transfer( MOTOROLA_MODE);
	if ( value != ADDR_ANSW)
		error++;
	_delay_ms( 8);
	value = spi_transfer( MOTOROLA_MODE);
	if ( value != CMD_ANSW)
		error++;
	_delay_ms( 12);

	PORTB |= ( 1 << PINB2);
};

byte spi_transfer( volatile byte data)
{
	byte res;

	SPDR = data;
	while (!(SPSR & (1 << SPIF))) {};
	res = SPDR;

	return res;
};

int main(void)
{
	setup();
	sei();
	
	while (1)
	{
		if( !error) {
			
			button  = register_get( GETMAIL( 1));
			wheel   = register_get( GETMAIL( 2));

			if( wheel > oldwheel || wuloop != ( button & ( 1 << WUP))) {
				wuloop = ( button & ( 1 << WUP));
				oldwheel = wheel;
				PORTB |= ( 1 << PINB0);
				execCmd( tup);
				PORTB &= ~( 1 << PINB0);
			}
			else if( wheel < oldwheel || wdwnloop != ( button & ( 1 << WDWN))) {
				wdwnloop = ( button & ( 1 << WDWN));
				oldwheel = wheel;
				PORTB |= ( 1 << PINB0);
				execCmd( tdwn);
				PORTB &= ~( 1 << PINB0);
			}
			else if ( button & ( 1 << SRC)) {
				execCmd( source);
			}
			else if ( ( button & ( 1 << VUP)) && ( button & ( 1 << VDWN))) {
				execCmd( att);
			}
			else if ( button & ( 1 << VDWN)) {
				execCmd( voldwn);
			}
			else if ( button & ( 1 << VUP)) {
				execCmd( volup);
			}
			else if ( button & ( 1 << SDWN)) {
				execCmd( tdwn);
			}
			else if ( button & ( 1 << SUP)) {
				execCmd( tup);
			}
			
			register_set( CHANNEL_ADDR( 0) + 3, 0xf8);
		}
		else {
			tss_init();
		}
		_delay_ms(10);
	}
};

ISR(INT0_vect)
{
	//interr = register_get( INTSTATUS);
	//if( interr & ( 1 << ROKR) && interr & ( 1 << RNOKR))
	//register_set( INTRESET, 0x83);
	//inter |= ( 1 << 1) | (1 << 0);
	//register_set( INTRESET, 0x00);
};