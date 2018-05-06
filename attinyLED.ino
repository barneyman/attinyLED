#include <TinyWire.h> // https://github.com/lucullusTheOnly/TinyWire.git

#include "light_ws2812.h"
#include "ws2812_config.h"

// for attiny85
#define LED_BUILTIN		PB4
#define LED_WHITE		PB1

#define I2C_ADDR	0x10

// 120 = 2m @60pm or 4 metres @30pm
// 60 = 1m @60pm or 2 metres @30pm


//#define _USE_PALETTE

#ifdef _USE_PALETTE

const struct cRGB ledPalette[] PROGMEM = {
	{ 0, 0, 0 },		// black
	{ 255, 255, 255 },	// white
	{ 255,0,0 },		// red
	{ 0, 255, 0 },		// lime
	{ 0,0,255 },		// blue
	{ 255,255,0 },		// yellow
	{ 0,255,255 },		// cyan
	{ 255,0,255 },		// magenta
	{ 192,192,192 },		// silver
	{ 128,128,128 },		// grey
	{ 128,0,0 },			// maroon
	{ 128,128,0 },		// olive
	{ 0,128,0 },			// green
	{ 128,0,128 },		// purple
	{ 0,128,128 },		// teal
	{ 0,0,128 }			// navy

};

#define _COLOR_PALLETE_BLACK		0
#define _COLOR_PALLETE_WHITE		1
#define _COLOR_PALLETE_RED			2
#define _COLOR_PALLETE_LIME			3
#define _COLOR_PALLETE_BLUE			4
#define _COLOR_PALLETE_YELLOW		5
#define _COLOR_PALLETE_CYAN			6
#define _COLOR_PALLETE_MAGENTA		7
#define _COLOR_PALLETE_SILVER		8
#define _COLOR_PALLETE_GREY			9
#define _COLOR_PALLETE_MAROON		10
#define _COLOR_PALLETE_OLIVE		11
#define _COLOR_PALLETE_GREEN		12
#define _COLOR_PALLETE_PURPLE		13
#define _COLOR_PALLETE_TEAL			14
#define _COLOR_PALLETE_NAVY			15
#define _COLOR_PALLETE_USER1		16
#define _COLOR_PALLETE_USER2		17
#define _COLOR_PALLETE_USER3		18
#define _COLOR_PALLETE_USER4		19
#define _COLOR_PALLETE_USER5		20
#define _COLOR_PALLETE_USER6		21
#define _COLOR_PALLETE_USER7		22
#define _COLOR_PALLETE_USER8		23
#define _COLOR_PALLETE_USER9		24
#define _COLOR_PALLETE_USER10		25
#define _COLOR_PALLETE_USER11		26
#define _COLOR_PALLETE_USER12		27
#define _COLOR_PALLETE_USER13		28
#define _COLOR_PALLETE_USER14		29
#define _COLOR_PALLETE_USER15		30
#define _COLOR_PALLETE_USER16		31

#define MAXPIX 2


byte leds[300];
struct cRGB userPalette[8];

#else

#define MAXPIX 90

#endif


struct cRGB led[MAXPIX];


// how many leds we actually have
byte currentCount = MAXPIX;




#include <avr\power.h>
#include <avr\wdt.h>





#define _DISABLE_TIMER

// the setup function runs once when you press reset or power the board
void setup() 
{
	// power

	ADCSRA &= ~(1 << ADEN); //Disable ADC, saves ~230uA



#ifdef 	_USE_PALETTE
	// just to stop the linker hosing an unused array
	memset(&leds, 0, sizeof(leds));
	memset(&userPalette, 0, sizeof(userPalette));
#endif


	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(LED_WHITE, OUTPUT);

	digitalWrite(LED_BUILTIN, LOW);
	digitalWrite(LED_WHITE, LOW);

	for(int flash=0;flash<3;flash++)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		digitalWrite(LED_WHITE, HIGH);
		delay(200);
		digitalWrite(LED_BUILTIN, LOW);
		digitalWrite(LED_WHITE, LOW);
		delay(200);
	}

#ifdef _DISABLE_TIMER
	// turn timer0 off, it MAY help with the i2c lockups?
	// it may have made a difference ... not convinced
	TCCR0B &= 0B11111000;



#endif


	// turn watchdog off
	MCUSR = 0;
	wdt_disable();

	// clear the LEDS
	memset(&led, 0, sizeof(led));
	// state stays the same
	Display();

	// config TinyWire library for I2C slave functionality
	TinyWire.begin(I2C_ADDR);

	// sets callback for the event of a slave receive
	TinyWire.onReceive(onI2CReceive);
	TinyWire.onRequest(onI2CRequest);


}


enum stateMachine { smIdle=0, smPossibleWork=20, smSizing=100, smSetAll, smSetOne, smShifting, smRolling, smInverting } ;

volatile stateMachine currentState = stateMachine::smIdle;

#define CMD_RESET	0	// turn it all off
#define CMD_SIZE	1	// actual number of LEDS
#define CMD_SETALL	2	// set all leds to RGB
#define CMD_SETONE	3	// set a single led - offset(0) RGB
#define CMD_SHIFT	4	// shift current set - signed byte (for L and R) RGB replace
//#define CMD_ROLL	5	// roll - signed byte
#define CMD_DISPLAY	6	// shunt out to the LEDS - beware, interrupts get cleared, so I2C will fail
#define CMD_INVERT	7	// invert all rgbs

// size of the biggest command data (less the command byte)
#define MAX_Q_DATA					4
#define MAX_Q_COMMAND_AND_DATA		(MAX_Q_DATA+1)

unsigned dataCount = 0, dataOutstanding=0;
byte data[MAX_Q_COMMAND_AND_DATA];

#define NEED_DATA(y,x)	{ dataCount=0; dataOutstanding=x; currentState=y; }



bool SumpData(byte theByte)
{
	data[dataCount++] = theByte;
	return (dataCount == dataOutstanding);
}

class circQueue
{

#define _CIRCQSIZE		8
#define _CIRQSIZEBITS	3
protected:

	byte m_data[_CIRCQSIZE];
	// let the compiler do the work
	volatile byte readCursor : _CIRQSIZEBITS, writeCursor : _CIRQSIZEBITS;
	volatile byte availBytes;

public:

	circQueue() :readCursor(0), writeCursor(0), availBytes(0)
	{
	}

	int space()
	{
		return _CIRCQSIZE - available();
	}

	unsigned available()
	{
		return availBytes;
	}

	byte read()
	{
		byte ret = -1;

		{
			// remember int state
			uint8_t tmp = SREG;
			// clear (may be cleared already)
			cli();

			if (readCursor != writeCursor)
			{
				// 
				availBytes--;
				ret = m_data[readCursor++];
			}

			// reinstate interrupts, maybe
			SREG = tmp;
		}

		return ret;
	}

	bool write(byte data)
	{
		bool ret = false;

		{
			// remember int state
			uint8_t tmp = SREG;
			// clear (may be cleared already)
			cli();

			// relying on bits width math
			if ((readCursor - 1) != writeCursor)
			{
				m_data[writeCursor++] = data;
				availBytes++;
				ret = true;
			}
			// reinstate interrupts, maybe
			SREG = tmp;
		}

		return ret;
	}

};

circQueue theQueue;

//#define _TRY_SLEEP

#ifdef _TRY_SLEEP
#include <avr/sleep.h>
#endif

//#define _CHECK_STACK 128

#ifdef _CHECK_STACK

// defined by GCC
extern uint8_t _end;
extern uint8_t __stack;

#define STACK_CANARY	0xc5

// https://www.nongnu.org/avr-libc/user-manual/mem_sections.html
void StackPaint(void) __attribute__((naked)) __attribute__((section(".init1")));

void StackPaint(void)
{
	__asm volatile (
		"    ldi r30,lo8(_end)\n"
		"    ldi r31,hi8(_end)\n"
		"    ldi r24,lo8(0xc5)\n" /* STACK_CANARY = 0xc5 */
		"    ldi r25,hi8(__stack)\n"
		"    rjmp .cmp\n"
		".loop:\n"
		"    st Z+,r24\n"
		".cmp:\n"
		"    cpi r30,lo8(__stack)\n"
		"    cpc r31,r25\n"
		"    brlo .loop\n"
		"    breq .loop"::);
}



bool CheckStackMinimum(uint16_t minRequired)
{
	return (StackRoomCount() < minRequired)?false:true;
}

// walk from end of bss to where the stack has been, max
uint16_t StackRoomCount(void)
{
	//const uint8_t *p = &_end;
	const uint8_t *p = (&__stack)-_CHECK_STACK;
	uint16_t       c = 0;

	while ((*p == STACK_CANARY) && (p <= &__stack))
	{
		p++;
		c++;
	}

	return c;
}

#endif


//#define _SELF_POPULATE

#define _HANDLE_IN_ISR

// disable wdt
void get_mcusr(void) \
__attribute__((naked)) \
__attribute__((section(".init3")));
void get_mcusr(void)
{
	MCUSR = 0;
	wdt_disable();
}


void  loop()
{

	if (currentState == smIdle)
	{
#ifdef _TRY_SLEEP
		// go to sleep
		set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
		sleep_enable();
		sleep_mode();                        // System actually sleeps here
		sleep_disable();                     // System continues execution here when watchdog timed out 
#endif
	}
}


// called from the OnReceive ISR
void HandleQueue()
{
#ifdef _CHECK_STACK
	if (!CheckStackMinimum(8))
	{
		digitalWrite(LED_WHITE, HIGH);
	}
#endif

	// loops, until all received bytes are read
	while (theQueue.available())
	{
		byte readByte = theQueue.read();

		switch (currentState)
		{
		case smSizing:
			if (SumpData(readByte))
			{
				currentCount = data[0];
				// bounds check
				if (currentCount > MAXPIX)
					currentCount = MAXPIX;

				// go around again
				currentState = smPossibleWork;
			}
			break;
		case smSetAll:
			if (SumpData(readByte))
			{
				for (unsigned each = 0; each < currentCount; each++)
				{
					led[each].r = (uint8_t)data[0];
					led[each].g = (uint8_t)data[1];
					led[each].b = (uint8_t)data[2];
				}
				// go around again
				currentState = smPossibleWork;
			}
			break;
		case smSetOne:
			if (SumpData(readByte))
			{
				uint8_t offset = (uint8_t)data[0];
				if (offset < currentCount)
				{
					led[(uint8_t)offset].r = (uint8_t)data[1];
					led[(uint8_t)offset].g = (uint8_t)data[2];
					led[(uint8_t)offset].b = (uint8_t)data[3];
				}
				// go around again
				currentState = smPossibleWork;
			}
			break;
		case smRolling:
#ifdef CMD_ROLL
			if (SumpData(readByte))
			{
				int offset = (int)data[0];
				// if there's no offset, there's nothing to do!
				if (offset)
				{
					int source = 0, dest = 0, size = currentCount, fillStart = 0;
					if (offset > 0)
					{
						// shift right
						dest += offset;
					}
					else
					{
						// shift left
						// yes - double negative
						source -= offset;
						fillStart = dest;
					}

					size -= abs(offset);

					for (unsigned mover = 0; mover < currentCount; mover++)
					{
						led[mover]
					}
				}
#endif
				break;
		case smShifting:
			if (SumpData(readByte))
			{
				signed char offset = (signed char)data[0];
				// if there's no offset, there's nothing to do!
				if (offset)
				{
					int source = 0, dest = 0, size = currentCount, fillStart;
					if (offset > 0)
					{
						// shift right
						dest += offset;
						fillStart = 0;
					}
					else
					{
						// shift left
						// yes - double negative
						source -= offset;
						fillStart = currentCount + offset;
					}

					size -= abs(offset);

					// shift
					memmove(&led[dest], &led[source], sizeof(led[source])*size);

					// then fill the void ones
					size = abs(offset);

					for (unsigned fill = fillStart; size; size--, fill++)
					{
						led[fill].r = (uint8_t)data[1];
						led[fill].g = (uint8_t)data[2];
						led[fill].b = (uint8_t)data[3];
					}

				}
				currentState = smPossibleWork;
			}
			break;

		case smInverting:
			if (SumpData(readByte))
			{
				int mask = data[0];
				for (unsigned each = 0; each < currentCount; each++)
				{
					led[each].r = (~led[each].r)&mask;
					led[each].g = (~led[each].g)&mask;
					led[each].b = (~led[each].b)&mask;
				}
			}
			currentState = smPossibleWork;
			break;
			//case smIdle:
		case smPossibleWork:
			switch (readByte)
			{
			case CMD_RESET:
				memset(&led, 0, sizeof(led));
				// state stays the same
				currentState = smPossibleWork;
				break;
			case CMD_DISPLAY:
				Display();
				currentState = smPossibleWork;
				break;
			case CMD_SIZE:
				NEED_DATA(smSizing, 1);
				break;
			case CMD_SETALL:
				NEED_DATA(smSetAll, 3);
				break;
			case CMD_SETONE:
				NEED_DATA(smSetOne, 4);
				break;
			case CMD_SHIFT:
				NEED_DATA(smShifting, 4);
				break;
			case CMD_INVERT:
				NEED_DATA(smInverting, 1);
				break;

#ifdef CMD_ROLL
			case CMD_ROLL:
				NEED_DATA(smRolling, 1);
				break;
#endif
			default:
				break;
			}
			break;
		default:
			break;
		}
	}

	// if we leave the available loop guessing there's more, assume none
	// clear interrupts so the ISR doesn't change this underneath us
	if (currentState == smPossibleWork)
	{ 
		currentState = smIdle;
	}

}



void onI2CReceive(int howMany) 
{
	// we are NOT assured of getting all the bytes in a endTransmission in one chunk
	// so, don't assume we will

	{
		while (TinyWire.available())
		{
			theQueue.write(TinyWire.read());
		}

		if(currentState==smIdle)
			currentState = smPossibleWork;
	}

	// take as long as you need - no wdt, and nothing else running on the chip
	HandleQueue();

}

void onI2CRequest(void)
{

	// meh 
	byte result = currentState;
	// if we have more space left then biggest command + data ..
	if (theQueue.space() == _CIRCQSIZE)
	{
		result |= 64;
	}
	else if (theQueue.space() >= (MAX_Q_COMMAND_AND_DATA))
	{
		result |= 128;
	}

	TinyWire.send(result);
}

#define _FAKE

void Display()
{
#ifdef _FAKE
	digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN)==HIGH?LOW:HIGH);
	//for (int blink = 0; blink < 5; blink++)
	//{
	//	digitalWrite(LED_BUILTIN, HIGH);
	//	for (int loop = 0; loop < 1000; loop++);
	//	digitalWrite(LED_BUILTIN, LOW);
	//	for (int loop = 0; loop < 1000; loop++);
	//	digitalWrite(LED_BUILTIN, HIGH);
	//	for (int loop = 0; loop < 1000; loop++);
	//	digitalWrite(LED_BUILTIN, LOW);
	//}

#else
	ws2812_setleds(led, currentCount);
#endif
}



