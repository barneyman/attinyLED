#include <TinyWire.h> // https://github.com/lucullusTheOnly/TinyWire.git
#include "light_ws2812.h"
#include "ws2812_config.h"



// define this to use the canned palette = it allows access to more LEDS (because each is referenced by one byte, not 3)
#define _USE_PALETTE

// this enables macro support (you lose some leds)
#define _USE_MACROS
#define _RUN_MACRO_ON_BUTTON	PB1

// I2C address used by this chip - feel free to change it 
#define I2C_ADDR	0x10

//#define _XSISTOR_FOR_ON PB1

// for attiny85
#define LED_BUILTIN		PB4
#ifndef _XSISTOR_FOR_ON
#define LED_WHITE		PB1
#endif


template <int _CIRCQSIZE, int _CIRQSIZEBITS>
class circQueueT 
{

protected:

	byte m_data[_CIRCQSIZE];
	// let the compiler do the work
	volatile byte readCursor : _CIRQSIZEBITS, writeCursor : _CIRQSIZEBITS;
	volatile byte readCursorState : _CIRQSIZEBITS, writeCursorState : _CIRQSIZEBITS;
	volatile byte availBytes, availBytesState;

public:

	circQueueT()
	{
		reset();
	}

	unsigned available()
	{
		return availBytes;
	}

	int size()
	{
		return _CIRCQSIZE;
	}

	int space()
	{
		return _CIRCQSIZE - available();
	}

	void reset()
	{
		readCursor = writeCursor = availBytes=0;
		readCursorState = writeCursorState = availBytesState = 0;
	}

	void pushState()
	{
		readCursorState = readCursor;
		writeCursorState = writeCursor;
		availBytesState = availBytes;
	}

	void popState()
	{
		readCursor=readCursorState;
		writeCursor = writeCursorState;
		availBytes = availBytesState;
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


#ifdef _USE_MACROS
#include <twi.h>
// the max in the TWI code i think, so lets not exceed that
#define MAX_MACRO	TWI_RX_BUFFER_SIZE

circQueueT<32,5> macro;
#endif

//circQueueT<8, 3> theQueueTemp;


#ifdef _USE_PALETTE

// YES - these palettes are 4 bytes wide, even tho only 3 are used - it makes the chip math easier in the display functions

uint8_t paletteDiv = 0;

const struct cRGBW ledPalette[] PROGMEM = {
// WARNING - GR Rd Bl !!!!!
	{ 0, 0, 0 },		// black
	{ 255, 255, 255 },	// white
	{ 0,255,0 },		// red
	{ 255,0,  0 },		// lime
	{ 0,0,255 },		// blue
	{ 255,255,0 },		// yellow
	{ 255,0,255 },		// cyan
	{ 0,255,255 },		// magenta
	{ 192,192,192 },	// silver
	{ 128,128,128 },	// grey
	{ 0,128,0 },		// maroon
	{ 128,128,0 },		// olive
	{ 128,0,0 },		// green
	{ 0,128,128 },		// purple
	{ 128,0,128 },		// teal
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

// 5m @ 60pm 
#ifdef _USE_MACROS
#define MAXPIX 280
#else
#define MAXPIX 300
#endif
#define MAX_USER_PALETTE	8
byte led[MAXPIX];
struct cRGBW userPalette[MAX_USER_PALETTE];

#else

// 120 = 2m @60pm or 4 metres @30pm
// 60 = 1m @60pm or 2 metres @30pm
#ifdef _USE_MACROS
#define MAXPIX 80
#else
#define MAXPIX 90
#endif
struct cRGB led[MAXPIX];

#endif




// requestData returned flags
#define _FLAG_ROOM_IN_QUEUE	128
#define _FLAG_QUEUE_FLUSHED	64
#define _FLAG_PALETTE_MODE	32
#define _FLAG_MACROS		16

// sendData commands
#define CMD_RESET	0	// turn it all off
#define CMD_SIZE	1	// actual number of LEDS
#define CMD_SETALL	2	// set all leds to RGB
#define CMD_SETONE	3	// set a single led - offset(0) RGB
#define CMD_SHIFT	4	// shift current set - signed byte (for L and R) RGB replace
//#define CMD_ROLL	5	// roll - signed byte
#define CMD_DISPLAY	6	// shunt out to the LEDS - beware, interrupts get cleared, so I2C will fail
#define CMD_INVERT	7	// invert all rgbs
// only works when _XSISTOR_FOR_ON define
#define CMD_ON_OFF		8	// + on/off byte
// palette commands
#define CMD_SETALL_PALETTE		10	// set all leds to RGB
#define CMD_SETONE_PALETTE		11	// set a single led - offset(0) RGB
#define CMD_SHIFT_PALETTE		12	// shift current set - signed byte (for L and R) RGB replace
#define CMD_DIV_PALETTE			13	// apply div to palette colours - one byte = rgb >> div
#define CMD_USER_PALETTE_SET	14	// set one of the user colours - 4 bytes - offset r g b 
// macros
#define CMD_SET_MACRO			15	// len + len bytes
#define CMD_RUN_MACRO			16	// do it

// how many leds we actually have
unsigned currentCount = MAXPIX;




#include <avr\power.h>
#include <avr\wdt.h>



// this turns off the delay() timer
#define _DISABLE_TIMER

// the setup function runs once when you press reset or power the board
void setup() 
{
	// settle down!
	delay(200);

	// power
	ADCSRA &= ~(1 << ADEN); //Disable ADC, saves ~230uA





	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
#ifdef _XSISTOR_FOR_ON
	pinMode(_XSISTOR_FOR_ON, OUTPUT);
#else
#ifdef _RUN_MACRO_ON_BUTTON
	pinMode(_RUN_MACRO_ON_BUTTON, INPUT_PULLUP);
	attachInterrupt(_RUN_MACRO_ON_BUTTON, RunOnButton, LOW);
#else
	pinMode(LED_WHITE, OUTPUT);
#endif
#endif

	digitalWrite(LED_BUILTIN, LOW);
#ifdef _XSISTOR_FOR_ON
	digitalWrite(_XSISTOR_FOR_ON, LOW);
#else
	digitalWrite(LED_WHITE, LOW);
#endif

	for(int flash=0;flash<3;flash++)
	{
		digitalWrite(LED_BUILTIN, HIGH);
#ifndef _XSISTOR_FOR_ON
		digitalWrite(LED_WHITE, HIGH);
#endif
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
#ifndef _XSISTOR_FOR_ON
		digitalWrite(LED_WHITE, LOW);
#endif
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

#ifdef 	_USE_PALETTE
	memset(&userPalette, 0, sizeof(userPalette));
#endif


	// state stays the same
	Display();

	// config TinyWire library for I2C slave functionality
	TinyWire.begin(I2C_ADDR);

	// sets callback for the event of a slave receive
	TinyWire.onReceive(onI2CReceive);
	TinyWire.onRequest(onI2CRequest);


}


enum stateMachine {		smIdle=0, smPossibleWork, smSizing, smOnOff,
						smSetAll=10, smSetOne, smShifting, smRolling, smInverting, 
						smSetAllPalette=20, smSetOnePalette, smShiftingPalette, smRollingPalette, smInvertingPalette, smDivPalette, smUserPalette,
						smMacroGetLen=30, smMacroGet
				} ;

volatile stateMachine currentState = stateMachine::smIdle;


// size of the biggest command data (less the command byte)
#define MAX_Q_DATA					4
#define MAX_Q_COMMAND_AND_DATA		(MAX_Q_DATA+1)

unsigned dataCount = 0, dataOutstanding=0;
byte data[MAX_Q_COMMAND_AND_DATA];

#define NEED_DATA(y,x)	{ dataCount=0; dataOutstanding=x; currentState=y; }

#ifdef _USE_MACROS
bool SumpMacro(byte theByte)
{
	// 
	macro.write(theByte);
	dataCount++;
	return (dataCount == dataOutstanding);
}

bool SumpBucket(byte theByte)
{
	// dev/nul
	dataCount++;
	return (dataCount == dataOutstanding);
}

#endif


bool SumpData(byte theByte)
{
	data[dataCount++] = theByte;
	return (dataCount == dataOutstanding);
}



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



// disable wdt
void get_mcusr(void) \
__attribute__((naked)) \
__attribute__((section(".init3")));
void get_mcusr(void)
{
	MCUSR = 0;
	wdt_disable();
}

#define _DISPLAY_IN_LOOP
#ifdef _DISPLAY_IN_LOOP
volatile bool displayNow = false;
#endif

volatile bool runMacro=false;

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

	if (runMacro)
	{
		currentState = smPossibleWork;

		HandleQueue(runMacro);
		runMacro = false;
		// and display
		Display();
		digitalWrite(LED_BUILTIN, LOW);
	}

	// flag set in the recv ISR
	if (displayNow)
	{
		Display();
		displayNow = false;
	}

}


// called from the OnReceive ISR
void HandleQueue(bool domacro)
{
#ifdef _CHECK_STACK
	if (!CheckStackMinimum(8))
	{
		digitalWrite(LED_WHITE, HIGH);
	}
#endif

	// loops, until all received bytes are read
	while (domacro?macro.available():TinyWire.available())
	{
		byte readByte = (domacro ? macro.read(): TinyWire.read());

		switch (currentState)
		{
		case smMacroGet:
			// we cannot do this IN a macro
			if (domacro)
			{
				if (SumpBucket(readByte))
				{
					// go around again
					currentState = smPossibleWork;
				}
			}
			// sumps to a DIFFERENT buffer
			else if (SumpMacro(readByte))
			{
				// go around again
				currentState = smPossibleWork;
				// and push the state of the queue - every time we run we pop it back
				macro.pushState();

				if (macro.available() == 9)
					digitalWrite(LED_BUILTIN, HIGH);
			}
			break;
		case smMacroGetLen:
			if (SumpData(readByte))
			{
				// go around again
				NEED_DATA(smMacroGet,data[0]);
			}
			break;
		case smOnOff:
			if (SumpData(readByte))
			{
#ifdef _XSISTOR_FOR_ON
				digitalWrite(_XSISTOR_FOR_ON, data[0]?HIGH:LOW);
#endif
				// go around again
				currentState = smPossibleWork;
			}
			break;

		case smDivPalette:
			if (SumpData(readByte))
			{
				paletteDiv = data[0];
				// bounds check
				paletteDiv &= 7;

				// go around again
				currentState = smPossibleWork;
			}
			break;

		case smUserPalette:
			if (SumpData(readByte))
			{
				int offset = data[0];
				offset -= _COLOR_PALLETE_USER1;

				if (offset < MAX_USER_PALETTE)
				{

					userPalette[offset].r = data[1];
					userPalette[offset].g = data[2];
					userPalette[offset].b = data[3];
				}

				// go around again
				currentState = smPossibleWork;
			}
			break;

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
		case smSetAllPalette:
			if (SumpData(readByte))
			{
#ifdef _USE_PALETTE
				for (unsigned each = 0; each < currentCount; each++)
				{
					led[each] = (uint8_t)data[0];
				}
#endif
				// go around again
				currentState = smPossibleWork;
			}
			break;
		case smSetAll:
			if (SumpData(readByte))
			{
#ifndef _USE_PALETTE
				for (unsigned each = 0; each < currentCount; each++)
				{
					led[each].r = (uint8_t)data[0];
					led[each].g = (uint8_t)data[1];
					led[each].b = (uint8_t)data[2];
				}
#endif
				// go around again
				currentState = smPossibleWork;
			}
			break;
		case smSetOnePalette:
			if (SumpData(readByte))
			{
#ifdef _USE_PALETTE
				uint8_t offset = (uint8_t)data[0];
				if (offset < currentCount)
				{
					led[(uint8_t)offset] = (uint8_t)data[1];
				}
#endif
				// go around again
				currentState = smPossibleWork;
			}
			break;
		case smSetOne:
			if (SumpData(readByte))
			{
#ifndef _USE_PALETTE
				uint8_t offset = (uint8_t)data[0];
				if (offset < currentCount)
				{
					led[(uint8_t)offset].r = (uint8_t)data[1];
					led[(uint8_t)offset].g = (uint8_t)data[2];
					led[(uint8_t)offset].b = (uint8_t)data[3];
				}
#endif
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
		case smShiftingPalette:
			if (SumpData(readByte))
			{
#ifdef _USE_PALETTE
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
						led[fill] = (uint8_t)data[1];
					}

				}
#endif
				currentState = smPossibleWork;
			}
			break;

		case smShifting:
			if (SumpData(readByte))
			{
#ifndef _USE_PALETTE
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
#endif
				currentState = smPossibleWork;
			}
			break;
		case smInvertingPalette:
			if (SumpData(readByte))
			{
#ifdef _USE_PALETTE
				// TODO - this doesnt work, logically
				int mask = data[0];
				for (unsigned each = 0; each < currentCount; each++)
				{
					led[each] = (~led[each])&mask;
				}
#endif
				currentState = smPossibleWork;
			}
			break;
		case smInverting:
			if (SumpData(readByte))
			{
#ifndef _USE_PALETTE
				int mask = data[0];
				for (unsigned each = 0; each < currentCount; each++)
				{
					led[each].r = (~led[each].r)&mask;
					led[each].g = (~led[each].g)&mask;
					led[each].b = (~led[each].b)&mask;
				}
#endif
				currentState = smPossibleWork;
			}
			break;
			//case smIdle:
		case smPossibleWork:
			switch (readByte)
			{
			case CMD_RUN_MACRO:
				// we CANNOT do this while we're in macro!
				if (!domacro)
				{
					// pop the queue back to its useful state
					macro.popState();
					// flag to run in loop()
					runMacro = true;
				}
				currentState = smPossibleWork;
				break;

			case CMD_SET_MACRO:
				// we need the length first
				NEED_DATA(smMacroGetLen, 1);
				// and reset the macro queue
				macro.reset();
				break;

			case CMD_ON_OFF:
				NEED_DATA(smOnOff, 1);
				break;
			case CMD_RESET:
				memset(&led, 0, sizeof(led));
				// state stays the same
				currentState = smPossibleWork;
				break;
			case CMD_DISPLAY:
#ifdef _DISPLAY_IN_LOOP
				displayNow = true;
#else
				Display();
#endif
				currentState = smPossibleWork;
				break;
			case CMD_SIZE:
				NEED_DATA(smSizing, 1);
				break;
			case CMD_DIV_PALETTE:
				NEED_DATA(smDivPalette, 1);
				break;
			case CMD_SETALL_PALETTE:
				NEED_DATA(smSetAllPalette, 1);
				break;
			case CMD_SETALL:
				NEED_DATA(smSetAll, 3);
				break;
			case CMD_SETONE_PALETTE:
				NEED_DATA(smSetOnePalette, 2);
				break;
			case CMD_SETONE:
				NEED_DATA(smSetOne, 4);
				break;
			case CMD_SHIFT_PALETTE:
				NEED_DATA(smShiftingPalette, 2);
				break;
			case CMD_SHIFT:
				NEED_DATA(smShifting, 4);
				break;
			case CMD_INVERT:
				NEED_DATA(smInverting, 1);
				break;
			case CMD_USER_PALETTE_SET:
				NEED_DATA(smUserPalette, 4);
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

	if (currentState == smIdle)
		currentState = smPossibleWork;

	//while (TinyWire.available())
	//{
	//	theQueueTemp.write(TinyWire.read());
	//}

	// take as long as you need - no wdt, and nothing else running on the chip
	HandleQueue(false);

}


void onI2CRequest(void)
{

	// meh 
	byte result = currentState;
	// if we have more space left then biggest command + data ..
	//if (TinyWire.available() == theQueueTemp.size())
	{
		result |= _FLAG_QUEUE_FLUSHED;
	}
	//else if (theQueueTemp.space() >= (MAX_Q_COMMAND_AND_DATA))
	{
		result |= _FLAG_ROOM_IN_QUEUE;
	}

#ifdef _USE_PALETTE
	result |= _FLAG_PALETTE_MODE;
#endif

#ifdef _USE_MACROS
	result |= _FLAG_MACROS;
#endif

	TinyWire.send(result);
}

#ifdef _RUN_MACRO_ON_BUTTON
void RunOnButton()
{
	macro.popState();
	runMacro = true;
}
#endif


void Display()
{
#ifndef _USE_PALETTE
	ws2812_setleds(led, currentCount);
#else
	ws2812_sendarray_mask_palette(userPalette, ledPalette, led, currentCount, paletteDiv, _BV(ws2812_pin));
#endif
}



