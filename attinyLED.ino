#include <TinyWire.h> // https://github.com/lucullusTheOnly/TinyWire.git

#include "light_ws2812.h"
#include "ws2812_config.h"


// for attiny85
#define LED_BUILTIN	PB4

#define I2C_ADDR	0x10

// 120 = 2m @60pm or 4 metres @30pm
// 60 = 1m @60pm or 2 metres @30pm

// CANNOT exceed 127
#define MAXPIX 60

// how many leds we actually have
byte currentCount = MAXPIX;

struct cRGB led[MAXPIX];

//#define _PIPE_TO_NULL

#include <avr\power.h>

// the setup function runs once when you press reset or power the board
void setup() 
{
	// power
	//ADCSRA &= ~(1 << ADEN); //Disable ADC, saves ~230uA
	//adc_disable();

	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);

	// config TinyWire library for I2C slave functionality
	TinyWire.begin(I2C_ADDR);

	// sets callback for the event of a slave receive
	TinyWire.onReceive(onI2CReceive);
	TinyWire.onRequest(onI2CRequest);

	// clear the LEDS
	memset(&led, 0, sizeof(led));
	// state stays the same
	Display();


}


// the loop function runs over and over again forever
//void loop() 
//{
//	HandleI2C();
//}

enum stateMachine { smIdle=0, smPossibleWork=20, smSizing=100, smSetAll, smSetOne, smShifting, smRolling } ;

volatile stateMachine currentState = stateMachine::smIdle;

#define CMD_RESET	0	// turn it all off
#define CMD_SIZE	1	// actual number of LEDS
#define CMD_SETALL	2	// set all leds to RGB
#define CMD_SETONE	3	// set a single led - offset(0) RGB
#define CMD_SHIFT	4	// shift current set - signed byte (for L and R) RGB replace
//#define CMD_ROLL	5	// roll - signed byte

#define MAX_Q_DATA	4

unsigned dataCount = 0, dataOutstanding=0;
byte data[MAX_Q_DATA];

#define NEED_DATA(y,x)	{ dataCount=0; dataOutstanding=x; currentState=y; }



bool SumpData(byte theByte)
{
	data[dataCount++] = theByte;
	if (dataCount == dataOutstanding)
		return true;
	return false;
}

class circQueue
{

#define _CIRCQSIZE 32
protected:

	byte m_data[_CIRCQSIZE];
	// let the compiler do the work
	byte readCursor : 5, writeCursor : 5;

public:

	circQueue() :readCursor(0), writeCursor(0)
	{
	}

	int space()
	{
		return _CIRCQSIZE - available();
	}

	unsigned available()
	{
		if (readCursor == writeCursor)
			return 0;
		if (readCursor > writeCursor)
		{
			// use the wrap
			return ((int)writeCursor | 32) - readCursor;
		}
		return (int)(writeCursor - readCursor);
	}

	byte read()
	{
		if (readCursor == writeCursor)
			return -1;

		return m_data[readCursor++];
	}

	bool write(byte data)
	{
		// relying on bits width math
		if ((readCursor - 1) == writeCursor)
			return false;
		m_data[writeCursor++] = data;
		return true;
	}

};

circQueue theQueue;

//#define _TRY_SLEEP

#ifdef _TRY_SLEEP
#include <avr/sleep.h>
#endif

//void HandleI2C()
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
		return;
	}

	// loops, until all received bytes are read
	while (theQueue.available()>0) 
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
				Display();
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
					Display();
				}
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
				byte offset = data[0];
				// if there's no offset, there's nothing to do!
				if (offset)
				{
					int source = 0, dest = 0, size=currentCount, fillStart;
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
						fillStart = dest;
					}

					size -= abs(offset);

					// shift
					memmove( &led[dest], &led[source], sizeof(led[source])*size);

					// then fill the void ones
					size = abs(offset);
					for (unsigned fill = fillStart; size; size--, fill++)
					{
						led[fill].r = (uint8_t)data[1];
						led[fill].g = (uint8_t)data[2];
						led[fill].b = (uint8_t)data[3];
					}

					Display();
				}
				currentState = smPossibleWork;
			}
			break;
		case smIdle:
		case smPossibleWork:
			switch (readByte)
			{
			case CMD_RESET:
				//memset(&led, 0, sizeof(led));
				for (int each = 0; each < currentCount; each++)
					led[each].r = led[each].b = led[each].g = 0;
				// state stays the same
				Display();
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
#ifdef CMD_ROLL
			case CMD_ROLL:
				NEED_DATA(smRolling, 1);
				break;
#endif
			default:
				break;
			}
			break;
		}
	}

	currentState = smIdle;

}


void onI2CReceive(int howMany) 
{
	digitalWrite(LED_BUILTIN, (digitalRead(LED_BUILTIN)==HIGH)?LOW:HIGH);

	if (currentState == smIdle)
	{
		while (TinyWire.available())
			theQueue.write(TinyWire.read());

		currentState = smPossibleWork;
	}

}

void onI2CRequest(void)
{
	digitalWrite(LED_BUILTIN, (digitalRead(LED_BUILTIN) == HIGH) ? LOW : HIGH);

	// meh 
	byte result = currentState;
	// if we have more space left then biggest command + data ..
	if (theQueue.space() >= (MAX_Q_DATA+1))
	{
		result |= 128;
	}

	TinyWire.send(result);

}



void Display()
{
	ws2812_setleds(led, currentCount);
}



