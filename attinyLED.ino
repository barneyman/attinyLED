#include <TinyWire.h> // https://github.com/lucullusTheOnly/TinyWire.git

#include "light_ws2812.h"
#include "ws2812_config.h"

// for attiny85
#define LED_BUILTIN		PB4
#define LED_WHITE		PB1

#define I2C_ADDR	0x10

// 120 = 2m @60pm or 4 metres @30pm
// 60 = 1m @60pm or 2 metres @30pm

#define MAXPIX 90

// how many leds we actually have
byte currentCount = MAXPIX;

struct cRGB led[MAXPIX];

//#define _PIPE_TO_NULL

#include <avr\power.h>
#include <avr\wdt.h>




// the setup function runs once when you press reset or power the board
void setup() 
{
	// power
	//ADCSRA &= ~(1 << ADEN); //Disable ADC, saves ~230uA
	//adc_disable();

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


	// turn watchdog off
	//MCUSR = 0;
	//wdt_disable();

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

#define _CIRCQSIZE		32
#define _CIRQSIZEBITS	5
protected:

	byte m_data[_CIRCQSIZE];
	// let the compiler do the work
	volatile byte readCursor : _CIRQSIZEBITS, writeCursor : _CIRQSIZEBITS;
	volatile signed availBytes;

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

		//if (readCursor == writeCursor)
		//	return 0;
		//if (readCursor > writeCursor)
		//{
		//	// use the wrap
		//	return (_CIRCQSIZE-(unsigned)readCursor) + (unsigned)writeCursor;
		//}
		//return (int)(writeCursor - readCursor);
	}

	byte read()
	{
		if (readCursor == writeCursor)
			return -1;
		availBytes--;
		return m_data[readCursor++];
	}

	bool write(byte data)
	{
		// relying on bits width math
		if ((readCursor - 1) == writeCursor)
			return false;
		m_data[writeCursor++] = data;
		availBytes++;
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

	digitalWrite(LED_BUILTIN, LOW);

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
						fillStart = currentCount+offset;
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
				//memset(&led, 0, sizeof(led));
				memset(&led, 4, sizeof(led));
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
				//digitalWrite(LED_BUILTIN, (digitalRead(LED_BUILTIN) == HIGH) ? LOW : HIGH);
				//digitalWrite(LED_WHITE, (digitalRead(LED_WHITE) == HIGH) ? LOW : HIGH);
				break;
			}
			break;
		default:
			break;
		}
	}

	if (currentState != smPossibleWork)
	{
		digitalWrite(LED_BUILTIN, HIGH);
	}

	// if we leave the available loop guessing there's more, assume none
	// clear interrupts so the ISR doesn't change this underneath us
	cli();
	if(currentState==smPossibleWork)
		currentState = smIdle;
	sei();
}



void onI2CReceive(int howMany) 
{

	digitalWrite(LED_WHITE, LOW);

	// we are NOT assured of getting all the bytes in a endTransmission in one chunk
	// so, don't assume we will

	if (theQueue.space() >= howMany)
	{
		while (TinyWire.available())
		{
			byte read = TinyWire.read();

			if (!theQueue.write(read))
			{
				digitalWrite(LED_WHITE, HIGH);
			}
		}

		if(currentState==smIdle)
			currentState = smPossibleWork;
	}
	else
	{
		// pipe to null
		//while (TinyWire.available())
		//	TinyWire.read();
	}
}

void onI2CRequest(void)
{
	//digitalWrite(LED_BUILTIN, (digitalRead(LED_BUILTIN) == HIGH) ? LOW : HIGH);

	// meh 
	byte result = currentState;
	// if we have more space left then biggest command + data ..
	if (theQueue.space() >= (MAX_Q_DATA + 1))
	{
		result |= 128;
	}

	result = TinyWire.send(result);

	//if (!result)
	//{
	//	digitalWrite(LED_BUILTIN, HIGH);
	//}
	//else
	//{
	//	digitalWrite(LED_BUILTIN, LOW);
	//}

}



void Display()
{
	ws2812_setleds(led, currentCount);
}



