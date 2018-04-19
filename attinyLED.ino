
#include <TinyWire.h>

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


// the setup function runs once when you press reset or power the board
void setup() 
{
	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);

	// config TinyWire library for I2C slave functionality
	TinyWire.begin(I2C_ADDR);

	// sets callback for the event of a slave receive
	//TinyWire.onReceive(onI2CReceive);

	// clear the LEDS
	memset(&led, 0, sizeof(led));
	// state stays the same
	Display();


}


// the loop function runs over and over again forever
void loop() 
{
	HandleI2C();
}

enum stateMachine { smIdle, smSizing, smSetAll, smSetOne, smShifting, smRolling } currentState = stateMachine::smIdle;

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

void HandleI2C()
{
	// loops, until all received bytes are read
	while (TinyWire.available()>0) 
	{
		byte readByte = TinyWire.read();

		switch (currentState)
		{
		case smSizing:
			if (SumpData(readByte))
			{
				currentCount = data[0];
				// bounds check
				if (currentCount > MAXPIX)
					currentCount = MAXPIX;
				currentState = smIdle;
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
				currentState = smIdle;
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
				currentState = smIdle;
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
				currentState = smIdle;
			}
			break;
		case smIdle:
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

}


void onI2CReceive(int howMany) 
{
	digitalWrite(LED_BUILTIN, (digitalRead(LED_BUILTIN)==HIGH)?LOW:HIGH);

#ifdef _PIPE_TO_NULL
	while (TinyWire.available())
		TinyWire.read();
#endif
}




void Display()
{
	ws2812_setleds(led, currentCount);
}



