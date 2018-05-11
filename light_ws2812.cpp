/*
* light weight WS2812 lib V2.0b
*
* Controls WS2811/WS2812/WS2812B RGB-LEDs
* Author: Tim (cpldcpu@gmail.com)
*
* Jan 18th, 2014  v2.0b Initial Version
* Nov 29th, 2015  v2.3  Added SK6812RGBW support
*
* License: GNU GPL v2+ (see License.txt)

https://cpldcpu.wordpress.com/2014/01/14/light_ws2812-library-v2-0-part-i-understanding-the-ws2812/
*/

#include "light_ws2812.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>


// Setleds for standard RGB 
void /*inline*/ ws2812_setleds(struct cRGB *ledarray, uint16_t leds)
{
   ws2812_setleds_pin(ledarray,leds, _BV(ws2812_pin));
}

void /*inline*/ ws2812_setleds_pin(struct cRGB *ledarray, uint16_t leds, uint8_t pinmask)
{
  ws2812_sendarray_mask((uint8_t*)ledarray,leds+leds+leds,pinmask);
  _delay_us(ws2812_resettime);
}

// Setleds for SK6812RGBW
void /*inline*/ ws2812_setleds_rgbw(struct cRGBW *ledarray, uint16_t leds)
{
  ws2812_sendarray_mask((uint8_t*)ledarray,leds<<2,_BV(ws2812_pin));
  _delay_us(ws2812_resettime);
}

void ws2812_sendarray(uint8_t *data,uint16_t datlen)
{
  ws2812_sendarray_mask(data,datlen,_BV(ws2812_pin));
}

/*
  This routine writes an array of bytes with RGB values to the Dataout pin
  using the fast 800kHz clockless WS2811/2812 protocol.
*/

// Timing in ns
#define w_zeropulse   350
#define w_onepulse    900
#define w_totalperiod 1250

// Fixed cycles used by the inner loop
#define w_fixedlow    2
#define w_fixedhigh   4
#define w_fixedtotal  8   

// Insert NOPs to match the timing, if possible
#define w_zerocycles    (((F_CPU/1000)*w_zeropulse          )/1000000)
#define w_onecycles     (((F_CPU/1000)*w_onepulse    +500000)/1000000)
#define w_totalcycles   (((F_CPU/1000)*w_totalperiod +500000)/1000000)

// w1 - nops between rising edge and falling edge - low
#define w1 (w_zerocycles-w_fixedlow)
// w2   nops between fe low and fe high
#define w2 (w_onecycles-w_fixedhigh-w1)
// w3   nops to complete loop
#define w3 (w_totalcycles-w_fixedtotal-w1-w2)

#if w1>0
  #define w1_nops w1
#else
  #define w1_nops  0
#endif

// The only critical timing parameter is the minimum pulse length of the "0"
// Warn or throw error if this timing can not be met with current F_CPU settings.
#define w_lowtime ((w1_nops+w_fixedlow)*1000000)/(F_CPU/1000)
#if w_lowtime>550
   #error "Light_ws2812: Sorry, the clock speed is too low. Did you set F_CPU correctly?"
#elif w_lowtime>450
   #warning "Light_ws2812: The timing is critical and may only work on WS2812B, not on WS2812(S)."
   #warning "Please consider a higher clockspeed, if possible"
#endif   

#if w2>0
#define w2_nops w2
#else
#define w2_nops  0
#endif

#if w3>0
#define w3_nops w3
#else
#define w3_nops  0
#endif

#define w_nop1  "nop      \n\t"
#define w_nop2  "rjmp .+0 \n\t"
#define w_nop4  w_nop2 w_nop2
#define w_nop8  w_nop4 w_nop4
#define w_nop16 w_nop8 w_nop8


void inline ws2812_sendarray_mask(uint8_t *data,uint16_t datlen,uint8_t maskhi)
{
  uint8_t curbyte,ctr,masklo;

  uint8_t sreg_prev;

  ws2812_DDRREG |= maskhi; // Enable output
  
  masklo	=~maskhi&ws2812_PORTREG;
  maskhi |=        ws2812_PORTREG;
  
  sreg_prev = SREG;
  cli();


  while (datlen--) {
    curbyte=*data++;


    asm volatile(
    "       ldi   %0,8  \n\t"	// loadImmediate 8 into ctr
    "loop%=:            \n\t"
    "       out   %2,%3 \n\t"    //  '1' [01] '0' [01] - re // push HI out to port?
#if (w1_nops&1)
w_nop1
#endif
#if (w1_nops&2)
w_nop2
#endif
#if (w1_nops&4)
w_nop4
#endif
#if (w1_nops&8)
w_nop8
#endif
#if (w1_nops&16)
w_nop16
#endif
    "       sbrs  %1,7  \n\t"    //  '1' [03] '0' [02] // skip next instruction if bit7 set in data
    "       out   %2,%4 \n\t"    //  '1' [--] '0' [03] - fe-low // push LOW out of led port ?
    "       lsl   %1    \n\t"    //  '1' [04] '0' [04] // left shift data
#if (w2_nops&1)
  w_nop1
#endif
#if (w2_nops&2)
  w_nop2
#endif
#if (w2_nops&4)
  w_nop4
#endif
#if (w2_nops&8)
  w_nop8
#endif
#if (w2_nops&16)
  w_nop16 
#endif
    "       out   %2,%4 \n\t"    //  '1' [+1] '0' [+1] - fe-high // push LOW out of led port ?
#if (w3_nops&1)
w_nop1
#endif
#if (w3_nops&2)
w_nop2
#endif
#if (w3_nops&4)
w_nop4
#endif
#if (w3_nops&8)
w_nop8
#endif
#if (w3_nops&16)
w_nop16
#endif

    "       dec   %0    \n\t"    //  '1' [+2] '0' [+2]	// decrement counter
    "       brne  loop%=\n\t"    //  '1' [+3] '0' [+4]	// branch NEQ to loop
    :	"=&d" (ctr)
    :	"r" (curbyte), "I" (_SFR_IO_ADDR(ws2812_PORTREG)), "r" (maskhi), "r" (masklo)
    );

  }
  
  SREG=sreg_prev;
}


#include <avr/pgmspace.h>

// passed as RGBW *only* to make the ASM math simpler (ie *4 == <<2 instead of *3)
void ws2812_sendarray_mask_palette(const struct cRGBW *userPaletteArray, bool paletteInProgmem, const struct cRGBW *paletteArray, uint8_t *data, uint16_t datlen,unsigned char paletteDivLocal, uint8_t maskhi)
{
	uint8_t curbyte, ctr, masklo;

	uint8_t sreg_prev;

	ws2812_DDRREG |= maskhi; // Enable output

	masklo = ~maskhi&ws2812_PORTREG;
	maskhi |= ws2812_PORTREG;

	sreg_prev = SREG;
	cli();

	const uint8_t *componentDataAddress;
	bool paletteInProgmemConfirmed;

	while (datlen--) 
	{
		// the "check which palette" logic pushed it over the edge (timewise) at 8mhz,
		// increasing t0 16mhz made it happier

		// check it's a user colour
		if (*data & 0x10)
		{
			componentDataAddress = (uint8_t*)&userPaletteArray[*data++];
			// can't be!
			paletteInProgmemConfirmed = false;
		}
		else
		{
			componentDataAddress = (uint8_t*)&paletteArray[*data++];
			// honour what was pasted
			paletteInProgmemConfirmed = paletteInProgmem;
		}

		for (uint8_t component = 0; component < 3; component++)
		{
			if(paletteInProgmemConfirmed)
			{
				curbyte = pgm_read_byte(componentDataAddress++);

				asm volatile(
					// there is some strange voodoo magic with which registers actually honour Z flag tests ?
					// life was much simpler in 6502 :)

					"		mov r0, %2 \n\t"		// move div into r0 (temp register) - marked as clobbered (1 cyc)
					"		tst r0 \n\t"			// test it (1 cyc)
					"		breq divDone%= \n\t"	// if it's 0 jump past div (1 cyc for failed, 2 for hit)
					"divLoop%=: \n\t"
					"		lsr %0 \n\t"			// div the byte by 2 (1 cyc)
					"		dec r0 \n\t"			// dec (1 cyc)
					"		brne divLoop%= \n\t"	// finished? (1 cyc for failed, 2 for hit)

													// div = 0 : 4 cycles
													// div = x : 3 + ( 4(x-1) + 3)  [ 6 : 30 ] 30cycles is approx 4usec at 8mhz, well below the 10usec reset time

					"divDone%=: \n\t"
					: "=r" (curbyte)
					: "0" (curbyte), "r" (paletteDivLocal)
					: "r0"

					);


			}
			else
			{
				curbyte = *componentDataAddress++;
			}




			asm volatile(

				"       ldi   %0,8  \n\t"	// loadImmediate 8 into ctr
				"loop%=:            \n\t"
				"       out   %2,%3 \n\t"    //  '1' [01] '0' [01] - re // push HI out to port?
#if (w1_nops&1)
				w_nop1
#endif
#if (w1_nops&2)
				w_nop2
#endif
#if (w1_nops&4)
				w_nop4
#endif
#if (w1_nops&8)
				w_nop8
#endif
#if (w1_nops&16)
				w_nop16
#endif
				"       sbrs  %1,7  \n\t"    //  '1' [03] '0' [02] // skip next instruction if bit7 set in data
				"       out   %2,%4 \n\t"    //  '1' [--] '0' [03] - fe-low // push LOW out of led port ?
				"       lsl   %1    \n\t"    //  '1' [04] '0' [04] // left shift data
#if (w2_nops&1)
				w_nop1
#endif
#if (w2_nops&2)
				w_nop2
#endif
#if (w2_nops&4)
				w_nop4
#endif
#if (w2_nops&8)
				w_nop8
#endif
#if (w2_nops&16)
				w_nop16
#endif
				"       out   %2,%4 \n\t"    //  '1' [+1] '0' [+1] - fe-high // push LOW out of led port ?
#if (w3_nops&1)
				w_nop1
#endif
#if (w3_nops&2)
				w_nop2
#endif
#if (w3_nops&4)
				w_nop4
#endif
#if (w3_nops&8)
				w_nop8
#endif
#if (w3_nops&16)
				w_nop16
#endif

				"       dec   %0    \n\t"    //  '1' [+2] '0' [+2]	// decrement counter
				"       brne  loop%=\n\t"    //  '1' [+3] '0' [+4]	// branch NEQ to loop
				:	"=&d" (ctr)
				: "r" (curbyte), "I" (_SFR_IO_ADDR(ws2812_PORTREG)), "r" (maskhi), "r" (masklo)
				);
		}
	}

	SREG = sreg_prev;
}
