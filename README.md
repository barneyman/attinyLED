# attinyLED
ws2812B I2C driver using attiny85.

The tiny85 is now an I2C slave that can drive a string of ws2812B leds.

There are two builds - Palette and non-Palette.

The latter will let you control ~90 LEDs, and set RGB for each one, while the former allows you to address ~300 of 
them using a pre-defined palette (basically the web pallete) - there are 8 user palettes available.

There's also a rudimentary macro capability, and it will also run the macro on a PCint.

This project uses my fork of TinyWire (to bubble the PCINT up) - the example app ( https://github.com/barneyman/espMaster )
can be used as a template to drive it - there is a library (ATleds) in https://github.com/barneyman/myLibraries that will 
drive the I2C from a master


The project also includes the eagle and gerber files for the board
The board was produced by ALLPCB for US$15 for 5

![Board - Eagle](https://github.com/barneyman/attinyLED/blob/master/board.png)

![Wemos driving board driving 15 leds](https://github.com/barneyman/attinyLED/blob/master/wemos-led.jpg)
