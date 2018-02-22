/*
 serLCD.cpp - Library for controlling a SparkFun serLCD
              module.
	      Code written for firmware version 2.5

 Created by Cody B. Null, September 9, 2011
 Modified by Jordan Hochenbaum, Dec 5, 2011. (Version 1.5)

 Version 1.4 - This version wrote specifically for 16x2
               Display.
               More display support coming in later version.

 Version 1.5 - Since mid 2011 the Arduino IDE has implemented the NewSoftSerial 
                library updates in the standard SoftwareSerial library and so 
                using NewSoftSerial is no longer needed. serLCD has been updated 
                to use the Standard SoftwareSerial library. Also, Arduino 1.0
                support has been added. Backwards compatible with previous versions.

 
 Note -	This library requires NewSoftSerial library
 The latest version of NewSoftSerial library can 
 always be found at http://arduiniana.org. -> NO LONGER NECESSARY. See V1.5 notes above
*/

//#include <../NewSoftSerial/NewSoftSerial.h>

// #include <SoftwareSerial.h>
#include "hardSerLCD.h"
#include "Arduino.h"

//	PUBLIC FUNCTIONS

// Contstructor
// defaults to 16x2 display
hardSerLCD::hardSerLCD(){
	 _Serial = NULL;



	// pinMode(pin, OUTPUT);
	// begin(9600);
	// _numlines = LCD_2LINE;
	// _numchars = LCD_16CHAR;
	// _rowoffset = 0;
}

void hardSerLCD::begin(HardwareSerial *serIn, int baudRate)
{
 _Serial = serIn;
 _Serial->begin(baudRate);
 	command(LCD_CLEARDISPLAY);

 _Serial->print("Ready to Rip!");
 delay(1000);
 
 	_numlines = LCD_2LINE;
	_numchars = LCD_16CHAR;
	_rowoffset = 0;
 
 
}

void hardSerLCD::print(String input)
{

 _Serial->print(input);
 
}

// void hardSerLCD::printFresh(String input)
// {
 	// command(LCD_CLEARDISPLAY);
	// home();
	// _Serial->print(input);
 
// }




/* Initialize.. not used trying to implement all display sizes
void serLCD::init(int pin, int rows, int cols){
	pinMode(pin, OUTPUT);
	delay(4);
	begin(9600);
	if(cols == LCD_20CHAR){
		_numchars = LCD_20CHAR;
		specialCommand(LCD_SET20CHAR);
	}else{ // default to 16 char display
		_numchars = LCD_16CHAR;
		specialCommand(LCD_SET16CHAR);
	}	
	if(rows == LCD_4LINE){
		_rowoffset = 1;
		_numlines = LCD_4LINE;
		specialCommand(LCD_SET4LINE);
	}else{ // default to 2 line if input was invalid 
		_rowoffset = 0;
		_numlines = LCD_2LINE;
		specialCommand(LCD_SET2LINE);
	}
	// clear the display
	clear();
	// set brightness to full
	setBrightness(30);
}
*/

// Set brightness value range 1-30 1=OFF 30=FULL
void hardSerLCD::setBrightness(int val){
	if(val >= 1 && val <= 30){
		specialCommand(LCD_BACKLIGHT | (val - 1));
	}
}

// Clears screen and returns cursor to home position
void hardSerLCD::clear(){
	command(LCD_CLEARDISPLAY);
}

// Clears a single line by writing blank spaces then returning
// cursor to beginning of line
void hardSerLCD::clearLine(int num){
	if(num > 0 && num <= _numlines){
		setCursor(num, 1);
		_Serial->print("                ");
		setCursor(num, 1);
	}
}

// Moves cursor to the beginning of selected line
void hardSerLCD::selectLine(int num){
	if(num > 0 && num <= _numlines){
		setCursor(num, 1);
	}
}

// returns cursor to home position
void hardSerLCD::home(){
	command(LCD_RETURNHOME);
}

// Saves first 2 lines of txt to splash screen memory
void hardSerLCD::setSplash(){
	specialCommand(LCD_SETSPLASHSCREEN);
}

// Toggles splashscreen on and off
void hardSerLCD::toggleSplash(){
	specialCommand(LCD_SPLASHTOGGLE);
}

//  This is for text that flows Left to Right
void hardSerLCD::leftToRight(){
	_displaymode |= LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void hardSerLCD::rightToLeft() {
	_displaymode &= ~LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// Blinking cursor on/off
void hardSerLCD::blink(){
	_displaycontrol |= LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void hardSerLCD::noBlink(){
	_displaycontrol &= ~LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Underline cursor on/off
void hardSerLCD::cursor(){
	_displaycontrol |= LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void hardSerLCD::noCursor(){
	_displaycontrol &= ~LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Display on/off
void hardSerLCD::display(){
	_displaycontrol |= LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void hardSerLCD::noDisplay(){
	_displaycontrol &= ~LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Set cursor to specific row and col values start at 1 not 0
void hardSerLCD::setCursor(int row, int col){
	int row_offsets[2][4] = {
		{ 0x00, 0x40, 0x10, 0x50 },
		{ 0x00, 0x40, 0x14, 0x54 }
	};
	if((row > 0 && row < 3) && (col > 0 && col < 17)){
           command(LCD_SETDDRAMADDR | ((col - 1) + row_offsets[_rowoffset][(row - 1)]));
	}
}

// Creates custom characters 8 char limit
// Input values start with 1
void hardSerLCD::createChar(int location, uint8_t charmap[]){
	location -= 1;
	location &= 0x07;
  for (int i=0; i<8; i++){
    command(LCD_SETCGRAMADDR | (location << 3) | i);
    _Serial->write(charmap[i]);
  }
}

// Prints custom character
// Input values start with 1
void hardSerLCD::printCustomChar(int num){
	_Serial->write((num - 1));
}

// PRIVATE FUNCTIONS

// Functions for sending the special command values
void hardSerLCD::command(uint8_t value){
	_Serial->write(0xFE);
	_Serial->write(value);
	delay(5);
}
void hardSerLCD::specialCommand(uint8_t value){
	_Serial->write(0x7C);
	_Serial->write(value);
	delay(5);
}