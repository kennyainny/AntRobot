/*MAG3110.cpp
MAG3110 Breakout Example Code
  
  by: Aaron Weiss, aaron at sparkfun dot com
      SparkFun Electronics 2011
  date: 9/6/11
  license: beerware, if you use this code and happen to meet me, you
           can by me a beer

  The code reads the raw 16-bit x, y, and z values and prints them 
  out. This sketch does not use the INT1 pin, nor does it poll for
  new data.
*/

/* JSP Comment:
Modified from MAG3110 Breakout Example Code
This code ignores z-value, as they tend to stick to different values after
it has gone beyond the detectable threshold magnetic field
Only uses x-value and y-value
Also uses the x-calib and y-calib (values given by the sensor when there is no magnetic field applied)
to calibrate the sensor values
File Created on 05/09/16
Modified Last on 05/09/16
*/

#ifndef MAG3110_h
#define MAG3110_h

#include <Arduino.h> 
#include <Wire.h>

#define MAG_ADDR  0x0E //7-bit address for the MAG3110, doesn't change

#define x_calib	64580
#define y_calib	30 //3000
#define field_threshold	5000// 7000 - 3000 avoid false -ves: kehinde

class MAG3110
{
	public:
		MAG3110();
		void setup();
		void config();
		int measure_values();
		bool Detected();
		int readx();
		int ready();
		int readz();
};
#endif
