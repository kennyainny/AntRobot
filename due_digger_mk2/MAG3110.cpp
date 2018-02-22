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

#include <Arduino.h> 
#include "MAG3110.h"

MAG3110::MAG3110()
{

}

void MAG3110::setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  config();            // turn the MAG3110 on
}

void MAG3110::config()
{
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x11);              // cntrl register2
  Wire.write(0x80);              // send 0x80, enable auto resets
  Wire.endTransmission();       // stop transmitting
  
  delay(15);
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x10);              // cntrl register1
  Wire.write(1);                 // send 0x01, active mode
  Wire.endTransmission();       // stop transmitting
}

int MAG3110::measure_values()
{
  int x = readx();
  int y = ready();
  int z = readz();
  // Serial.print("x=");
  // Serial.print(x); 
  // Serial.print(",");  
  // Serial.print("y=");    
  // Serial.print(y);
  // Serial.print(",");       
  // Serial.print("z=");    
  // Serial.println(z);
  // Serial.print("    ");
  int field_measurement = sqrt(pow(x - x_calib, 2)); //+ pow(y - y_calib, 2));
  // Serial.print("field_measurement=");    

	Serial.println(field_measurement);
	
	return field_measurement;
}

bool MAG3110::Detected()
{
	if (measure_values() >= field_threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int MAG3110::readx()
{
  int xl, xh;  //define the MSB and LSB
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x01);              // x MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(10); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    xh = Wire.read(); // receive the byte
  }
  
  delayMicroseconds(10); //needs at least 1.3us free time between start and stop
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x02);              // x LSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(10); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may write less than requested
  { 
    xl = Wire.read(); // receive the byte
  }
  
  int xout = (xl|(xh << 8)); //concatenate the MSB and LSB
  return xout;
}

int MAG3110::ready()
{
  int yl, yh;  //define the MSB and LSB
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x03);              // y MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(10); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may write less than requested
  { 
    yh = Wire.read(); // receive the byte
  }
  
  delayMicroseconds(10); //needs at least 1.3us free time between start and stop
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x04);              // y LSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(10); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    yl = Wire.read(); // receive the byte
  }
  
  int yout = (yl|(yh << 8)); //concatenate the MSB and LSB
  return yout;
}

int MAG3110::readz()
{
  int zl, zh;  //define the MSB and LSB
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x05);              // z MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(10); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    zh = Wire.read(); // receive the byte
  }
  
  delayMicroseconds(10); //needs at least 1.3us free time between start and stop
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x06);              // z LSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(10); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    zl = Wire.read(); // receive the byte
  }
  
  int zout = (zl|(zh << 8)); //concatenate the MSB and LSB
  return zout;
}