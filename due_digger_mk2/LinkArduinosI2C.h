//#include "RobotSelector.h"
#include <Wire.h>
#define SLAVE_ADDRESS 9 //address of this board
extern bool manualMode;

void initiateFioSerial(){
	Serial2.begin(19200); // join serial
	return;
}

void fioWrite(int protocolCode){
	// Serial.print(F("fioWrite() method called. Protocol is...")); Serial.println(protocolCode);
	Serial2.println(protocolCode, HEX);
}

void fioWriteInt(int protocolCode){
 Serial2.println(protocolCode);
}

int fioRead(){
	while(1){ //wait for a byte. duct tape solution
		if( Serial2.available() > 0){
			int byte= Serial2.read();
			return byte;
		} 
	}
}

byte masterRead(){
// Wire.requestFrom(SLAVE_ADDRESS, 1);    // request 1 byte from slave device SLAVE_ADDRESS
// byte onebyte = Wire.read(); // receive a byte as character
// return onebyte;
}
//====================================================



