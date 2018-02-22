/**

Modified by : Ross Warkentin
email: ross.warkentin@gmail.com

**/

#include "Arduino.h"
#include "RobotSelector.h"
#include "CapacitiveSensor.h"
#include <Wire.h>
#include "mpr121.h"

extern int switchState;

extern void FollowLane(); // used in the delay of getSwitchState()
extern void FollowLaneBackward(); // used in the delay of getSwitchState()

extern bool goingIn;
extern bool goingOut;
extern bool exitTunnelMode;

extern bool turnReversalMode;


CapacitiveSensor::CapacitiveSensor(int CapSensorPin){
	irqpin = CapSensorPin;
}

void CapacitiveSensor:: setup(){
  pinMode(irqpin, INPUT);
  digitalWrite(irqpin, HIGH); //enable pullup resistor
  
  Serial.begin(9600);
  Wire.begin();

  mpr121_setup();
}

void CapacitiveSensor:: readSensorSignal(){

  Wire.requestFrom(0x5A,20); 

  LSB_CAP = Wire.read();//touch status
  MSB_CAP = Wire.read();
  aa1 = Wire.read();//OOR Status
  aa2 = Wire.read();
  pin0L = Wire.read();//electrode 0
  pin0M = Wire.read();
  pin1L = Wire.read();//electrode 1
  pin1M = Wire.read();
  pin2L = Wire.read();//electrode 2
  pin2M = Wire.read(); 
  pin3L = Wire.read();//electrode 3
  pin3M = Wire.read(); 
  pin4L = Wire.read();//electrode 4
  pin4M = Wire.read(); 
  pin5L = Wire.read();//electrode 5
  pin5M = Wire.read();
  pin6L = Wire.read();//electrode 6
  pin6M = Wire.read();
  pin7L = Wire.read();//electrode 7
  pin7M = Wire.read();

}

int CapacitiveSensor:: readTouch(){
	readSensorSignal();
	return LSB_CAP;
}



void CapacitiveSensor:: mpr121_setup(void){

  set_register(0x5A, ELE_CFG, 0x00); 
  
  // Section A - Controls filtering when data is > baseline.
  set_register(0x5A, MHD_R, 0x01); //0x01 default
  set_register(0x5A, NHD_R, 0x01);
  set_register(0x5A, NCL_R, 0xFF); //0x00 default
  set_register(0x5A, FDL_R, 0x00);

  // Section B - Controls filtering when data is < baseline.
  set_register(0x5A, MHD_F, 0x01);
  set_register(0x5A, NHD_F, 0x01);
  set_register(0x5A, NCL_F, 0xFF);
  set_register(0x5A, FDL_F, 0x02);
  
  // Section C - Sets touch and release thresholds for each electrode
  set_register(0x5A, ELE0_T, TOU_THRESH);
  set_register(0x5A, ELE0_R, REL_THRESH);
 
  set_register(0x5A, ELE1_T, TOU_THRESH);
  set_register(0x5A, ELE1_R, REL_THRESH);
  
  set_register(0x5A, ELE2_T, TOU_THRESH);
  set_register(0x5A, ELE2_R, REL_THRESH);
  
  set_register(0x5A, ELE3_T, TOU_THRESH);
  set_register(0x5A, ELE3_R, REL_THRESH);
  
  set_register(0x5A, ELE4_T, TOU_THRESH);
  set_register(0x5A, ELE4_R, REL_THRESH);
  
  set_register(0x5A, ELE5_T, TOU_THRESH);
  set_register(0x5A, ELE5_R, REL_THRESH);
  
  set_register(0x5A, ELE6_T, TOU_THRESH);
  set_register(0x5A, ELE6_R, REL_THRESH);
  
  set_register(0x5A, ELE7_T, TOU_THRESH);
  set_register(0x5A, ELE7_R, REL_THRESH);
  
  set_register(0x5A, ELE8_T, TOU_THRESH);
  set_register(0x5A, ELE8_R, REL_THRESH);
  
  set_register(0x5A, ELE9_T, TOU_THRESH);
  set_register(0x5A, ELE9_R, REL_THRESH);
  
  set_register(0x5A, ELE10_T, TOU_THRESH);
  set_register(0x5A, ELE10_R, REL_THRESH);
  
  set_register(0x5A, ELE11_T, TOU_THRESH);
  set_register(0x5A, ELE11_R, REL_THRESH);
  
  // Section D
  // Set the Filter Configuration
  // Set ESI2
  
  //charge time
	set_register(0x5A, 0x5D, ChargeTime); //default setting
  //charge current
  set_register(0x5A, 0x5C, ChargeCurrent); //desired setting
  
  // Section E
  // Electrode Configuration
  // Set ELE_CFG to 0x00 to return to standby mode
  set_register(0x5A, ELE_CFG, 0x0C);  // Enables all 12 Electrodes
  
  
  // Section F
  // Enable Auto Config and auto Reconfig
  /*set_register(0x5A, ATO_CFG0, 0x0B);
  set_register(0x5A, ATO_CFGU, 0xC9);  // USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   set_register(0x5A, ATO_CFGL, 0x82);  // LSL = 0.65*USL = 0x82 @3.3V
  set_register(0x5A, ATO_CFGT, 0xB5);*/  // Target = 0.9*USL = 0xB5 @3.3V
  
  set_register(0x5A, ELE_CFG, 0x0C);
  
}



void CapacitiveSensor:: set_register(int address, unsigned char r, unsigned char v){
    Wire.beginTransmission(address);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
}

int CapacitiveSensor:: getOneContact(int pinNum){
//this method will return a sensor value for one pin; for test and calibration
set_register(0x5A, 0x5C, 0x00); //disables electrode charging on all pins
set_register(0x5A, 0x5F, ChargeCurrent); //enable only one pin for charging/discharging; enables pin0	

readSensorSignal();

// Ross - I am not sure I get what is going on here. Is there a sort of bias term that we are accounting for?
int pin0 = pin0L + 256*pin0M;
int pin1 = pin1L + 256*pin1M;
int pin2 = pin2L + 256*pin2M;
int pin3 = pin3L + 256*pin3M;
int pin4 = pin4L + 256*pin4M;
int pin5 = pin5L + 256*pin5M;
int pin6 = pin6L + 256*pin6M;
int pin7 = pin7L + 256*pin7M;

//Serial.print("pin0L: ");
//Serial.print(pin0L);
//Serial.print("  pin0M: ");
//Serial.print(pin0M);
//Serial.print("  pin0:  ");
//Serial.println(pin0L + 256*pin0M);

set_register(0x5A, 0x5C, ChargeCurrent); //disables electrode charging on all pins
switch (pinNum){
case 0:
return pin0;
break;
case 1:
return pin1;
break;
case 2:
return pin2;
break;
case 3:
return pin3;
break;
case 4:
return pin4;
break;
case 5:
return pin5;
break;
case 6:
return pin6;
break;
case 7:
return pin7;
break;
}

}


int CapacitiveSensor:: getDetectedContacts(){
	//this method will return a hex number representing which switches are turned on or off using polling method
	readSensorSignal();
	int pin0 = pin0L + 256*pin0M;
	int pin1 = pin1L + 256*pin1M;
	int pin2 = pin2L + 256*pin2M;
	int pin3 = pin3L + 256*pin3M;
	int pin4 = pin4L + 256*pin4M;
	int pin5 = pin5L + 256*pin5M;
	int pin6 = pin6L + 256*pin6M;
	int pin7 = pin7L + 256*pin7M;

	//bool switchH=digitalRead(SwitchPinH);
	switchState=0x0;

	// SWITCH_A_MASK_WALL 	0b0000000000000001
	// SWITCH_A_MASK_ANT  	0b0000000000000011
	// SWITCH_B_MASK_WALL 	0b0000000000000100
	// SWITCH_B_MASK_ANT  	0b0000000000001100
	// SWITCH_C_MASK_WALL 	0b0000000000010000
	// SWITCH_C_MASK_ANT  	0b0000000000110000
	// SWITCH_D_MASK_WALL 	0b0000000001000000
	// SWITCH_D_MASK_ANT  	0b0000000011000000
	// SWITCH_E_MASK_WALL 	0b0000000100000000
	// SWITCH_E_MASK_ANT  	0b0000001100000000
	// SWITCH_F_MASK_WALL 	0b0000010000000000
	// SWITCH_F_MASK_ANT  	0b0000110000000000
	// SWITCH_G_MASK_WALL 	0b0001000000000000
	// SWITCH_G_MASK_ANT  	0b0011000000000000
	// SWITCH_H_MASK_WALL 	0b0100000000000000
	// SWITCH_H_MASK_ANT  	0b1100000000000000
	// SWITCH_I_MASK      	0b0000000000000000
	// SWITCH_ANT_MASK    	0b1010101010101010
	// SWITCH_WALL_MASK			0b0101010101010101


	// switchState= switchA | switchB | switchC | switchD | switchE | switchF | switchG | switchH ;
	if((pin0 > AntDoubleThresh1 && pin0 < AntDoubleThresh2) || (pin0 > AntSingleThresh1 && pin0 < AntSingleThresh2)){
		switchState=switchState | SWITCH_A_MASK_ANT;  //bitwise OR  operation
	}
	else if ((pin0 > WallDoubleThresh1 && pin0 < WallDoubleThresh2) ||(pin0 > WallSingleThresh1 && pin0 < WallSingleThresh2)){// && pin0 < WallThresh2){
		switchState = switchState | SWITCH_A_MASK_WALL;  //bitwise OR  operation
	}

	if((pin1 > AntDoubleThresh1 && pin1 < AntDoubleThresh2) || (pin1 > AntSingleThresh1 && pin1 < AntSingleThresh2)){
		switchState=switchState | SWITCH_B_MASK_ANT;  //bitwise OR  operation
	}
	else if ((pin1 > WallDoubleThresh1 && pin1 < WallDoubleThresh2) ||(pin1 > WallSingleThresh1 && pin1 < WallSingleThresh2)){// && pin1 < WallThresh2){
		switchState=switchState | SWITCH_B_MASK_WALL;  //bitwise OR  operation
	}

	if((pin2 > AntDoubleThresh1 && pin2 < AntDoubleThresh2) || (pin2 > AntSingleThresh1 && pin2 < AntSingleThresh2)){
		switchState=switchState | SWITCH_C_MASK_ANT;  //bitwise OR  operation
	}
	else if ((pin2 > WallDoubleThresh1 && pin2 < WallDoubleThresh2) ||(pin2 > WallSingleThresh1 && pin2 < WallSingleThresh2)){
		switchState=switchState | SWITCH_C_MASK_WALL;  //bitwise OR  operation
	}

	if((pin3 > AntDoubleThresh1 && pin3 < AntDoubleThresh2) || (pin3 > AntSingleThresh1 && pin3 < AntSingleThresh2)){
		switchState=switchState | SWITCH_D_MASK_ANT;  //bitwise OR  operation
	}
	else if ((pin3 > WallDoubleThresh1 && pin3 < WallDoubleThresh2) ||(pin3 > WallSingleThresh1 && pin3 < WallSingleThresh2)){
		switchState=switchState | SWITCH_D_MASK_WALL;  //bitwise OR  operation
	}

	if((pin4 > AntDoubleThresh1 && pin4 < AntDoubleThresh2) || (pin4 > AntSingleThresh1 && pin4 < AntSingleThresh2)){
		switchState=switchState | SWITCH_E_MASK_ANT;  //bitwise OR  operation
	}
	else if ((pin4 > WallDoubleThresh1 && pin4 < WallDoubleThresh2) ||(pin4 > WallSingleThresh1 && pin4 < WallSingleThresh2)){
		switchState=switchState | SWITCH_E_MASK_WALL;  //bitwise OR  operation
	}

	if((pin5 > AntDoubleThresh1 && pin5 < AntDoubleThresh2) || (pin5 > AntSingleThresh1 && pin5 < AntSingleThresh2)){
		switchState=switchState | SWITCH_F_MASK_ANT;  //bitwise OR  operation
	}
	else if ((pin5 > WallDoubleThresh1 && pin5 < WallDoubleThresh2) ||(pin5 > WallSingleThresh1 && pin5 < WallSingleThresh2)){
		switchState=switchState | SWITCH_F_MASK_WALL;  //bitwise OR  operation
	}

	if((pin6 > AntDoubleThresh1 && pin6 < AntDoubleThresh2) || (pin6 > AntSingleThresh1 && pin6 < AntSingleThresh2)){
		switchState=switchState | SWITCH_G_MASK_ANT;  //bitwise OR  operation
	}
	else if ((pin6 > WallDoubleThresh1 && pin6 < WallDoubleThresh2) ||(pin6 > WallSingleThresh1 && pin6 < WallSingleThresh2)){
		switchState=switchState | SWITCH_G_MASK_WALL;  //bitwise OR  operation
	}

	if((pin7 > AntDoubleThresh1 && pin7 < AntDoubleThresh2) || (pin7 > AntSingleThresh1 && pin7 < AntSingleThresh2)){
		switchState=switchState | SWITCH_H_MASK_ANT;  //bitwise OR  operation
	}
	else if ((pin7 > WallDoubleThresh1 && pin7 < WallDoubleThresh2) ||(pin7 > WallSingleThresh1 && pin7 < WallSingleThresh2)){
		switchState=switchState | SWITCH_H_MASK_WALL;  //bitwise OR  operation
	}
	
	// Serial.println(F("pin0   pin1   pin2   pin3   pin4   pin5   pin6   pin7"));
	// Serial.print(pin0);
	// Serial.print("   ");
	// Serial.print(pin1);
	// Serial.print("   ");
	// Serial.print(pin2);
	// Serial.print("   ");
	// Serial.print(pin3);
	// Serial.print("   ");
	// Serial.print(pin4);
	// Serial.print("   ");
	// Serial.print(pin5);
	// Serial.print("   ");
	// Serial.print(pin6);
	// Serial.print("   ");
	// Serial.println(pin7);
	   
	//set_register(0x5A, 0x5C, ChargeCurrent); //enable electrode charging on all pins again
	return switchState;
}

/**
getSwitchState is a method that essentially filters switch states to ensure that they are persistent before allowing the signals to affect the ant motion
The readings from getDetectedContacts must be the same for checkMeasurementNum*100 ms in order for the return of switchstate to be nonzero
**/
int CapacitiveSensor:: getSwitchState(){
	checkMeasurementNum = 2; //number of check comparison of switchstate.
	unsigned long timeDelay;
	int i = 0; // integer used for iterating
	int last_switchState = getDetectedContacts();
	// Serial.println(last_switchState);
	int new_switchState = 0;
	while (i < checkMeasurementNum){ //repeat measurements to assure that switchstate is correct and not caused by coincident measurements
		if (goingIn || goingOut){
			FollowLane();
		}
		if(exitTunnelMode){
			FollowLaneBackward();
		}
		new_switchState = getDetectedContacts();
		if (new_switchState != last_switchState){
			switchState = 0; //check failed. switchstate is different from last measurement. set switchState to 0
			break;
		}
		last_switchState = new_switchState;
		i++;
		timeDelay = millis();
		
		// There is a delay time that requires a 100ms delay.
		// If we are going in or out, use the delay period to follow the 
		if (goingIn || goingOut){
			Serial.print("goingIn is "); 			Serial.println(goingIn); 
			Serial.print("goingOut is "); 			Serial.println(goingOut); 
			while(millis() - timeDelay < 100){
				FollowLane();
				// Serial.println("1");

			}
		}
		
		else if (exitTunnelMode){
			while(millis() - timeDelay < 100){
				FollowLaneBackward();
				// Serial.println("1");
			}
		}

		// If we are in a different state, ie diggin or dumping...just delay for 100ms
		else{
				delay(100);
		}
		
	}
	return switchState; //check passed. return the switchstate.
}

/**
The value returned from isDetected() is the value taken on by CONTACT in the main .ino file, as declared in the def*.h files

**/
bool CapacitiveSensor::isDetected(){
	if (getSwitchState() == 0){ //(getDetectedContacts() == 0){
		return false;
	}
	else{
		return true;
	}		
}
