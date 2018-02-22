/* voltage divider circuit with 3.9k resistors  */
#include "Arduino.h"
#include "RobotSelector.h"

extern bool goingCharging;
extern void ChargingMode();

void initiateChargingDetector()
{
	// pinMode(31, INPUT); //set pin as input
	pinMode(33, INPUT); //set pin as input
	// pinMode(35, INPUT); //set pin as input 

	// digitalWrite(31,LOW);
	// digitalWrite(33,LOW);
	// digitalWrite(35,LOW);

	// digitalWrite(ChargingDetectorPin,LOW); //turn on pullup resistor 
}

bool isChargerDetected(){
	//returns 1 if voltage is detected on the pin, 0 if not
	// bool val=digitalRead(ChargingDetectorPin);
	return (digitalRead(ChargingDetectorPin) == HIGH);
}
