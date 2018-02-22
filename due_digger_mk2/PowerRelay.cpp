#include "Arduino.h"
#include "PowerRelay.h"


PowerRelay :: PowerRelay(){
	pinMode(relay_pin1, OUTPUT); //turns on 20k~50k pullup resistor on a microcontroller
	digitalWrite(relay_pin1, HIGH);//turn relay on by default
}

void PowerRelay :: PowerOn(){
	//returnPinsToNormalState();
	digitalWrite(relay_pin1, HIGH);
	delay(5); //debouncing
	// _state=true;
}

void PowerRelay :: PowerOff(){
	digitalWrite(relay_pin1, LOW);
	delay(5);//debouncing
	// _state=false;
}

// bool PowerRelay :: State(){
// return _state;
// }

