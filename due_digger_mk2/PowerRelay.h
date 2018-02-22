/**
	This class sets up a relay to turn on and off so that
	the power to motors can be cut when charging


*/
#ifndef PowerRelay_h
#define PowerRelay_h
#include "Arduino.h"
#include "RobotSelector.h"


class PowerRelay{
 public:
 PowerRelay();
 void PowerOn();  //turns relay on
 void PowerOff(); //turns relay off
// bool State();    //true if relay is on, false if its off
 // private:


 //bool _state;

};


#endif