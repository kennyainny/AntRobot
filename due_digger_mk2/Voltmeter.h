/**
	This class enables an analog pin to read voltage of a power source
	Arduino Fio v2 or Arduino Due
*/

#ifndef Voltmeter_h
#define Voltmeter_h
#include "Arduino.h"
#include "RobotSelector.h"

class Voltmeter{
 public:
	 Voltmeter();
	 void setPin(int analogPin);

	 float Read();
	 float GrabMin(int Samples=20);
	 float GrabAvg(int Samples=20);
	 private:
	 const float Vcc=3.3; //3.3; //ADC pin voltage
	 int _analogPin;
	 int _Samples;
	 float _SampleMin=0; //initiate 
};


#endif
