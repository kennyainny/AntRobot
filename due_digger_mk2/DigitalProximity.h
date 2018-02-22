/** This class will interface QRE1113 digital line sensor
which will be used as a proximity sensor */

#ifndef DigitalProximity_h
#define DigitalProximity_h
#include "Arduino.h"


class DigitalProximity{
 public:
 DigitalProximity(int digitalPin);
 int Read();
 bool isDetected();
 int detectionThresh=2990;
 
 private:
 int _digitalPin; 
};

#endif