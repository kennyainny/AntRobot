#include "Arduino.h"
#include "GripperSensor.h"
#include "RobotSelector.h"
#include <Wire.h>

GripperSensor :: GripperSensor(int sensorPin){
//GripperSensor :: GripperSensor(){}
_sensorPin=sensorPin; //save assigned pin to a private variable for robustness
	// // pinMode(_sensorPin,OUTPUT);
// #if ANALOG_TYPE
pinMode(_sensorPin,INPUT); //turns internal resistor off
// #endif
}

bool GripperSensor :: IsDetected(){
int _Reading = Read();
#if DIGITAL_TYPE

//Serial.println(_Reading); //debugging line

if ( _Reading <= GRIPPER_SENSOR_THRESH_DIGITAL){
return 1;
} else {
return 0;
}
#endif


#if ANALOG_TYPE
if (_Reading <= threshold && _Reading > 1){//>1 protects against 0 reading which may be caused by a lose wire 
return 1;
} else {
return 0;
}
#endif
}



int GripperSensor :: Read(){
#if DIGITAL_TYPE
  //Returns value from the digital line sensor 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode( GripperSensorPin, OUTPUT );
  digitalWrite( GripperSensorPin, HIGH );  
  delayMicroseconds(10);
  pinMode( GripperSensorPin, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(GripperSensorPin) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
#endif

#if ANALOG_TYPE
 int _Reading=analogRead(_sensorPin);
 return _Reading;
#endif  
}
 
