#include "Arduino.h"
#include "RobotSelector.h"

void arduinoReset(){
  // pinMode(ResetPin, OUTPUT);
  // digitalWrite(ResetPin, LOW);    // Restart arduino by pulling pin LOW 
 pinMode(ResetPin,OUTPUT);  //puts pin in low impeadence mode
 digitalWrite(ResetPin,HIGH); //turns on pullup resistor to turn transistor on to short RESET (which is HIGH) to ground 
}

void pullResetPinHigh(){
 pinMode(ResetPin,OUTPUT);
 digitalWrite(ResetPin,HIGH);
}

void pullResetPinLow(){
 pinMode(ResetPin,INPUT); //puts pin in high impeadence mode 
 digitalWrite(ResetPin,LOW);
}