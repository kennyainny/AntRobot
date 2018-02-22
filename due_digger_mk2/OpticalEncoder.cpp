#include "Arduino.h"
#include "OpticalEncoder.h"

OpticalEncoder :: OpticalEncoder(int encoderOutA, int encoderOutB){
// _interruptA=interruptA;
// _interruptB=interruptB;
_encoderOutA=encoderOutA; //store passed in pin numbers in private variables
_encoderOutB=encoderOutB;

 

//initiate variables to zero
}
void OpticalEncoder :: Initialize(){
pinMode(_encoderOutA,INPUT); //set pins to input mode
pinMode(_encoderOutB,INPUT);
digitalWrite(_encoderOutA,HIGH);// turn pullup resistors on
digitalWrite(_encoderOutB,HIGH);
}

// void OpticalEncoder :: foo(){
 // // attachInterrupt(EncoderPinA1, OpticalEcoder::foo, CHANGE); //and call like this  
// // attachInterrupt(EncoderPinB1, OpticalEncoder::foo, CHANGE);
// bar=20;
// }

