#include "Arduino.h" //includes standard Arduino library
#include "DigitalProximity.h" //include header file

DigitalProximity :: DigitalProximity(int digitalPin){
_digitalPin=digitalPin;
}

int DigitalProximity :: Read() {
  /* this section of code was adapted from Sparkfun's sample code, shared under MIT Licence */
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode( _digitalPin, OUTPUT );
  digitalWrite( _digitalPin, HIGH );  
  delayMicroseconds(10);
  pinMode( _digitalPin, INPUT );

  long time = micros();
  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(_digitalPin) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;
  return diff;

}

bool DigitalProximity :: isDetected(){
	int val=Read();
	if (val < detectionThresh){
		return true;
	}
	else{
		return false;
	}
}