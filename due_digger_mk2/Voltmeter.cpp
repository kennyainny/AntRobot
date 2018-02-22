#include "Arduino.h"  //includes standard Arduino Library
#include "Voltmeter.h" //includes header file
#include "RobotSelector.h"
#define R1 3.3 //2.68 //3.3 //k ohms, actual resistance 
//#define R2 1.65 //k omhs ,two 3.3k resistors in parallel
#define R2 3.3 //2.65 //3.3 //k omhs ,two 3.3k resistors in parallel

Voltmeter :: Voltmeter(){}

void Voltmeter :: setPin(int analogPin){
 _analogPin=analogPin;
  pinMode(_analogPin,OUTPUT); //fixed this bug
  //pinMode(_analogPin,INPUT); //set pin mode
  // digitalWrite(_analogPin,HIGH);  //turns on 20k~50k pullup resistor on a microcontroller
}

float Voltmeter :: Read(){
int reading = analogRead(_analogPin);
float volt = ( (float)reading/1023)* Vcc*(R2+R1)/R2;// + 0.2; //pin reads 0 if there is 0V on a pin and 1023 when the voltage is VCC
//volt=volt * R2/(R1+R2); //voltage divider adjustment
return volt;
} 

float Voltmeter :: GrabMin(int Samples){
// _Samples=Samples; //unnecessary; reserved for further development 
float Measurements[Samples];

 for (int i=0; i < Samples; i++){
 Measurements[i]=Read(); 
 //Serial.println( Measurements[i] ); //debug
 delay(2); // 
 } //this loop is really not needed, can be removed and Read moved in next loop. saving values for averages and stuff
 
 _SampleMin=Measurements[0];
 for (int j=1; j < Samples; j++){
  if(Measurements[j] < _SampleMin){
  _SampleMin=Measurements[j];
  }
 }
return _SampleMin;
} 

float Voltmeter :: GrabAvg(int Samples){
// _Samples=Samples; //unnecessary; reserved for further development 
	float Measurements[Samples];
	float sum=0;
	for (int i=0; i < Samples; i++){
		Measurements[i]=Read(); 
		sum+=Measurements[i];
		delay(10); // 
	} //this loop is really not needed, can be removed and Read moved in next loop. saving values for averages and stuff
 
	return sum/Samples;
}   
