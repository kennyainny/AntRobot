#include "Arduino.h"
#include "CurrentSensor.h"
#include "RobotSelector.h"

CurrentSensor :: CurrentSensor(){}


void CurrentSensor :: setPin(int analogPin)
{
	_analogPin=analogPin;
}


int CurrentSensor :: ReadRaw()
{
  int _rawValue=analogRead(_analogPin);
  return _rawValue;
}

int CurrentSensor :: ReadRawAvg(int CURRENT_SAMPLE_SIZE)
{
 long _rawAvg = 0;
 int currentVals[CURRENT_SAMPLE_SIZE]; //create an array to store current value readings
 int i; //create two dumb varials to run loops
 for(i=0; i<CURRENT_SAMPLE_SIZE; i++){
  float _rawValue=ReadRaw();
  _rawAvg=_rawAvg +_rawValue; //summ up all readings
  }
  _rawAvg=_rawAvg/CURRENT_SAMPLE_SIZE; //divide the sum by a sample size to compute average
  return _rawAvg; //maybe make it a float?
}

float CurrentSensor :: Read()
{
 int _rawValue=ReadRaw();
 float val=convertToAmps(_rawValue);
  /*
 sensor is center at 2.5V. Analog pin has 10bit resolution and goes from 0 to 5v.
 This means that analog pin can read out a number from 0 to 1023 (note 2^10=2024)
 Hence 2.5V corresponds to about 512 reading
 Sensor has a 185mV/A resolutuin with a typical error +-1.5%. Hence the conversion formula is:
 */
 //---FOR a 5V   Arduino system
 //_sensorValue= (_rawValue - 512 )*2.5*1000 / (512*185); //Note, this can and will overflow if type int is used. 512*185=94720. However, 2^16=65536. Overfloat will occur and so it will think 512*185=94720-65536=29184
 //_sensorValue= (_rawValue - 512 )*0.026393581;  //0.026393581=2.5*1000 / 512 / 185; //im removing redundand calculation to guard against an overflow
 //---FOR a 3.3V Arduino system
 //_sensorValue= (_rawValue - 775 )*2.5*1000 / (512*185); //Note, this can and will overflow if type int is used. 512*185=94720. However, 2^16=65536. Overfloat will occur and so it will think 512*185=94720-65536=29184
 //_sensorValue=( _rawValue - 775)*0.026393581;
 return val;  
}

float CurrentSensor :: ReadAvg(int CURRENT_SAMPLE_SIZE) //this function needs to be checked.
{
 int _rawAvg=ReadRawAvg(CURRENT_SAMPLE_SIZE); 
 
 float avg=convertToAmps(_rawAvg);
 
// Serial.println(_rawAvg);
// //_sensorAvg=(_rawAvg - 775 ) *0.026393581; //note that this is for a 3.3v system
// float Vadc=3.3/1024*_rawAvg;
// Serial.println(Vadc);
//
// float dv=Vadc-2.5; //substract centered voltage
// Serial.print(dv);
// _sensorValue=dv/185; //returns value Amps in mA
//  Serial.print(dv);

 return avg;  
}
/*
bool CurrentSensor :: IsStalled( int CURRENT_SAMPLE_SIZE )
{
  _sensorAvg=ReadAvg( CURRENT_SAMPLE_SIZE );
 if ( _sensorAvg >= STALL_THRESH ){
	return 1;
	} else {
	return 0;
 }

}

bool CurrentSensor :: IsStalledRaw(int CURRENT_SAMPLE_SIZE)
{
 _rawValue=ReadRaw();
 if ( _rawValue >= STALL_THRESH_RAW){
 return 1;
 } else {
 return 0;
 }
}
*/
float CurrentSensor :: convertToAmps(int adcReading){
 
 float Vadc=3.3/1024*(float(adcReading));
 float dv= Vadc-2.5; //subtract centered voltage
 float Amps=dv/185*1000; //returns value Amps in mA
 
 return Amps;
  
}
