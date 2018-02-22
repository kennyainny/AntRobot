/*
	This class connects a current sensor with via analog pin
        This code is written for a Pololu ACS714 Current Sensor Carrier, -5A to +5A, Pololu item #1185
        
        Product description: This board is a simple carrier of Allegro’s ±5A ACS714 Hall effect-based linear current sensor,
        which offers a low-resistance (~1.2 mΩ) current path and electrical isolation up to 2.1 kV RMS. This version accepts 
        a bidirectional current input with a magnitude up to 5 A and outputs a proportional analog voltage (185 mV/A) 
        centered at 2.5 V with a typical error of ±1.5%. It operates from 4.5 V to 5.5 V and is intended for use in 5 V systems.
        
*/

#ifndef CurrentSensor_h
#define CurrentSensor_h
#include "Arduino.h"
#include "RobotSelector.h"



#define STALL_THRESH_RAW 820   
#define STALL_THRESH 2.0       //stall threshold in amperes

class CurrentSensor
{
	public:
	CurrentSensor();  	   			//class constructor
	void setPin(int currentSensorPin); // this will set up the pin
	int ReadRaw();                				//this will read out a raw value
	int ReadRawAvg(int CURRENT_SAMPLE_SIZE);	//this will read out an average of raw readings. TESTED for 50 samples. Larger number of samples may result in overload
        float Read();                  				//this will read out a value in Amperes
        float ReadAvg(int CURRENT_SAMPLE_SIZE);   	//this will read out an average value in Amperes for a given number of sesnors
	/*
	#ifdef STALL_THRESH;
	bool IsStalled(int CURRENT_SAMPLE_SIZE);  		//checks if the the motors are stalled
	#endif;
	
	#ifdef STALL_THRESH_RAW;
	bool IsStalledRaw(int CURRENT_SAMPLE_SIZE); 
	#endif;
	*/
	private:
	int _analogPin;
        float convertToAmps(int adcReading);


};

#endif
