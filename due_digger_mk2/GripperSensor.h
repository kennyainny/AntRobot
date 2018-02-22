/** 
	This class interfaces an analog sensor with OUT pin
	on a digital sensors, raw readings decrease as the white stuff appears
	
	1/22/2015. removed pin storage, replaced with macros
*/

#ifndef GripperSensor_h
#define GripperSensor_h
#include "Arduino.h"
#include "RobotSelector.h"
#include <Wire.h>

#define GRIPPER_SENSOR_THRESH_DIGITAL 100//prev. 2500 // sensor threshold is set to 500
#define GRIPPER_SENSOR_THRESH_ANALOG 100 //prev. 940      // sensor threshold is set to 550, 900 gets set off randomly with IMU added. 850 was okay. 870 worked well with small mcMaster cotton balls but not walmart. Nothing in front=1023
#define DIGITAL_TYPE 0   //is gripper digital
#define ANALOG_TYPE 1    //is gripper analog

class GripperSensor{

	public:
	GripperSensor(int sensorPin);
	//GripperSensor(); //JSP
	bool IsDetected();
    int Read(); //move this to private later;
	int threshold=GRIPPER_SENSOR_THRESH_ANALOG;
    
	private:
	int _sensorPin;
	int _Reading;


};

#endif