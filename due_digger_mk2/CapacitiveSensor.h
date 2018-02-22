#ifndef CapacitiveSensor_h
#define CapacitiveSensor_h
#include "Arduino.h"
#include "RobotSelector.h"
#include "mpr121.h"


class CapacitiveSensor{
	public:
		CapacitiveSensor(int CapSensorPin);
		void setup();
		void readSensorSignal();
		int readTouch();
		void mpr121_setup();
		void set_register(int address, unsigned char r, unsigned char v);
		int getOneContact(int PinNum);
		int getDetectedContacts();
		int getSwitchState();
		bool isDetected();
		byte LSB_CAP;
		byte MSB_CAP;
		byte aa1;
    byte aa2;
    byte pin0L;
    byte pin0M;
    byte pin1L;
    byte pin1M;
    byte pin2L;
    byte pin2M;
    byte pin3L;
    byte pin3M;
    byte pin4L;
    byte pin4M;
    byte pin5L;
    byte pin5M;
    byte pin6L;
    byte pin6M;
    byte pin7L;
    byte pin7M;
		byte pin8L;
    byte pin8M;
		byte pin9L;
    byte pin9M;
	private:
		int irqpin;
		int checkMeasurementNum;
};

#endif