/*
	This class sets up controls for the arm
    1/22/2015, got rid of pin storing with variables, replaced with macros
*/

#ifndef DiggerArm_h
#define DiggerArm_h

#include "Arduino.h"
#include <Servo.h>
#include "RobotSelector.h"


class DiggerArm
{
	public:
		DiggerArm();
		// DiggerArm(int pwmPitch, int pwmGrip);
		void GripperGo(int pos1);
		void PitchGo(int pos2);
        void Attach();
		int pitch; //last pitch pos
		int grip;  //last grip pos
		int prev_pitch; //JSP previous pitch angle
		int prev_grip;	//JSP previous grip angle
	private:
        // int _pwmPitch;
		// int _pwmGrip;

        Servo PitchServo; //declare a pitch  object
		Servo GripperServo; //declare a gripper object
};




#endif
