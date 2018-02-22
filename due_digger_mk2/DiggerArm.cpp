#include "Arduino.h"
#include <Servo.h>
#include "DiggerArm.h"
#include "RobotSelector.h"

//swapped out gripper.read() with grip
//swapped out pitch.read() with pitch
DiggerArm :: DiggerArm()
// DiggerArm :: DiggerArm(int pwmPitch, int pwmGrip)
{
// _pwmPitch=pwmPitch;
// _pwmGrip=pwmGrip;

// int pos1=90;
// int pos2=180;
// pitch=pos1;
// grip=pos2;

pitch=90;
grip=80; //previously 180 //JSP
prev_pitch = 90; //JSP
prev_grip = 80; //JSP
}

void DiggerArm :: Attach(){
 GripperServo.attach(GripPin);
 PitchServo.attach(PitchPin);
}


void DiggerArm :: GripperGo(int pos1)
{
  //gripper->attach(_pwmGrip);
  if(pos1 > prev_grip)
  {
    while(prev_grip != pos1)
    {
      GripperServo.write(prev_grip+1);
      prev_grip++;
	  delay(10);    
    }
  }
  if(pos1 < prev_grip)
  {
   while(prev_grip != pos1)
    {
      GripperServo.write(prev_grip-1);
			prev_grip--;
      delay(10);    
    } 
  }
	prev_grip = pos1;
	Serial.println(prev_grip);
}

void DiggerArm :: PitchGo(int pos2)
{
  if(pos2 > prev_pitch)
  {
  
    while(prev_pitch != pos2)
    {
      PitchServo.write(prev_pitch+1);
			prev_pitch++;
      delay(20);
    }
  }
  if(pos2 < prev_pitch)
  {
   while(prev_pitch != pos2)
    {
      PitchServo.write(prev_pitch-1);
      prev_pitch--;
	  delay(20);    
    } 
  }
	prev_pitch = pos2;
	
}
