/* this file contains drive methods and functions */

#include "Arduino.h"
#include "RobotSelector.h"
// #include "MasterSlaveProtocol.h"
#include "DigitalProximity.h" //possibly a bug. compiler cant find extern DigitalProximity
// #include "IRsensor.h" //possibly a bug. compiler cant find extern IRsensor
//---used by Forward, Backward, Right, Left, Stop
extern char lastDriveState;
extern MotorBoard Drive;
extern uint16_t x1, xc, x7, xt; //declare storage variables
//---used by DriveForward
extern double Setpoint, Input, Output; //define vars
extern myPID PD;
extern uint16_t Area1, Areac, Area7, Areat; //declare storage variables;
extern bool goingIn;
//used by ...
extern void GetDetectedSigs();
extern bool Object[]; //various objects. MAY NEED TO SPECIFY INDIVIDUALLY
// extern bool Object[6]; //COTTON
//---used by FollowLane
extern void myDelay(unsigned long);
extern void TurnHeading(float);
extern unsigned long last_dash; //keep track of last robot dash forward if trail is missing
//---used by pick_going_charging_direction
extern uint16_t DUMPING_TARGET_THRESH;
//---used by go_docking
extern void bumpDelay(unsigned long delayTime);
extern bool isChargerDetected();
extern void handleContact();
// extern bool handleIRcontacts();
extern void enable_ChargingMode();
extern void enable_GoingInMode();
extern void enable_RestingMode();
extern void enable_GoingCharging();
extern bool CheckPower();
// extern GripperSensor HeadSensor;
extern  DigitalProximity FrontBumpSensor;
// extern IRsensor IRright;
// extern IRsensor IRleft;
// extern GripperSensor HeadSensor;
//---used by redock
extern PowerRelay Relay;  
//---used by get_back_on_trail
extern int current_target_heading;

static volatile int nolane = 0; //tracks when lane is out of view

void Forward(int speed){
  Drive.RightForward(speed);
  Drive.LeftForward(speed);
  lastDriveState=drivingForward;
}
//----------------------------------------------------
void Backward(int speed){  
  Drive.RightBackward(speed);
  Drive.LeftBackward(speed);
  lastDriveState=drivingBackward;
}
//----------------------------------------------------
void Right(int speed){
  Drive.RightBackward(speed);
  Drive.LeftForward(speed);
  lastDriveState=turningRight;
}
//----------------------------------------------------
void Left(int speed){
  Drive.RightForward(speed);
  Drive.LeftBackward(speed);
  lastDriveState=turningLeft;
}
//----------------------------------------------------
void Stop(){
 Drive.RightStop();
 Drive.LeftStop();
 lastDriveState=stopped;
}
//----------------------------------------------------
void DriveForward(uint16_t x){
	/* this method implements PID controller to drive forward
	input is a pixy block x coordinate 
	pwmR=+ output
	pwmL=- output
	*/
	Input=x;
	//Setpoint=160; // 320/2=160, center pixel
	PD.Compute(); //updates controller
	//command right wheel
	#if defined(ROBOT_A) || defined(ROBOT_B) || defined (ROBOT_C)
		int pwmR=BASE_SPEED + (int)Output;
	#endif

	#ifdef ROBOT_D
		int pwmR=BASE_SPEED + (int)Output; //JSP
		//int pwmR=BASE_SPEED - (int)Output; //Vadim's code
	#endif

	#ifdef ROBOT_E //JSP
		int pwmR=BASE_SPEED + (int)Output; // JSP
	#endif // JSP

	if (pwmR>255){ 
		pwmR=255; //guard against overflow 
	} 
	if (pwmR < 0 ){ //differential turning is needed
		pwmR=0;
 // pwmR=-1*pwmR; //flip the sign
  // if(pwmR>255){ 
  // pwmR=255; //guard against overflow
  // } 
 // Drive.RightBackward(pwmR);
	}
 // else{
	Drive.RightForward(pwmR);
	// Serial.print("Right: ");
	// Serial.println(pwmR);
 // }

	//command left wheel
	#if defined(ROBOT_A) || defined(ROBOT_B) || defined (ROBOT_C)
		int pwmL=BASE_SPEED - (int)Output; //Note the sign of PV is different comparing to the statement above
	#endif
	
	#ifdef ROBOT_D
		int pwmL=BASE_SPEED - (int)Output; //JSP
		//int pwmL=BASE_SPEED + (int)Output; //Note the sign of PV is different comparing to the statement above
	#endif

	#ifdef ROBOT_E
		int pwmL=BASE_SPEED - (int)Output; //JSP
	#endif

	if (pwmL>255){ 
		pwmL=255;
	} //guard against overflow
	
	if (pwmL < 0 ){ //differential turning is needed
		pwmL=0;
		// pwmL=-1*pwmL; //flip the sign
		// if(pwmL>255){ 
		// pwmL=255;
		// } //guard against overflow
	// Drive.LeftBackward(pwmL);
	}
	// else{
	Drive.LeftForward(pwmL);
	// Serial.print("Left: ");
	// Serial.println(pwmL);
	// }
	// Serial.print(pwmR); 
	// Serial.print('\t');
	// Serial.println(pwmL);
}
//----------------------------------------------------
void DriveBackward(uint16_t x){
	/* this method implements PID controller to drive forward
	input is a pixy block x coordinate  */
	Input=x;
	//Setpoint=160; // 320/2=160, center pixel
	PD.Compute(); //updates controller
	//command right wheel
	int pwmR=BASE_SPEED - (int)Output; // used to be +
	if (pwmR>255){ 
		pwmR=255; //guard against overflow 
	} 
	
	if (pwmR < 0 ){  //guard against overflow
		pwmR=0;
	} 
	Drive.RightBackward(pwmR);
 

	//command left wheel
	int pwmL = BASE_SPEED + (int)Output; // used to be -
	if (pwmL>255){ 
		pwmL=255; //guard against overflow 
	} 
	if (pwmL < 0 ){  //guard against overflow
		pwmL=0;
	} 
	Drive.LeftBackward(pwmL); 
	// Serial.print(pwmR); Serial.print('\t'); Serial.println(pwmL);
}
//----------------------------------------------------
void FollowLane(){
	// Serial.println("In FollowLane()...");

	WDT_Restart(WDT);
	/* this method will make the robot to follow a pheromone trail, calls PID controller method
	DriveForward(int x). 
	Every so often the robot does a dash/kick forward to help it travel faster and 
	go over obstacles (loose GM) better */
	// Serial.println("your pixy camera is not working");
	GetDetectedSigs(); //poll camera
	// Serial.println(" ;-) "); //VADIM
	if(goingIn){
		if(COTTON){ // area7 is associated with the area of cotton seen
			if(Area7>200){ //150
				DriveForward(x7);
				// Serial.print('c');
				// Serial.print("\t");
				// Serial.println(x7);
				return;
			}
		}
	}

	if(TRAIL1){
		if(Area1>150){
			Serial.println("Driving forward");
			DriveForward(x1);
			nolane = 0;
			// Serial.print('t');
			// Serial.print("\t");
			// Serial.println(x1);
		}
		
		if(Area1<150){
			nolane = 1;
			Serial.println("Cannot find pheromone trail");
		}
		return;
	}
 // else{ //got rid of this portion of the code, could be giving us current spikes
  // if( (millis()-last_dash) > 1500){
  // Forward(255); delay(100); //dash
  // // Forward(BASE_SPEED); //return to base speed forward
  // Forward(BASE_SPEED); //return to base speed forward
  // last_dash=millis();
  // }
 // }
}

void FollowLaneBackward(){
/* this method will make the robot to follow a pheromone trail BACKWARDS, calls PID controller method
DriveForward(int x). 
WDT_Restart(WDT);
	/* this method will make the robot to follow a pheromone trail, calls PID controller method
	DriveForward(int x). 
	Every so often the robot does a dash/kick forward to help it travel faster and 
	go over obstacles (loose GM) better */
	// Serial.println("your pixy camera is not working");
	GetDetectedSigs(); //poll camera
	// Serial.println(" ;-) "); //VADIM
	// if(COTTON){ // area7 is associated with the area of cotton seen
		if(Areat>100){ //150
			DriveBackward(xt);
			// Serial.print('c');
			// Serial.print("\t");
			// Serial.println(x7);
			return;
		}
	


	if(TRAIL1){
		if(Area1>150){
			Serial.println("Driving forward");
			DriveBackward(x1);
			// Serial.print('t');
			// Serial.print("\t");
			// Serial.println(x1);
			nolane = 1;
		}
		
		if(Area1<150){
			Serial.println("Cannot find pheromone trail");
			nolane=0;
		}
		return;
	}
 // else{ //got rid of this portion of the code, could be giving us current spikes
  // if( (millis()-last_dash) > 1500){
  // Forward(255); delay(100); //dash
  // // Forward(BASE_SPEED); //return to base speed forward
  // Forward(BASE_SPEED); //return to base speed forward
  // last_dash=millis();
  // }
}

void FollowChargingTrail(){
 if(CHARGING_TRAIL){
  if(Areac>50){
  DriveForward(xc);
  }
 }

}

//---------------------------------------------------------
/* Going Charging Support */
bool pick_going_charging_direction(){
// bool is_going_out;
// GetDetectedSigs(); //poll camera
// if(DUMPING_BEACON){ //if the robot sees the dumping beacon, and its too close
 // if(Areab > DUMPING_TARGET_THRESH/2){//area protection thresh
  // TurnHeading(IN_DIRECTION); //head towards excavation area 
  // is_going_out=false; //record turning decision
 // }
// }
// else{
 // TurnHeading(OUT_DIRECTION); //head towards dumping beacon
 // is_going_out=true; //record turning decision
// }
// return is_going_out;
TurnHeading(OUT_DIRECTION); //head towards dumping beacon
return true;
}
//----------------------------------------------------
/* ChargingMode  Support */

bool redock(){
	// lcd.clear();
	// lcd.setBrightness(30);
	// lcd.print("Redocking charge station");
// printFresh("Redocking charge station");

WDT_Restart(WDT);
 /* this method makes the robot try to reattach itself to a charging station
 if it gets pulled or pushed away from it while charging */
 Backward(BASE_SPEED); delay(500); Stop(); delay(100); // nudge back slightly
 // TurnHeading(STARBOARD_DIRECTION); //face charging station 
 
 //now, lets assume that the charging station is right there !!!
 unsigned long forwardDashStart=millis();
 Forward(BASE_SPEED); //go forward
  while(!CHARGER || !DUMPING_SWITCH){
  WDT_Restart(WDT);  
  /* do nothing, just drive */
   delay(50); //ensure firm parking
   if( millis() - forwardDashStart > 3000 ){//timeout guard
   //Backward(BASE_SPEED); delay(1000); 
   Stop(); //uh oh, something is wrong 
   return false;
   }   
  }
 // delay(20); //ensure firm parking
 Stop(); delay(100);
 Relay.PowerOff(); delay(2000); //turn power off
 return true; //success !
}






/*
 void advanceForward(){

 if( (millis()-last_dash) > 1000){
  Forward(BASE_SPEED); myDelay(100); //dash
  // Forward(BASE_SPEED); //return to base speed forward
  Forward(BASE_SPEED); //return to base speed forward
  last_dash=millis();
 }

} */

