// ********** BEGIN {MISC TUNABLE CONSTNATS} ----------
#define GO_DIG_PROB   50  
#define GO_REST_PROB  50 //not used in a code, included for amusement purposes.

// #define DUMPING_TARGET_THRESH 5000 //minimum area of a dumping target to be detected  //10K works up close with the beacon turned sideways
#define CHARGING_GROUND_TRAP 100 //minimum area the robot must see
#define CHARGING_TARGET_THRESH 1000 //minimum area to start driving to a docking station
#define CHARGING_DOCKING_THRESH 20000 //area needs to be seen to initate docking  //unused i think
#define LOWEST_ALLOWABLE_VOLTAGE 3.4 //Kehinde: 3.8, 3.4//5v Power is lost when battery level reaches 3v, step up chip no longer functions  //3.3 too low
#define CHARGED_VOLTAGE 4.1 //3.8 // 3.65 appears to be the max voltage  //3.79 was too high

// #define TURNING_THRESH 1000 //minimum area needed to terminate feedback turning  //modified

////////////////////////////////////////////// Pitch Servo Parameters //////////////////////////////////////////////
#define LOW_ROW_ANGLE   80 //60 //45, JSP  //pitch servo is commanded to point toward ground
#define HIGH_ROW_ANGLE  140 //110 //120 //100, JSP //pitch servo is commanded to point toward the ceiling without blocking camera 
#define MID_ROW_ANGLE   110 //60//85, JSP//pitch servo is commanded to be parallel to ground
#define TRAVEL_ANGLE    60//90, JSP     //pitch servo will be maintained around this setpoint while the robot is driving 

////////////////////////////////////////////// Gripper Servo Parameters //////////////////////////////////////////////
#define CLOSED_POS 45 //80 //110//179, JSP  //grip servo fully closed angle
#define OPEN_POS 120 //55 //90, JSP     //grip servo fully open angle
#define MID_POS 80 //65 //JSP //grip servo half-open angle

#define ForceSensorThresh 120 // JSP

////////////////////////////////////////////// Capacitive Sensor Parameters //////////////////////////////////////////////
// #define AntThresh 3 // Ant Detection Threshold for Capacitive Sensor

#define WallSingleThresh1	500 //Wall Detection, Single Contact Threshold for Capacitive Sensor - Lower
#define WallSingleThresh2 650 //Wall Detection, Single Contact Threshold for Capacitive Sensor - Upper

#define AntSingleThresh1	50
#define AntSingleThresh2	500

#define WallDoubleThresh1 10000 //1 //for Wall Detection, two or more contact threshold for Capacitive Sensor
#define WallDoubleThresh2 100000 

#define AntDoubleThresh1 1	//for Ant Detection, two or more contact threshold for Capacitive Sensor - Lower
#define AntDoubleThresh2 50 // Upper

////////////////////////////////////////////// PixyCam Parameters //////////////////////////////////////////////
#define MINIMUM_AREA_THRESH 20   //ensure that only blocks that are at least this big are considered. Also can be configured in camera via PIXYMON

////////////////////////////////////////////// IMU Direction Parameters //////////////////////////////////////////////
#define OUT_DIRECTION 263 //240//291//300 // previously set to 270-  //modified // Ross10/11 changed 270->291 
#define IN_DIRECTION  90 //89//30 //  used to be 60  //modified
#define GET_BACK_DIRECTION 25 //compass direction
#define CHARGING_DIRECTION 240//240//300 //170 is at the wall, towards me
#define DIRECTION_UNCERTAINTY 60 //used to be 5. increased to 10
#define PORT_DIRECTION        357 //11//11//355//350    //left with respect to IN_DIRECTION  //modified
#define STARBOARD_DIRECTION   17 //150//167   //right with respect to IN_DIRECTION  //modified Ross10/11 changed 170->200

////////////////////////////////////////////// IMU Bias Parameters //////////////////////////////////////////////
#define HX_MIN -0.1600
#define HX_MAX 0.1800

#define HY_MIN -0.1800
#define HY_MAX 0.0900

#define PHD 0 // Positive Heading Direction: 0 for CW and 1 for CCW

#define DIGGING_INTERRUPT_DELAY 500 //used to pause robot if its bumped into while digging
#define GOINGCHARGING_INTERRUPT_DELAY 500 //used to pause robot if its bumped into while going charging
#define CHARGINGMODE_INTERRUPT_DELAY 500  //used to pause robot if its bumped into while going charging
#define CONTINUOUS_CONTACT 1000    //if two contacts occur within this time, they are treated as one
#define CONTACT_TIME_LIMIT 2000   //cap limit on contact //4s
#define CONTACT_RESET_TIME 2000 //5000 //used to reset contact switches if they are disabled by software //15s
#define SAW_TRAILS_TIMEOUT 10000  //if the robot fails to see a pheromone trail, force it to back out and turn back on the trail 
#define RIGHT_IR_THRESH 850 //threshold for right IR sensor reading. was set at 400 prior
#define LEFT_IR_THRESH 850  //threshold for left IR sensor reading
#define RIGHT_IR_MAX_TIME 4000 //4 secs
#define LEFT_IR_MAX_TIME  4000
#define BOTH_IR_MAX_TIME  4000 //10 secs
#define HEAD_IR_MAX_TIME  4000
//IR tresholds: 700, blind zone 8s, min 320, max 699
#define COTTON_IR_SUPRESS 30000 //pixels squared. measured the area by printing what it is close 
#define COTTON_START_DIGGING 46334 //pixels squared measured the area by printing what it is super close
// --- PID control stuff  //modified
#define Kp 2.0 //3.5 // Ross 11/13 1.5 -> 2 //2 //1.5 worked fine// 3.5 on robot A //modified //
#define Ki 0.1 //0
#define Kd 0  //.2
#define PD_SAMPLE_TIME 80
#define BASE_SPEED 200 // Ross 11/13 160 -> 120 //worked okay with 100
#define PV_adjmax 205  //155f
#define PD_expected_limit 600 

#define drivingForward  1
#define drivingBackward 2
#define turningRight    3
#define turningLeft     4
#define stopped         0

//--- Turning stuff
#define SLOW_TURNING_RATE 11 //used to be 15, 13
#define TURNING_TIMEOUT     1500 //8 seconds timeout
#define DEFAULT_TURNING_SPEED 155 //used to be 255
// ********** END {MISC TUNABLE CONSTNATS} ----------
#define R1 3.3 //k ohms, actual resistance 
#define R2 3.3 //k omhs, two 3.3k resistors in parallel