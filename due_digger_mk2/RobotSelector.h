//include this .h file to libraries
//this will enable to program each robot individually 
#define RobotSelector_h

// TESTING VARIABLES
	// TEST_IMU,
	// TEST_IMU_CAL,
	// TEST_FORCE,
	// TEST_MAG,
	// TEST_CAP_0, 	// REMEMBER THAT THE THRESHOLDS MIGHT BE DIFFERENT WHEN THE ROBOT IS PLUGGED IN. THIS WILL MAKE IT HARDER TO DEBUG ISSUES ASSOCIATED WITH THE CAPACITIVE SENSORS
	// TEST_CAP_1,
	// TEST_CAP_2,
	// TEST_CAP_3,
	// TEST_CAP_4,
	// TEST_CAP_5,
	// TEST_CAP_6,
	// TEST_CAP_7,
	
	
	// TEST_CHARGER,
	// TEST_CURRENT,
	// TEST_VOLTAGE,
	// TEST_POWER_SENSORS,
	// TEST_DRIVE_MOTORS, 
	// TEST_SERVO_MOTORS, 
	// TEST_CAMERA,
	// TEST_GRIPPER_SENSOR,

	// TEST_ACCEL,
	// TEST_CHARGING,
	// TEST_ODOMETRY,
	// TEST_FollowLane,
	// TEST_FollowLane_Back

	// TEST_TURN_HEADING,
	// TEST_PICK_DIRECTION,
	// TEST_PID_CONTROLLER,
	// TEST_NOTHING, */
#define TEST_CASE TEST_CHARGING
#define ROBOT_C

// ********** BEGIN {SET BEHAVIOR} **********

//lorenz stuff
#define PROBABILITY_DIG 0 //0 for active, 1 for lorenz
#define RESTING_TIME 20000 // number of seconds before rerolling probabilty in lorenz mode -- was originally 20000
#define lorenzProb 21.61    

//useless run stuff turn back if it did not reach the face
#define ALLOW_USELESS_RUNS 1 // 1 is allow
#define USELESS_RUN_THRESH 75000 //used to be 75000 

// This will be used in the handleContact
#define REVERSE_ON_CONTACT 0
#define REVERSE_ON_CONTACT_PROB 0

//dont worry about this stuff
#define ALLOW_CHARGING_ON_REST 1
#define ALLOW_POWER_SAVINGS 0
//VADIM. FIND A TIMER FROM A PREVIOUS CODE> THIS WAS BROKEN 
// #define BACKWARDS_KICK_TIME 1000   //every so often the robot will drive back for this many seconds. needed for avoiding getting stuck 
//run trhesh: 40s too short, 120s too long
// **********  END   {SET BEHAVIOR} ---------

#ifdef ROBOT_A
#include "defA.h"        //definition macros
#include "constantsA.h"  //tunable constants
#endif

#ifdef ROBOT_B
#include "defB.h"        //definition macros
#include "constantsB.h"  //tunable constants
#endif

#ifdef ROBOT_C
#include "defC.h"        //definition macros
#include "constantsC.h"  //tunable constants
#endif

#ifdef ROBOT_D
#include "defD.h"        //definition macros
#include "constantsD.h"  //tunable constants
#endif

#ifdef ROBOT_E
#include "defE.h"        //definition macros//BANI JSP
#include "constantsE.h"  //tunable constants
#endif
// #endif