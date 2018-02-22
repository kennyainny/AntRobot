/*
Author : Vadim Linevich
email  : vadim.linevich@gmail.com

Modified by : Jungsoo Park
email : ryanryan0906@hotmail.com
Last Modified Date: 05/26/2016

Modified by : Ross Warkentin
email: ross.warkentin@gmail.com
Last Modified Date: 07/22/2017

Property of CRABLAB, Georgia Institute of Technology, School of Physics
[2013]-[2014]

This is the main program, written in embedded C++, to be compiled and uploaded
to Arduino family MCU's using ARDUINO IDE (works with v1.5.7)'

11/13/2014: the code is specific to Arduino Due

WATCHDOG RESET IS ENABLED
read this forum http://forum.arduino.cc/index.php?topic=314647.0
*/


#define FIO_LINK 1

// // ********** BEGIN {SET BEHAVIOR} **********
// //--comment things out if unwanted
// //lorenz stuff
// #define PROBABILITY_DIG 0 //0 for active, 1 for lorenz
// #define RESTING_TIME 3000// number of seconds before rerolling probabilty in lorenz mode -- was originally 20000
// int lorenzProb = 50;

// //useless run stuff turn back if it did not reach the face
// #define ALLOW_USELESS_RUNS 0 // 1 is allow
// #define USELESS_RUN_THRESH 75000 //used to be 75000

// //dont worry about this stuff
// #define ALLOW_CHARGING_ON_REST 0
// #define ALLOW_POWER_SAVINGS 0
// //VADIM. FIND A TIMER FROM A PREVIOUS CODE> THIS WAS BROKEN
// #define BACKWARDS_KICK_TIME 1000   //every so often the robot will drive back for this many seconds. needed for avoiding getting stuck
// //run trhesh: 40s too short, 120s too long
// // **********  END   {SET BEHAVIOR} ---------

// 2 ants: P1=11, P2=89
// 3 ants: P1=4, P2=21, P3=75
// 4 ants: P1=1.80,  P2=8.56, P3=25.29, P4=64.35

// #define MANUAL_ON 1
// ********** BEGIN {LIBRARY IMPORT} **********
/* note: included libraries with "" must be in the same folder with .ino sketch, while external <> libraries must be in the arduino library folder (either documents>arduino or program files>arduino */
// #include "RobotSelector.h" //load appropriate robot specific definitions and constants
#include "MotorBoard.h" //loads a custom library to set up drive motors
//power relay, switches, drive methods, vision methods have def.h files
#include "DiggerArm.h"	 //loads a custom library to provide easy arm control command
#include "CurrentSensor.h" //loads a custom library to read data from current sensors
//#include "AntComm.h"       //loads a custom library to set up a wireless communication (Xbee Series 2 Radio)
#include "GripperSensor.h" //loads a custom library to set up a reflective gripper sensor#include <Wire.h>
#include <Wire.h>		   //loads external arduino library to set up I2C communication protocol. Used to talk to Pixy camera
// #include <PixyI2C.h>       //loads external arduino library to set up CM Pixy Camera Sensor
#include <PixyUART.h>	//loads external arduino library to set up CM Pixy Camera Sensor
#include <SPI.h>		 //loads external arduino library. This library is not used but required by SFE_LSM9DS0 library
#include <SFE_LSM9DS0.h> //loads an IMU library written by a sensor manufacturer and modified by me
#include "PowerRelay.h"  //loads a custom library to set up a relay to turn power on and off
#include "Voltmeter.h"   //loads a custom library to set up an analog pin to read voltage on a source battery
#include "myPID.h"		 //loads an open source library to set up PID/PD control.
#include <Servo.h>		 //loads a servo library, also included inside DiggerArm, here as a reminder. HAS A BUGGED read() method as of 8/11/2014

//#include "OpticalEncoder.h"//loads a custom library to read encoders using interrupt routine
//#include "Encoders.h"      //sets up ISR interrupt functions. Should merge optical encoders into here
//#include "Switches.h"   //methods containing ISR interrupt  functions used to keep track of which switches are pressed
#include "driveMethods.h"  //contains locomotion drive functions
#include "visionMethods.h" //contains functions setting up Pixy camera
//#include "LED.h" //loads a custom library that turns LED on and off, used for debugging purposes
#include "ChargingDetector.h" //load a file to set up charging dection
//#include "IRsensor.h"  //load a custom library to set up IR distance sensors
//#include "DigitalProximity.h" //loads a custom library to set up a digital proximity sensor
//#include "AntBuzzer.h" //loads a custom library to set up a speaker/buzzer for debugging purposes
#include "enableModes.h" //sets up mode changing

#include "LinkArduinosI2c.h"	 //loads a header file with high level I2C communication commands to link main board and radio MCUs
#include "MasterSlaveProtocol.h" //loads custom protocol used to communicate between two boards
#include "WireReset.h"			 //enables arduino to self reset iself
#include "RandomGenerator.h"	 //generates random numbers to enable probabilistic behaviour.
#include "mpr121.h"				 //for capacitive sensor
#include "CapacitiveSensor.h"
#include "MAG3110.h"

#include <SPI.h> // SPI comm with SD card
#include <SD.h>  // SD card library
File myFile;
#define SD_CLR_SWITCH 6 // Clear files from SD card

#define CURRENT_SAMPLE_SIZE 100 //sets a number of samples to be used for reading current averages
#define VOLTAGE_SAMPLE_SIZE 100

#include "hardSerLCD.h"
hardSerLCD lcd; // library written by Ross to allow the Due to communicate with the LSC screen via hardware serial

// #include <wdt.h>
// **********  END   {LIBRARY IMPORT} ---------

// ********** BEGIN {GLOBAL VARIABLE DECLARATION} **********
// --- actuator setup
// MotorBoard Drive(enable1,phase1,enable2,phase2); //sets up motor drive, calling class MotorBoard to create an object "Drive"
MotorBoard Drive; //sets up motor drive, calling class MotorBoard to create an object "Drive"
DiggerArm Arm;	//sets up arm control, calling class DiggerArm  to create an object "Arm"
// --- current sensor setup
CurrentSensor Current;		   //sets up a current sensor. CATION: removed constructor input
#define CURRENT_SAMPLE_SIZE 50 //sets a number of samples to be used for reading current averages
// --- gripper sensor variables
//sensor threshold is defined inside GripperSensor.h
//GripperSensor HeadSensor;
//sets up proximity sensors used to detect granular media //JSP
GripperSensor LUSensor(LUGripperPin);
GripperSensor LDSensor(LDGripperPin);
GripperSensor RUSensor(RUGripperPin);
GripperSensor RDSensor(RDGripperPin);
MAG3110 FSensor;

CapacitiveSensor CapSensor(CapacitiveSensorPin);
// --- IMU stuff
#define LSM9DS0_XM 0x1D						  // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G 0x6B						  // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM); //sets up IMU (gyrsocope, magnetometer, accelerometer)

// --- camera sensor setup
/* vision stuff */
PixyUART pixy;   //sets up pixy camera sensors, calling class PixyUART to create an object "pixy"
uint16_t blocks; //store number of blocks detected when frames are sampled

//RIGHT NOW, THERE ARE ONLY 3 VISION ITEMS
bool Object[5];									   //array to hold boolean variables to tell whether or not the object of interest has been detected
uint16_t x1, xc, x7, xt, xCharging;				   //declare storage variables  x1=pheromone trail, xc= charging pheromone, x7 and xt are cotton stuff
uint16_t Area1, Areac, Area7, Areat, AreaCharging; //declare storage variables;

double Setpoint, Input, Output; //define vars
double KP = Kp;
double KI = KI;
double KD = KD;
myPID PD(&Input, &Output, &Setpoint, KP, KI, KD); //PID control
char lastDriveState = 0;						  //used to keep track of Forward/Backward/Right/Left/Stop commands

// --- switches
int switchState;						//used to set up contact switches
bool disableContacts = false;			//flag used to mask interrupts if the robot is stuck in a routine for too long
unsigned long whenDisabledContacts = 0; //timer used to remember when contact switches were disabled
unsigned long howLongWasContact = 0;	//timer counter used to track duration of a contact
unsigned long whenWasLastContact = 0;   //record time of last contact

int current_target_heading = IN_DIRECTION; //in charging mode, keep track of desired dir
int LFC;
int RFC;
int LSBC;
int RSBC;
int LSFC;
int RSFC;
int LBC;
int RBC;
int FS;
int LS;
int RS;
int BS;

// bool sawSomeone;
bool goingIn;
bool goingOut;
bool goingCharging;
bool chargingMode;
bool diggingMode; //used to declare that the robot is in a process of digging
bool dumpingMode; //used to declare that the robot is disposing of its payload
bool restingMode;
bool exitTunnelMode;
bool turnReversalMode;
int nextMode;
bool turn_reversal_direction; //decides which direction the robot will turn in turn reversal

bool manualMode = false; //used to mask i2c Write back, i2c appears to lock up when writing back and forth
bool uselessRun = false; //used in jamming

PowerRelay Relay; //initiating an object to turn motor and camera power on and off using 1 pin
//--- voltage tap
Voltmeter Voltage;
int checkPowerCount = 0; //variable to prevent robot from going charging due to a random voltage drop
#define CHECK_POWER_CNT_THRESH 5 //10 //kehinde 500 - 10

// This enumeration is used in the switch-case statements in loop() to conduct any necessary tests
enum Test
{
	TEST_IMU,
	TEST_IMU_CAL,
	TEST_FORCE,
	TEST_MAG,
	TEST_CAP_0, // remember that the thresholds might be different when the robot is plugged in. This will make it harder to debug issues associated with the capacitive sensors
	TEST_CAP_1,
	TEST_CAP_2,
	TEST_CAP_3,
	TEST_CAP_4,
	TEST_CAP_5,
	TEST_CAP_6,
	TEST_CAP_7,
	TEST_CHARGER,
	TEST_CURRENT,
	TEST_DRIVE_MOTORS,
	TEST_SERVO_MOTORS,
	TEST_CAMERA,
	TEST_VOLTAGE,
	TEST_GRIPPER_SENSOR,
	TEST_POWER_SENSORS,
	TEST_TURN_HEADING,
	TEST_PICK_DIRECTION,
	TEST_PID_CONTROLLER,
	TEST_ACCEL, //Kehinde
	TEST_CHARGING,
	TEST_NOTHING,
};

//--- set up IR distance sensors
//IRsensor IRright(IRsensorRightPin);
//IRsensor IRleft(IRsensorLeftPin);
//DigitalProximity FrontBumpSensor(ProximityBumpPin);
// ********** END   {GLOBAL VARIABLE DECLARATION} ----------

// bool watchdogFlag=1; //is set high when action is performed and pulled low when code cycles. If the flag stays low for a while, robot is presumed to be stuck
// unsigned long watchdogReset; //store time when watchdog was last reset

//quick global vars to test out some stuff

unsigned long last_dash;

unsigned long dirCheckTimer; //quick workaround to prevent instantaneous dir changing
bool dirCheckFlag;
int numOfConsequitiveBackwardKicks = 0;

// bool preferGyro = false; //duct tape solution to use gyroscope to do 180 degree turns

int StartIndicatorAddr = 0;

void printFresh(String lcddata)
{
	lcd.clear();
	lcd.setBrightness(30);
	lcd.print(lcddata);
}

//Kehinde: move the function here
void writeSDcard(char tag, String data, unsigned long time)
{
	// choose file to write
	switch (tag)
	{
		case 'N':
			myFile = SD.open("conlog.csv", FILE_WRITE); //BANI
			break;
		case 'M':
			myFile = SD.open("statelog.csv", FILE_WRITE);
			break;
		case 'C':
			myFile = SD.open("currlog.csv", FILE_WRITE);
			break;
		case 'V':
			myFile = SD.open("voltlog.csv", FILE_WRITE);
			break;
		case 'W':
			myFile = SD.open("powerlog.csv", FILE_WRITE);
			break;
		case 'D':
			myFile = SD.open("chargelog.csv", FILE_WRITE);
			break;
	}

	//Kehinde:form payload
	String payload = data + "," + time;
	//Serial.println(payload);
	// write to SD card
	if (myFile)
	{
		myFile.println(payload);
		myFile.close();
	}
	else
	{
#if DEBUG
		printFresh("Error opening file")
		Serial.println(F("error opening file"));
#endif
	}
}


void WDT_Setup()
{   // my default time is 18 seconds
	//15 seconds - resets when exiting tunnel
	double desiredTimeout = 18;

	double timeout2 = (int)(desiredTimeout * 227); //227

	timeout2 = 0x0fff2000 + timeout2; // 0xfff2000 is very inportant
	WDT_Enable(WDT, timeout2);
	// number of loops:
	// WDT_Restart(WDT); //USE THIS TO RESET WATCHDOG EVERYWHERE
}

// ********** BEGIN (SETUP SCRIPT} **********

unsigned long globalTimerDiff = 0;

//Kehinde: track how long it takes the robot to operate
unsigned long RobotStartTime;
int noOfTrips;
int noOfWallContact;
int noOfAntContact;
bool isNewTrip;

int contactSide = 0; //right =1, left =2

unsigned long startTime;// = millis(); //start time
unsigned long timeDiff;// = startTime - RobotStartTime;
int stationaryThreash;


void setup()
{
	//Kehinde: initialize tracking  variables
	RobotStartTime = millis(); //start time
	stationaryThreash = 3000;//period for checking stationary event
	noOfTrips = 0;
	noOfWallContact = 0;
	noOfAntContact = 0;
	isNewTrip = true;

	
	WDT_Restart(WDT);
	pullResetPinLow(); //pull reset pin low

#if PROBABILITY_DIG
	int someRandValue = analogRead(A4); //read ADC val from the unused pin
	randomSeed(someRandValue);			//set up random generator seed ///input can be changed to analogRead(int unused analog pin);
#endif

	/* establish serial communication */
	Serial.begin(9600); //Establishes Serial communication at a specified baud rate. This can be moved inside of the Ant Comm class
	WDT_Restart(WDT);

	delay(500); //ensure a power cycle after a watchdog reset -- used to be 3s
	WDT_Restart(WDT);

	lcd.begin(&Serial2, 9600); // lcd hardware serial on Serial2

	printFresh("Turning on relay...");
	Relay.PowerOn(); //turn the power to the robot on
	printFresh("Turning on relay...done");

	/* initiate servos and move them to test  */
	printFresh("Turning on servos...");

	Arm.Attach(); //hook up servos to pwm pins

	//Kehinde: raise arm first
	Arm.PitchGo(MID_ROW_ANGLE-10);
	delay(1000);
	
	Arm.GripperGo(MID_POS-10); //OPEN_POS
	delay(500); //myDelay between servo movements

	Arm.GripperGo(CLOSED_POS);
	delay(500); //myDelay between servo movements

	Arm.PitchGo(HIGH_ROW_ANGLE);
	delay(500); //puts an arm in a default configuration. Arm is extended and parallel to ground

	Arm.GripperGo(CLOSED_POS);
	delay(500);

	printFresh("Turning on servos...done");

	printFresh("Turning on IMU...");
	turnIMUon();
	printFresh("Turning on IMU...done");

	// pixy.init();        //Starts I2C communication with a camera

	WDT_Restart(WDT);

	// initiateFioSerial(); //start Fio serial on channel 2

	printFresh("Connecting to SD card...");

	if (!SD.begin(CS))
	{
		Serial.println("SD initialization failed!");
		printFresh("Connecting to SD card...FAILED");
		delay(2000);
	}
	else
	{
		Serial.println("SD initialization done.");
		printFresh("Connecting to SD card...done");
	}

	// Notify start of data logging in the file
	myFile = SD.open("chargelog.csv", FILE_WRITE);
	if (myFile)
	{
		//myFile.println(F("Time (milliseconds), Power (Watts)"));
		myFile.println(F("Trip No,Transition Time(ms),Voltage(V),Current(C),Absolute Time(ms)"));										
		myFile.close();
#if DEBUG
		// Serial.println(F("charging log started"));
#endif
	}
	else
	{
#if DEBUG
		printFresh("Error opening charging log");
		Serial.println(F("error opening file"));
#endif
	}
	delay(100);
	//bani opens contact logging file
	myFile = SD.open("conlog.csv", FILE_WRITE);
	if (myFile)
	{
		//myFile.println(F("Time (milliseconds), Contact Type"));
		myFile.println(F("Trip No,Transition Time(ms),Contact,Type,No of Contact,Switch State,Absolute Time(ms)"));						
		myFile.close();
#if DEBUG
		// Serial.println(F("contact logging started"));
#endif
	}
	else
	{
#if DEBUG
		printFresh("Error opening contact log");
		Serial.println(F("error opening file"));
#endif
	}
		delay(100);
	//bani: modified by Kehinde 110617
	myFile = SD.open("statelog.csv", FILE_WRITE);
	if (myFile)
	{
		//myFile.println(F("Time (milliseconds), State"));
		myFile.println(F("Trip No,Transition Time(ms),State,Absolute Time(ms)"));				
		myFile.close();
		// #if DEBUG
		// Serial.println(F("state logging started"));
		// #endif
	}
	else
	{
#if DEBUG
		printFresh("Error opening state log");
		Serial.println(F("error opening file"));
#endif
	}
		delay(100);
	myFile = SD.open("currlog.csv", FILE_WRITE);
	if (myFile)
	{
		//myFile.println(F("Time (milliseconds), Current (Amperes)"));
		myFile.println(F("Trip No,Transition Time(ms),State,Current(A),Absolute Time(ms)"));						
		myFile.close();
#if DEBUG
		// Serial.println(F("current logging started"));
#endif
	}
	else
	{
#if DEBUG
		printFresh("Error opening current log");
		Serial.println(F("error opening file"));
#endif
	}
		delay(100);
	myFile = SD.open("voltlog.csv", FILE_WRITE);
	if (myFile)
	{
		//myFile.println(F("Time (milliseconds), Voltage (Volts)"));
		myFile.println(F("Trip No,Transition Time(ms),State,Voltage(V),Absolute Time(ms)"));								
		myFile.close();
#if DEBUG
		// Serial.println(F("voltage logging started"));
#endif
	}
	else
	{
#if DEBUG
		printFresh("Error opening voltage log");
		Serial.println(F("error opening file"));
#endif
	}
		delay(100);

	myFile = SD.open("powerlog.csv", FILE_WRITE);
	if (myFile)
	{
		//myFile.println(F("Time (milliseconds), Power (Watts)"));
		myFile.println(F("Trip No,Transition Time(ms),State,Power(W),Absolute Time(ms)"));										
		myFile.close();
#if DEBUG
		// Serial.println(F("power logging started"));
#endif
	}
	else
	{
#if DEBUG
		printFresh("Error opening power log");
		Serial.println(F("error opening file"));
#endif
	}
	

	printFresh("Setting up power sensing...");

	Current.setPin(currentSensorPin);
	Voltage.setPin(voltage_pin);
	printFresh("Setting up power sensing...done");

	// Serial.println(F("Ready")); //debug line
	// watchdogReset=millis(); //initiate watchdog timer
	//initiateSwitches(); //sets up pins for contact switches
	printFresh("Setting up charing detector...");

	initiateChargingDetector(); //set up contact charger detector

	delay(1000);
	printFresh("Setting up charing detector...done");

	//start i2c bus stuff. CALL dof.begin() LAST, because it starts actually sending data
	// initiateMaster(); //start i2c bus , probably redundant
	WDT_Restart(WDT);

	printFresh("Setting up cap sensor...");
	CapSensor.setup();
	delay(1000);
	printFresh("Setting up cap sensor...done");

	printFresh("Setting up mag sensor...");

	FSensor.setup();
	delay(1000);
	printFresh("Setting up mag sensor...done");

	printFresh("Setting up PixyCam...");

	pixy.init(); //Starts I2C communication with a camera
	delay(1000);
	printFresh("Setting up PixyCam...done");

	WDT_Restart(WDT);

	printFresh("Setting up IMU...");

	uint16_t status = dof.begin(); //starts I2C communication with a IMU. status variable should have a hex value of 0x49D4. This should def be called, has calibration functions inside
	Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
	Serial.println(status, HEX);
	Serial.println("Should be 0x49D4");
	Serial.println();
	delay(1000);

	WDT_Restart(WDT);
	/* power electronics */

	/* set up PD or PID control */
	PD.SetMode(AUTOMATIC);
	PD.SetSampleTime(PD_SAMPLE_TIME);		   //sets sample time. default is 100ms
	PD.SetOutputLimits(-PV_adjmax, PV_adjmax); //clamp limits of PD controller feedback
	Setpoint = 160;							   //x coordinate of the center of the camera, 160=320/2

	last_dash = millis(); //initiate timer to keep track of last robot dash forward

	dof.setMagScale(dof.M_SCALE_2GS);
	dof.setMagODR(dof.M_ODR_125); //sets up magnetometers output data rate to the highest (fastest) available setting
	
	//Kehinde: set up accelerometer
	dof.setAccelScale(dof.A_SCALE_2G);
	dof.setAccelODR(dof.A_ODR_125);
	printFresh("Setting up IMU...done");

	// dof.setGyroODR(dof.G_ODR_760_BW_100);  //sets up gyro output data rate to the highest (fastest) available setting
	// dof.setMagODR(dof.M_ODR_100);          //sets up magnetometers output data rate to the highest (fastest) available setting
	/* 
	note that the gyroscope class creates 6 global variables
	gx gy gz  //gyro         readings in x y z
	ax ay az  //accel        readings in x y z
	mx my mz  //magnetometer readings in x y z
	Also, note that accelerometer readings are not much of use. To get a good 
	reading, accelerometer need to be exposed to at least 2g's 
	 */

	// %%
	dirCheckFlag = false;
	dirCheckTimer = millis();

	int pw = CheckPower();

	printFresh("Determining state...");

	determineState();
	printFresh("Determining state...done");

	WDT_Restart(WDT);
}

/**

determineState() was a method that Ross wrote while the robots were still suffering from regular resets due to the Arduino Fio
I think it is still useful for allowing the robots to continue to operate normally, and can usually assess the proper state, though
it is not gauranteed to.

The purpose of the method was, upon a robot reset, to take in observations from the various sensors to assess what the last robot state was prior to resetting.

Ultimately, it is likely not necessary anymore, but may be useful in handling edge cases or odd power issues

**/
void determineState()
{
	//Kehinde 10/30
	startTime = millis(); //start time
	timeDiff = startTime - RobotStartTime;
	writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Determining State", startTime);

	// default guess is goingInMode ie active
	enable_GoingInMode();

	int current_heading = getHeading();
	int goingOutDiff = abs(OUT_DIRECTION - current_heading);
	int goingInDiff = abs(IN_DIRECTION - current_heading);

	// if there is something in the gripper then we want to exit the tunnel
	if (CheckPayload())
	{
		enable_GoingOutMode();
		return;
	}

	Serial.println("In determineState");

#if PROBABILITY_DIG
	//if see green go in
	unsigned long checkTime = millis();
	while (millis() - checkTime < 1000)
	{
		// Serial.println("Check...");

		GetDetectedSigs(); //poll camera
		if (Areat > 10)
		{ // if you see green, keep going
			enable_GoingInMode();
			// Serial.println("Seen green");

			return;
		}
	}

	// if not see green go out
	// else{
	enable_GoingOutMode();
	uselessRun = true; // should just start like this
	// Serial.println("IIIIIIIIIIIII");

	return;
// }
#endif
// Serial.println("?????????");

// If the robot is facing the goingOut direction and there is nothing in the gripper, let's assume that we were in the useless run state.
#if ALLOW_USELESS_RUNS
	if (goingInDiff > goingOutDiff)
	{
		enable_GoingOutMode();
		uselessRun = true;
	}
#endif

	return;
}

// ********** End (SETUP SCRIPT} **********

// ********** BEGIN (MAIN SCRIPT} **********

void loop()
{

	Serial.println("main loop");

	// This is advantageous in that adding a new test is as simple as adding a test state to the global enumeration and adding a test method to a separate test.cpp file (does not currently exist)
	// enumeration defined in global declaration, and TEST_CASE is in RobotSelector
	switch (TEST_CASE)
	{
	case TEST_IMU_CAL:
		TestIMUcal();
		break;

	case TEST_CURRENT:
		float C;
		// pinMode(currentSensorPin, INPUT);
		Serial.println(String(currentSensorPin));
		while (1)
		{
			Serial.println("Analog reading is " + String(analogRead(currentSensorPin)));
			C = Current.ReadAvg(CURRENT_SAMPLE_SIZE);
			Serial.println("Current reading is " + String(C));
			WDT_Restart(WDT);
			printFresh(String(C));
		}
		break;

	case TEST_VOLTAGE:
		float V;
		// pinMode(currentSensorPin, INPUT);
		Serial.println(String(voltage_pin));
		while (1)
		{
			// WDT_Restart(WDT);
			delay(2000);
			// Serial.println("Analog reading is " + String(analogRead(voltage_pin)));
			V = Voltage.GrabAvg(VOLTAGE_SAMPLE_SIZE);
			C = Current.ReadAvg(CURRENT_SAMPLE_SIZE);

			Serial.println("Voltage: " + String(V)+" | Current: " + String(C));

			printFresh("Voltage: " + String(V) + " | Current: " + String(C));
			// printFresh("Analog: " + String(analogRead(voltage_pin)));
			WDT_Restart(WDT);

			// delay(1000);
			// WDT_Restart(WDT);

			// delay(3000);
			// Serial.println("Analog reading is " + String(analogRead(voltage_pin)));
			// V = Voltage.GrabAvg(25);
			// Serial.println("Voltage reading is (25) " + String(V));
			// printFresh("Voltage 25: " + String(V));
			// printFresh("Analog reading is " + String(analogRead(voltage_pin)));

			// delay(1000);
			// WDT_Restart(WDT);
		}
		break;
	case TEST_IMU:
		TestIMU();
		break;
	case TEST_PID_CONTROLLER:
		// if(goingOut){
		TestPDController();
		// }
		break;
	case TEST_FORCE:
		//For Force Sensitive Resistor Test and Calibration
		while (1)
		{
			Serial.println(analogRead(ForceSensor));
			if (CheckPayload())
			{
				Serial.println("Payload Detected");
			}
			delay(200);
		}
		break;

	case TEST_CHARGER:
		while (1)
		{
			WDT_Restart(WDT);

			Serial.println(digitalRead(31));
			Serial.println(digitalRead(33));
			Serial.println(digitalRead(35));
			// Serial.println(digitalRead(31));


			// if (CHARGER)
			// {
			// 	Serial.println("Charger detected");
			// 	printFresh("Charger detected");
			// }
			// else
			// {
			// 	Serial.println("Charger not detected");
			// 	printFresh("Charger not detected");
			// }

			delay(1000);
		}

	case TEST_MAG:
		//for magnetometer MAG3110 sensor test and calibration

		Relay.PowerOff();
		delay(1000);
		Relay.PowerOn();
		FSensor.setup();

		delay(1000);

		while (1)
		{
			WDT_Restart(WDT);
			Serial.println(FSensor.measure_values());
			delay(2000);
		}
		break;

	case TEST_CAMERA:
		Serial.println("Testing camera...");
		TestCamera();
		WDT_Restart(WDT);
		break;

	case TEST_CAP_0:
		//for capacitive sensor test and calibration
		testCap(0);
		break;
	case TEST_CAP_1:
		//for capacitive sensor test and calibration
		testCap(1);
		break;

	case TEST_CAP_2:
		//for capacitive sensor test and calibration
		testCap(2);
		break;
	case TEST_CAP_3:
		//for capacitive sensor test and calibration
		testCap(3);
		break;
	case TEST_CAP_4:
		//for capacitive sensor test and calibration
		testCap(4);
		break;
	case TEST_CAP_5:
		//for capacitive sensor test and calibration
		testCap(5);
		break;
	case TEST_CAP_6:
		//for capacitive sensor test and calibration
		testCap(6);
		break;
	case TEST_CAP_7:
		//for capacitive sensor test and calibration
		testCap(7);
		break;

	case TEST_DRIVE_MOTORS:
		TestDriveMotors();
		break;
	case TEST_SERVO_MOTORS:
		TestServoMotors();
		break;
	case TEST_GRIPPER_SENSOR:
		// Serial.println(analogRead(FGripperPin)); // FGripperPin not defined
		TestGripperSensor();
		// WDT_Restart(WDT);
		break;
	case TEST_POWER_SENSORS:
		TestPowerSensors();
		break;
	case TEST_TURN_HEADING:
		TestTurnHeading();
		break;
	case TEST_PICK_DIRECTION:
		TestPickDirection();
		break;
	case TEST_ACCEL:
		TestAccelReading();
		break;
	case TEST_CHARGING:
		TestCharging();
		break;
	case TEST_NOTHING:
		// Serial.println("Test intentionally ignored");
		break;
	default:
		// Serial.println("Default test case. TEST_CASE not properly assigned?");
		break;
	}

	if (goingIn)
	{
		//Kehinde 10/30
		if(!isNewTrip) //one trip cycle is completed, initialize variables
		{
			// //check if running time	greater than 30mins
			// if (CheckPower() || RobotStartTime >= 1800000) //check the battery before beginnig new trip
			// {
			// 	// startTime = millis(); //start time
			// 	// timeDiff = startTime - RobotStartTime;
			// 	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going In", startTime);

			// 	enable_GoingCharging();
			// 	return;
			// }
			// else //
			// {
				RobotStartTime = millis(); //start time
				// stationaryThreash = 3000;//period for checking stationary event
				noOfTrips +=1;
				noOfWallContact = 0; //contacts per trip
				noOfAntContact = 0;
				isNewTrip = true;
			// }
		}

		//Kehinde 10/30
		// startTime = millis(); //start time
		// timeDiff = startTime - RobotStartTime;
		// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going In", startTime);

		float BatVoltage = Voltage.GrabAvg(50); //double check;
		float C = Current.ReadAvg(50);			 //debug;
		//Kehinde 10/30
		startTime = millis(); //start time
		timeDiff = startTime - RobotStartTime;
		writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going In", startTime);
		writeSDcard('V', String(noOfTrips) + "," + String(timeDiff) + ",Going In," + String(BatVoltage), startTime);
		writeSDcard('C', String(noOfTrips) + "," + String(timeDiff) + ",Going In," + String(C), startTime);

		PD.SetTunings(Kp, Ki, Kd); // Ross' version of resetting the gains -- I don't think this is necessary
		current_target_heading = IN_DIRECTION;
		GoingInMode();
	}

	if (diggingMode)
	{
		//Kehinde 10/30
		// startTime = millis(); //start time
		// timeDiff = startTime - RobotStartTime;
		// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",", startTime);
		
		float BatVoltage = Voltage.GrabAvg(50); //double check;
		float C = Current.ReadAvg(50);
		startTime = millis(); //start time
		timeDiff = startTime - RobotStartTime;
		writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Digging", startTime);
		writeSDcard('V', String(noOfTrips) + "," + String(timeDiff) + ",Digging," + String(BatVoltage), startTime);
		writeSDcard('C', String(noOfTrips) + "," + String(timeDiff) + ",Digging," + String(C), startTime);
		
		current_target_heading = IN_DIRECTION;

		// Serial.println("diggingMode");
		while (diggingMode)
		{
			DiggingMode();
			//Serial.println("Digging Mode"); // JSP //bani commented
		}
	}

	if (dumpingMode)
	{
		//Kehinde 10/30
		 isNewTrip = false;					//trip completed
		// startTime = millis(); //start time
		// timeDiff = startTime - RobotStartTime;
		// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Dumping", startTime);
		// Serial.println("dumpingmode");

		float BatVoltage = Voltage.GrabAvg(50); //double check;
		float C = Current.ReadAvg(50);
		startTime = millis(); //start time
		timeDiff = startTime - RobotStartTime;
		writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Dumping", startTime);
		writeSDcard('V', String(noOfTrips) + "," + String(timeDiff) + ",Dumping," + String(BatVoltage), startTime);
		writeSDcard('C', String(noOfTrips) + "," + String(timeDiff) + ",Dumping," + String(C), startTime);

		current_target_heading = OUT_DIRECTION;
		
		while (dumpingMode)
		{
			DumpingMode();
			//Serial.println("Dumping Mode"); //JSP //bani commented
		}
	}

	if (goingOut)
	{
		//Kehinde 10/30
		// startTime = millis(); //start time
		// timeDiff = startTime - RobotStartTime;
		// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",", startTime);

		float BatVoltage = Voltage.GrabAvg(50); //double check;
		float C = Current.ReadAvg(50);
		startTime = millis(); //start time
		timeDiff = startTime - RobotStartTime;

		writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going Out", startTime);
		writeSDcard('V', String(noOfTrips) + "," + String(timeDiff) + ",Going Out," + String(BatVoltage), startTime);
		writeSDcard('C', String(noOfTrips) + "," + String(timeDiff) + ",Going Out," + String(C), startTime);

		PD.SetTunings(Kp, Ki, Kd); // Ross' version of resetting the gains -- I don't think this is necessary
		current_target_heading = OUT_DIRECTION;
		goingOutModeRoss();
	}

	if (goingCharging)
	{
		WDT_Restart(WDT);
	
		// printFresh("Going charging"); // Write something to the LCD

		float BatVoltage = Voltage.GrabAvg(50); //double check;
		float C = Current.ReadAvg(50);			 //debug;
		//Kehinde 10/30
		startTime = millis(); //start time
		timeDiff = startTime - RobotStartTime;
		writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going Charging", startTime);
		writeSDcard('V', String(noOfTrips) + "," + String(timeDiff) + ",Going Charging," + String(BatVoltage), startTime);
		writeSDcard('C', String(noOfTrips) + "," + String(timeDiff) + ",Going Charging," + String(C), startTime);

		//goingOutModeRoss(); //Kehinde: go out to charging station

		GoingCharging();
	}

	if (chargingMode)
	{
		// float BatVoltage = Voltage.GrabAvg(100); //double check;
		// float C = Current.ReadAvg(100);			 //debug;
		WDT_Restart(WDT);

		// printFresh("Charging mode"); // Write something to the LCD

		//Kehinde 10/30
		// startTime = millis(); //start time
		// timeDiff = startTime - RobotStartTime;
		// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + 
		// 	",Charging (V: " + BatVoltage + " - C: " + C + ")", startTime);

		float BatVoltage = Voltage.GrabAvg(50); //double check;
		float C = Current.ReadAvg(50);
		startTime = millis(); //start time
		timeDiff = startTime - RobotStartTime;
		writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Charging", startTime);
		writeSDcard('V', String(noOfTrips) + "," + String(timeDiff) + ",Charging," + String(BatVoltage), startTime);
		writeSDcard('C', String(noOfTrips) + "," + String(timeDiff) + ",Charging," + String(C), startTime);

		ChargingMode();
	}


	if (restingMode)
	{
		//Kehinde 10/30
		startTime = millis(); //start time
		timeDiff = startTime - RobotStartTime;
		writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Resting", startTime);

		KP = Kp; //reset gain
		RestingMode();
	}

	if (exitTunnelMode)
	{
		//Kehinde 10/30
		startTime = millis(); //start time
		timeDiff = startTime - RobotStartTime;
		writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Exit Tunnel", startTime);

		exitTunnel();
	}

	if (turnReversalMode)
	{
		//Kehinde 10/30
		startTime = millis(); //start time
		timeDiff = startTime - RobotStartTime;
		writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Turn Reversal", startTime);

		TurnHeadingRoss(current_target_heading);
					// TurnHeadingRoss(IN_DIRECTION);

	}

} //end main loop
// ********** END   {MAIN SCRIPT} ----------

// ********** BEGIN (MODE DEFINITION} **********

//----------------------------------------------------
void goingOutModeRoss()
{
	// Serial.println(F("Beginning of GoingInMode()"));
	//writeSDcard('M', "Going Out", millis());

	// //Kehinde 10/30
	startTime = millis(); //start time
	// timeDiff = startTime - RobotStartTime;
	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going Out", startTime);

	WDT_Restart(WDT);
	numOfConsequitiveBackwardKicks = 0;

	// #if ALLOW_USELESS_RUNS
	// unsigned long whenModeStart=millis(); //the robot has X seconds to get to the tunnel or it has to get out
	// #endif

	// unsigned long whenMovedHead=millis();  //initiate timer to myDelay head shaking; //pitch head up and down to make sure that the head sensor triggers if the robot is pushing in a tunnel

	Arm.PitchGo(HIGH_ROW_ANGLE);
	
	int MID_POS_low;

	//reduce pressure on gripper
	if (CheckPayload())
		MID_POS_low = CLOSED_POS+5; // MID_POS-20;
	else
		MID_POS_low = CLOSED_POS;
	
	Arm.PitchGo(HIGH_ROW_ANGLE);
	delay(300);				
	Arm.GripperGo(MID_POS_low); //close gripper again

	delay(500);
	Arm.PitchGo(HIGH_ROW_ANGLE); //raise again
	delay(500);				
	Arm.GripperGo(MID_POS_low); //close the jaws //JSP

	// TurnHeading(current_target_heading); //may need to add this for a case when the board resets
	// add head bump sensor
	// Serial.println(F("goingOutModeRoss is..."));

	while (goingOut)
	{
		WDT_Restart(WDT);

		// Serial.println("In goingIn while-loop");
		// Serial.print("The goingIn cycle-time is: ");
		// Serial.println(millis() - previousTime);
		// previousTime = millis();
		printFresh("Going Out");

		if (millis() - globalTimerDiff > 2000)
		{
			sendPowerUsage("Going Out");
			globalTimerDiff = millis();
		}

		//--- handle wrong way directions
		// Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
		
		if (!isWantedHeading(OUT_DIRECTION))
		{
			Serial.println("Not facing the correct direction");
			Stop();
			enable_turnReversalMode(3);
			current_target_heading = OUT_DIRECTION;
			TurnHeadingRoss(OUT_DIRECTION);
			//whenForcedBackwardKick=millis(); //reset timer to prevent immediate backup
		}	
		
		// Arm.PitchGo(HIGH_ROW_ANGLE);
		
		//Kehinde: check if robot is stalled
		// if (millis() - startTime > 6000) //stationaryThreash
		// {			
		// 	// Arm.PitchGo(HIGH_ROW_ANGLE);
		// 	//delay(300);				
		// 	// Arm.GripperGo(CLOSED_POS); //close gripper again
		// 	// //delay(300);
		// 	// Arm.PitchGo(HIGH_ROW_ANGLE); //raise again
			
		// 	startTime = millis();

			
		// 	if (IsStationary())
		// 	{

		// 		//stationaryThreash+=3000;	

		// 		int l_or_r = random(0, 100);

		// 		if (l_or_r >= 50)
		// 		{
		// 			Stop();
		// 			Backward(BASE_SPEED);
		// 			delay(500);
		// 			Left(DEFAULT_TURNING_SPEED);
		// 			delay(700);
		// 			Stop();				
		// 		}
		// 		else
		// 		{
		// 			Stop();
		// 			Backward(BASE_SPEED);
		// 			delay(500);
		// 			Right(DEFAULT_TURNING_SPEED);
		// 			delay(700);
		// 			Stop();				
		// 		}
		// 		//Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement			
		// 	}
		// }
		//printFresh("Going out"); //report over radio
		

		FollowLane(); //poll camera and call PD
		if(nolane)
			Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
		
		WDT_Restart(WDT);

		// Arm.PitchGo(HIGH_ROW_ANGLE);
		// Arm.GripperGo(MID_POS_low); //JSP
		
		// Checks for Contact -- will be calling followLane() within the contact check loop
		if (CONTACT)
		{
			Stop();

			WDT_Restart(WDT);
			handleContact();

		//writeSDcard('M', "Going Out", millis());
		// //Kehinde 10/30
		// startTime = millis(); //start time
		// timeDiff = startTime - RobotStartTime;
		// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going Out [Contact]", startTime);
	
		}

		// // If we run into the media - copied from front contact
		// if (checkHeadSensor())
		// {
		// 	Backward(BASE_SPEED);
		// 	delay(500);
		// 	Drive.RightForward(255);
		// 	Drive.LeftForward(75);
		// 	delay(500);
		// 	Drive.RightForward(75);
		// 	Drive.LeftForward(255);
		// 	delay(500);
		// }
		
		if (CHARGER)
		{
			Stop();
			Backward(BASE_SPEED);
			delay(50);
			Stop();

			// if (CheckPower()) //check the battery before beginnig new trip
			// {
			// 	enable_GoingCharging();
			// 	return;
			// }
			
			// if (goingCharging) //Kehinde
			// {
			// 	enable_ChargingMode();
			// 	return;
			// }
			// else
			// {
				enable_DumpingMode();
				return;
			//}
		}

		FollowLane(); //poll camera and call 
		if(nolane)
			Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
		
		WDT_Restart(WDT);

		// if(CheckPower()){
		// enable_GoingCharging();
		// return;
		// }
		// Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
			// if (checkHeadSensor())
			// {
			// 	Serial.println("checkHeadSensor() returns true");
			// 	// return; //found soemthing, lets dig
			// }
		FollowLane();
		if(nolane)
			Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
		
		/* 
		#ifdef MANUAL_ON
			handleManualOverride(); 
		#endif
		*/

		// If we find something in the payload on the way out, we want to dump when we get to the end
		if (uselessRun && CheckPayload())
		{
			Serial.println("Something found in payload");
			uselessRun = false;
		}

	} //end while(goingIn)
	return;
} // end goingInMode()
//----------------------------------------------------

//----------------------------------------------------
void GoingInMode()
{
	// Serial.println(F("Beginning of GoingInMode()"));
	//writeSDcard('M', "Going in", millis());
	//Kehinde 10/30
	startTime = millis(); //start time
	// timeDiff = startTime - RobotStartTime;
	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going In", startTime);


	WDT_Restart(WDT);
	numOfConsequitiveBackwardKicks = 0;

#if ALLOW_USELESS_RUNS
	unsigned long whenModeStart = millis(); //the robot has X seconds to get to the tunnel or it has to get out
#endif

	unsigned long whenMovedHead = millis(); //initiate timer to myDelay head shaking; //pitch head up and down to make sure that the head sensor triggers if the robot is pushing in a tunnel

	delay(500);
	Arm.PitchGo(HIGH_ROW_ANGLE); //raise again
	delay(500);				
	Arm.GripperGo(CLOSED_POS); //close the jaws //JSP

	// TurnHeading(current_target_heading); //may need to add this for a case when the board resets
	// add head bump sensor
	Serial.println(F("goingIn is..."));

	// dof.readMag(); //update magnetometer registers
	// float heading=getHeading((float) dof.mx, (float) dof.my);

	unsigned long extraPID = millis();

	while (goingIn)
	{

		printFresh("Going In");

		if (millis() - globalTimerDiff > 2000)
		{
			sendPowerUsage("Going In");
			globalTimerDiff = millis();
		}

		unsigned long loopTime = millis();
		// Serial.println("Not facing the correct direction");

		// while(1){
		// WDT_Restart(WDT);
		// FollowLane(); //poll camera and call PD
		// delay(10);
		// }

		//--- handle wrong way directions
		// unsigned long dirTime = millis();

		// Forward(BASE_SPEED);
		FollowLane(); //poll camera and call PD
		// delay(500);

		if(nolane)
			Forward(BASE_SPEED);

		if (!isWantedHeading(IN_DIRECTION))
		{
			Serial.println("Not facing the correct direction");
			Stop();
			enable_turnReversalMode(1); // more to ensure the modes stay the same than anything else
			TurnHeadingRoss(IN_DIRECTION);
		}

		// //Kehinde: check if robot is stalled
		// if (millis() - startTime > 3000) //stationaryThreash
		// {				
		// 	if (IsStationary())
		// 	{
		// 		startTime = millis();

		// 		//stationaryThreash+=3000;

		// 		int l_or_r = random(0, 100);
		// 		if (l_or_r >= 50)
		// 		{
		// 			Stop();
		// 			Backward(BASE_SPEED);
		// 			delay(500);
		// 			Left(DEFAULT_TURNING_SPEED);
		// 			delay(700);
		// 			Stop();				
		// 		}
		// 		else
		// 		{
		// 			Stop();
		// 			Backward(BASE_SPEED);
		// 			delay(500);
		// 			Right(DEFAULT_TURNING_SPEED);
		// 			delay(700);
		// 			Stop();				
		// 		}
		// 		//Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement			
		// 	}
		// }
		//printFresh("Going in"); //report over radio	

		// Serial.print("dirTime is "); Serial.println(millis() - dirTime);
		FollowLane(); //poll camera and call PD

		WDT_Restart(WDT);
		// Arm.PitchGo(HIGH_ROW_ANGLE);
		// Arm.GripperGo(CLOSED_POS); //JSP

		// unsigned long checkPayTime = millis();
		// Forward(BASE_SPEED);

		if (CheckPayload())
		{
			Serial.println("Something found in payload");
			current_target_heading = OUT_DIRECTION;
			enable_turnReversalMode(3); // this sets the variable nextMode to 3, which corresponds to goingOut. This sets a variable that will remember which mode the robot should enter after turning around...?
			return;						// if there is something in the gripper, then we need to exit goingInMode
		}
		// Serial.print("checkPayTime is "); Serial.println(millis() - checkPayTime);

		// Forward(BASE_SPEED);

		// Checks for Contact
		// unsigned long contactTime = millis();
		if (CONTACT)
		{
			// Serial.println("Something contacted");
			Stop();

			WDT_Restart(WDT);
			handleContact();
		}
		// Serial.print("contactTime is "); Serial.println(millis() - contactTime);

		// extraPID = millis();
		// while(millis()-extraPID < 500){
		// WDT_Restart(WDT);
		// FollowLane(); //poll camera and call PD
		// }

		// delay(100);
		// Forward(BASE_SPEED);
		// unsigned long chargerTime = millis();
		// Forward(BASE_SPEED);
		if (CHARGER)
		{
			Serial.println("Charging if-statement");

			//Serial.println("charger");
			//Serial.println("Charger!!!"); // JSP //BANI
			Stop();
			delay(100);
			//--- copied from chargingMode backing out routine
			unsigned long backingOutStart = millis();
			Backward(BASE_SPEED);
			delay(1000); //force backout for short time
			Stop();
			enable_turnReversalMode(1); // more to ensure the modes stay the same than anything else
			TurnHeadingRoss(IN_DIRECTION);
		}
		// Serial.print("chargerTime is "); Serial.println(millis() - chargerTime);

		FollowLane(); //poll camera and call PD
		// delay(100);
		// Forward(BASE_SPEED);

		// FollowLane();//poll camera and call PD
		WDT_Restart(WDT);

		// if(CheckPower()){
		// enable_GoingCharging();
		// return;
		// }

		// FollowLane(); //poll camera and call PD
		// delay(100);
		// Forward(BASE_SPEED);
		// unsigned long headTime = millis();
		// Forward(BASE_SPEED);
		if (checkHeadSensor())
		{
			Serial.println("checkHeadSensor() returns true");
			enable_DiggingMode();
			return; //found something, lets dig
		}
		// Serial.print("chargerTime is "); Serial.println(millis() - headTime);

		/* 
		#ifdef MANUAL_ON
			handleManualOverride(); 
		#endif
		*/

		FollowLane(); //poll camera and call PD
// delay(100);
// Forward(BASE_SPEED);

#if ALLOW_USELESS_RUNS //prob bugged
		Serial.println("ALLOW_USELESS_RUNS is true");

		if (millis() - whenModeStart > USELESS_RUN_THRESH)
		{
			//bool goBack = rollDiceProb(50); //%chance to roll true used to be 50
			bool goBack = true; // force the robot to go back
			// preferGyro=true;

			if (goBack)
			{
				Backward(BASE_SPEED);
				delay(2000);
				Stop();
				uselessRun = true;
				current_target_heading = OUT_DIRECTION;
				//TurnHeading(current_target_heading);
				enable_turnReversalMode(3);
				return;
			}
			//bool goBack = false;//BANI
			else
			{							  //BANI
				whenModeStart = millis(); //reset timer
				WDT_Restart(WDT);
				//Serial.println("WDT8");
			}
		}
#endif
		Serial.print("---------------loopTime is ");
		Serial.println(millis() - loopTime);

	} //end while(goingIn)
	return;
} // end goingInMode()
//----------------------------------------------------

void DiggingMode()
{
//no manual control, no contact support

	//writeSDcard('M', "Digging", millis());
	//Kehinde 10/30
	 startTime = millis(); //start time
	// timeDiff = startTime - RobotStartTime;
	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Digging", startTime);	

	//Serial.println("Digging!!!");
	Arm.GripperGo(MID_POS); //open gripper up
	// Arm.PitchGo(LOW_ROW_ANGLE);//raise the arm high
	int diggingAttempts = 0;
	int diggingAttemptsTotal = 0; //put a limit on maximum digging attempts;
	#define EXCAVATION_PENALTY 8000
	bool newAttempt = true;

	//Kehinde: check for false positives before diggin
	// delay(500);
	// if (!checkHeadSensor())
	// {
	// 	printFresh("checkHeadSensor() returns false");
	// 	enable_GoingInMode();
	// 	return; //nothing found return
	// }

	while (diggingMode)
	{
		printFresh("Digging");

		if (millis() - globalTimerDiff > 2000)
		{
			sendPowerUsage("Digging");
			globalTimerDiff = millis();
		}

		WDT_Restart(WDT);
		newAttempt = true;
		 //this will be reset if checkHeadSensor comes on
		//if(DUMPING_SWITCH){ //digging in a wrong area. cancel this mode and go back to goingIn
		//Backward(BASE_SPEED);//back up
		//delay(1500); //backing out delay
		//enable_GoingInMode();
		//Arm.PitchGo(HIGH_ROW_ANGLE);
		//Arm.GripperGo(OPEN_POS);
		//TurnHeading(IN_DIRECTION); //reorient toward digging area
		//return;
		//}

		//Handle Contact
		if (CONTACT)
		{			
			Stop();

			handleContact();
			WDT_Restart(WDT);
		// Serial.println("WDT5");
			//writeSDcard('M', "Digging", millis());

			//Kehinde 10/30
			// startTime = millis(); //start time
			// timeDiff = startTime - RobotStartTime;
			// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Digging [Contact]", startTime);

		}

		WDT_Restart(WDT);
		delay(2); //small delay

		if (checkHeadSensor()) //check if we are getting something
		{ 
			newAttempt = false;

			//Back Up from the Cohesive Material to lower the gripper system
			unsigned long BackUpStart = millis();
			unsigned long ContactStart;
			unsigned long ContactDuration;
			while ((millis() - BackUpStart) < 700) //Kehinde: 2000-1200
			{					  //Back up for 2 seconds, if it is interrupted by a contact, deal with contact, and then resume backing up.
				WDT_Restart(WDT); //JSP
				Backward(BASE_SPEED);
				if (CONTACT)
				{
					Stop();

					WDT_Restart(WDT);
					// ContactStart = millis();
					// handleContact();
					// ContactDuration = millis() - ContactStart; //Measure how much time the robot took to respond to contact
					// BackUpStart += ContactDuration;			   //Add time the robot took to respond to contact, and back up more accordingly

					//Kehinde 10/30
					// startTime = millis(); //start time
					// timeDiff = startTime - RobotStartTime;
					// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Digging [Contact]", startTime);

				}
			}
			Stop();
			delay(100);

			Arm.PitchGo(MID_ROW_ANGLE);
			delay(100);			
			Arm.GripperGo(OPEN_POS); //Kehinde
			delay(100);			
			Arm.PitchGo(LOW_ROW_ANGLE);
			delay(100);
			
			//Arm.GripperGo(OPEN_POS);
			Forward(255); //Kehinde: chaned from BASE_SPEED to 255
			delay(300);

			Dig();  //Digging Motion
			Stop(); //back up a little
			
			Arm.GripperGo(MID_POS-20); //close gripper
			delay(300);
			Arm.PitchGo(MID_ROW_ANGLE-20);
			 delay(300);
			Arm.GripperGo(CLOSED_POS); //close gripper
			delay(300);
//Kehinde:
			if(!diggingMode)
			{
				Backward(100);
				delay(500);
				Stop();
				return;
			}

			Backward(100);
			delay(200);
			Stop(); //back up a little
			//Arm.PitchGo(HIGH_ROW_ANGLE);
			//kehinde: raise arm gradually up
			for (int pos = MID_ROW_ANGLE-10; pos <= HIGH_ROW_ANGLE; pos += 10)
			{
				Arm.PitchGo(pos);
				delay(400);
				//Arm.PitchGo(HIGH_ROW_ANGLE);
			}
			
			Arm.PitchGo(HIGH_ROW_ANGLE);
			delay(300);				
			Arm.GripperGo(CLOSED_POS); //close gripper again
			delay(300);
			Arm.PitchGo(HIGH_ROW_ANGLE); //raise again
			delay(200);
			
			Backward(BASE_SPEED);
			delay(500);

			if (CheckPayload())
			{
				Arm.PitchGo(HIGH_ROW_ANGLE);
				delay(300);				
				Arm.GripperGo(CLOSED_POS); //close gripper again
				delay(300);
				Arm.PitchGo(HIGH_ROW_ANGLE);

				enable_exitTunnelMode(); //Go into exit tunnel mode
				return;
			}
			if(!checkHeadSensor())
			{
				enable_GoingInMode(); 
				return; //nothing found return
			}
			else
			{
				Backward(BASE_SPEED);
				delay(400);

				Arm.PitchGo(MID_ROW_ANGLE);
				delay(100);			
				Arm.GripperGo(OPEN_POS); //Kehinde
				delay(100);			
				Arm.PitchGo(LOW_ROW_ANGLE);
				delay(100);
				
				//Arm.GripperGo(OPEN_POS);
				Forward(255); //Kehinde: chaned from BASE_SPEED to 255
				delay(400);

				Dig(); //Digging Motion, JSP
				Stop();

				Arm.GripperGo(MID_POS-20); //close gripper
				delay(300);
				Arm.PitchGo(MID_ROW_ANGLE-20);
				delay(300);
				Arm.GripperGo(CLOSED_POS); //close gripper
				delay(300);
				//Kehinde:
				if(!diggingMode)
				{
					Backward(100);
					delay(500);
					Stop();
					return;
				}
				
				Backward(100);
				delay(200);
				Stop(); //back up a little
				//Arm.PitchGo(HIGH_ROW_ANGLE);
				//kehinde: raise arm gradually up
				for (int pos = MID_ROW_ANGLE-10; pos <= HIGH_ROW_ANGLE; pos += 10)
				{
					Arm.PitchGo(pos);
					delay(400);
					//Arm.PitchGo(HIGH_ROW_ANGLE);
				}

				Arm.PitchGo(HIGH_ROW_ANGLE);
				delay(300);				
				Arm.GripperGo(CLOSED_POS); //close gripper again
				delay(300);
				Arm.PitchGo(HIGH_ROW_ANGLE); //raise again	
				delay(200);
				
				Backward(BASE_SPEED);
				delay(500);

				if (CheckPayload())
				{
					Arm.PitchGo(HIGH_ROW_ANGLE);
					delay(300);				
					Arm.GripperGo(CLOSED_POS); //close gripper again
					delay(300);
					Arm.PitchGo(HIGH_ROW_ANGLE);

					enable_exitTunnelMode(); //could be made in its own mode
					return;
				}

				if(!checkHeadSensor())
				{
					enable_GoingInMode(); 
					return; //nothing found return
				}
			}
		}
		else
		{
			enable_GoingInMode(); 
			return; //nothing found return
		}

		Forward(BASE_SPEED); //Kehinde: changed from BASE_SPEED to 255
		delay(100);
		Stop(); //step in forward
		if (newAttempt)
		{
			WDT_Restart(WDT);
			diggingAttempts++; //increment this number
		}
		diggingAttemptsTotal++;
		if (diggingAttempts > 2 || diggingAttemptsTotal > 4)
		{ //check if digging was unsucessful
			WDT_Restart(WDT);
			enable_GoingInMode();
			Arm.PitchGo(MID_ROW_ANGLE);
			Backward(BASE_SPEED);
			delay(1000);
			Stop();
			int left_or_right = random(0, 100);
			if (left_or_right >= 50)
			{
				Left(DEFAULT_TURNING_SPEED);
			}
			else
			{
				Right(DEFAULT_TURNING_SPEED);
			}
			delay(200);
			Stop();
			Arm.PitchGo(MID_ROW_ANGLE);
			// Arm.GripperGo(CLOSED_POS);
			Arm.GripperGo(OPEN_POS);
			delay(1000);
			return; //if so, exit out
		}
		delay(100);
		Forward(BASE_SPEED); //Kehinde: changed from BASE_SPEED to 255
		delay(100);
		Stop();
	} //end while(diggingMode)
}

void Dig()
{
	WDT_Restart(WDT);

	delay(500);

	if(!isWantedHeading(IN_DIRECTION)) //if not in right orientation
	{
		enable_GoingInMode(); 
		return; //nothing found return
	}


	//TurnHeading(IN_DIRECTION); //reorient toward digging area
	unsigned long digStart = millis();
	Forward(255); 


	//check if cam can see the balls clearly


	// while (!(RUSensor.IsDetected() || LUSensor.IsDetected() || RDSensor.IsDetected() || LDSensor.IsDetected()))
	// while(!checkHeadSensor()) cannot check headsensor cos the gripper is opend
	// {
	// 	if ((millis() - digStart) > 2000) //kehinde: 5000 - 2000
	// 	{					  //for some reason, if the hall effect sensor on gripper doesn't see anything for too long, terminate dig function
	// 		WDT_Restart(WDT); //JSP
	// 		//printFresh("false +ve");
	// 		enable_GoingInMode(); 
	// 		return; //nothing found return
	// 	}
	// 	Forward(255); //BASE_SPEED Go Forward until the hall effect sensor on gripper touches the cohesive material
	// }

	// if (RUSensor.IsDetected())
	// {
	// 	Arm.PitchGo(MID_ROW_ANGLE + 4);
	// }
	// else if (LUSensor.IsDetected())
	// {
	// 	Arm.PitchGo(MID_ROW_ANGLE + 2);
	// }
	// else if (RDSensor.IsDetected())
	// {
	// 	Arm.PitchGo(MID_ROW_ANGLE);
	// }
	// else
	// {
	// 	Arm.PitchGo(MID_ROW_ANGLE - 2);
	// }

	int i = 0; //JSP
	while (i < 3)
	{
		// Arm.GripperGo(OPEN_POS-10);
		Arm.GripperGo(MID_POS); //+20
		Forward(255); //Kehinde BASE_SPEED
		delay(200);
		// Arm.GripperGo(MID_POS+10); //+20
		Arm.GripperGo(OPEN_POS-20);
		Forward(255); //Kehinde
		delay(200);
		i = i + 1;
	} //JSP
}

void exitTunnel()
{
	//writeSDcard('M', "Exit Tunnel", millis());
	//Kehinde 10/30
	startTime = millis(); //start time
	// timeDiff = startTime - RobotStartTime;
	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Exit Tunnel", startTime);

	WDT_Restart(WDT);
	printFresh("Exit tunnel");
	Arm.PitchGo(HIGH_ROW_ANGLE);
	// Backward(255); myDelay(3000);
	// TurnHeading(OUT_DIRECTION); //turn towards dumping area
	// Stop(); myDelay(1000); //debug myDelay
	// enable_GoingOutMode();
	// HeadSensor.threshold=940; //return threshold to its original value
	unsigned long exitStart = millis();		   //grab a time to enable timeout
	unsigned long exitStartTimeout = millis(); //grab a time to enable timeout

	bool inTunnel = true; //am I in the tunnel?, assume that we are to start

	// int MID_POS_low;
	//reduce pressure on gripper
	// if (CheckPayload())
	// 	MID_POS_low = MID_POS-10;
	// else
	// 	MID_POS_low = CLOSED_POS;

	Arm.PitchGo(HIGH_ROW_ANGLE); //raise again
	delay(500);				
	Arm.GripperGo(CLOSED_POS); //close the jaws //JSP
	delay(1000);

	Arm.PitchGo(HIGH_ROW_ANGLE);
	delay(300);				
	Arm.GripperGo(CLOSED_POS); //close gripper again
	delay(300);

	// Backward(BASE_SPEED); //go backward
	// delay(1000);

	while (inTunnel || checkHeadSensor())
	{
		printFresh("Exit Tunnel");

		if (millis() - globalTimerDiff > 2000)
		{
			sendPowerUsage("Exit Tunnel");
			globalTimerDiff = millis();
		}

		//Kehinde: check if it still has payload
		// Arm.GripperGo(MID_POS_low); //close gripper again
		// delay(300);
		// Arm.PitchGo(HIGH_ROW_ANGLE);		
		
		// Arm.PitchGo(HIGH_ROW_ANGLE);
		// Arm.GripperGo(CLOSED_POS); //reduce pressure on gripper
		
		// Arm.PitchGo(HIGH_ROW_ANGLE);
		// delay(300);				
		// Arm.GripperGo(CLOSED_POS); //close gripper again
		// delay(300);
		// Arm.PitchGo(HIGH_ROW_ANGLE); //raise again
			
		Stop();
		if (!CheckPayload())
		{
			enable_GoingInMode();
			delay(300);
			return;			
		}


		// //Kehinde: check if robot is stalled
		// if (millis() - startTime > 3000) //stationaryThreash
		// {	
		// 	if (IsStationary())
		// 	{
		// 		startTime = millis();

		// 		//stationaryThreash+=3000;
				
		// 		int l_or_r = random(0, 100);
		// 		if (l_or_r >= 50)
		// 		{
		// 			Stop();
		// 			Backward(BASE_SPEED);
		// 			delay(500);
		// 			Left(DEFAULT_TURNING_SPEED);
		// 			delay(700);
		// 			Stop();				
		// 		}
		// 		else
		// 		{
		// 			Stop();
		// 			Backward(BASE_SPEED);
		// 			delay(500);
		// 			Right(DEFAULT_TURNING_SPEED);
		// 			delay(700);
		// 			Stop();				
		// 		}
		// 		//Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement			
		// 	}
		// }

		//CLOSED_POS = CLOSED_POS + 20;

		//printFresh("Exit tunnel"); //report over radio
		

		WDT_Restart(WDT);
		// GetDetectedSigs(); //get the colors
	
		// Stop();
		FollowLaneBackward();
		// Backward(BASE_SPEED); //go backward
		// delay(500);

		// Backward(BASE_SPEED); //go backward
		// WDT_Restart(WDT); //JSP
		if ((millis() - exitStart) >= 4000 || (millis() - exitStartTimeout > 10000)) //Kehinde 5000(10000) - 20000
		{ // maximum exitTunnel time of 20 seconds before going into goingOut mode. Tune this
			WDT_Restart(WDT);
			inTunnel = false;
		}

		Stop();
		// FollowLaneBackward();
		// delay(500);
		// Backward(BASE_SPEED); //go backward

		if (CONTACT)
		{
			Stop();
			// Serial.println("Something contacted");
			WDT_Restart(WDT);
			handleContact();
			WDT_Restart(WDT);

			exitStart += 500; //Kehinde: 3500 -> 1500 //add time to compensate
			//writeSDcard('M', "Exit Tunnel", millis());
			// startTime = millis(); //start time
			// timeDiff = startTime - RobotStartTime;
			// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Exit Tunnel [Contact]", startTime);
		}

		// FollowLaneBackward();
	}

	// Backward(255); delay(1000); myDelay(2000); Stop();
	WDT_Restart(WDT); //JSP
	//HeadSensor.threshold=700; //return threshold to its original value. may add a second check //JSP
	current_target_heading = OUT_DIRECTION;

	delay(200);
	enable_turnReversalMode(3); //3 denotes that the robot will go into going_out mode after turn reversal mode
	return;
}

//----------------------------------------------------
void DumpingMode()
{
	//renamed Deposit Mode. dumping run
	WDT_Restart(WDT);

	Arm.GripperGo(CLOSED_POS-10); //close gripper again


	// #endif
	//this mode is allocated for further development of smart media stacking and such
	//added feedback dumping feature 11/10/2014 (dump untill head sensor is cleared)
	if (!uselessRun)
	{
		printFresh("Dumping Mode");
	
		if (millis() - globalTimerDiff > 2000)
		{
			sendPowerUsage("Dumping Mode");
			globalTimerDiff = millis();
		}
		
		DumpPayload(); //drop cotton balls

		// Arm.PitchGo(MID_ROW_ANGLE);
		// delay(200);				
		// Arm.GripperGo(CLOSED_POS); //close gripper again
		
		Arm.GripperGo(CLOSED_POS-10);
		delay(200);
		Arm.PitchGo(HIGH_ROW_ANGLE); //raise again
	
		unsigned long modeTimeout = millis();

		while (analogRead(ForceSensor) >= ForceSensorThresh)
		{ // if there are still materials stuck inside grippers //JSP
			Backward(BASE_SPEED);
			delay(500); //back up
			Stop();
			delay(100);			 //stop;
			Forward(BASE_SPEED); //go forward until dumping switches are compressed
			unsigned long now = millis();

				// while ((millis() - now) < 5000)
				// { //and limit forward dash for 5 seconds
				// 	// if(DUMPING_SIGNAL){ //replace a line below with this "improvmenet"
				// 	// if(DUMPING_SWITCH){
				// 	if (CHARGER)
				// 	{ //
				// 		WDT_Restart(WDT);
				// 		break;
				// 	}
				// }

			WDT_Restart(WDT);
			Stop();
			delay(100);	//stop, we just hit the switches
			DumpPayload(); //drop cotton balls again

			//Backward(255); delay(500); //back out, and check again if there are still cotton balls.
			//bumpDelay(500);
			if ((millis() - modeTimeout) > 15000) //Kehinde: 15000 - 10000
			{
				WDT_Restart(WDT);
				DumpPayload(); //drop cotton balls
				leaveDumpingSite();
				return; //force exit
						// break;
			}
		}
	} //do this untill the robot does not have anything in a jaw
	else
	{
		uselessRun = false; // reset uselessRun to false
		delay(2000);		//surveying++ or something
	}						//surveying completed
	WDT_Restart(WDT);

	Arm.GripperGo(CLOSED_POS);
	delay(200);
	Arm.PitchGo(HIGH_ROW_ANGLE); //raise again
	
	Backward(BASE_SPEED);
	delay(50);
	Stop(); //back out

#if PROBABILITY_DIG
	lorenzMode();
#endif

	if(CHARGER)
	{
		if (CheckPower()) //check the battery before beginnig new trip
		{
			printFresh("Going charging");
			// startTime = millis(); //start time
			// timeDiff = startTime - RobotStartTime;
			// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going In", startTime);

			enable_GoingCharging();
			return;
		}
	}
	else
	{
		Forward(BASE_SPEED);
		delay(100);

		if(CHARGER)
		{
			Stop(); //back out

			if (CheckPower()) //check the battery before beginnig new trip
			{
				printFresh("Going charging");
				// startTime = millis(); //start time
				// timeDiff = startTime - RobotStartTime;
				// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going In", startTime);

				enable_GoingCharging();
				return;
			}
		}
	}


	leaveDumpingSite();
	// enable_GoingInMode();

	// current_target_heading = IN_DIRECTION;
	// delay(500);
	// enable_turnReversalMode(1); // added by ross, since this is called immediately in leaveDumpingSite anyway

	// leaveDumpingSite(); //contains pre compiler directives inside
	return;
}

void leaveDumpingSite()
{
	printFresh("Leaving dumping site");

	WDT_Restart(WDT);
	
	// preferGyro=true;
	//current_target_heading=IN_DIRECTION;
	//TurnHeading(current_target_heading);
	// enable_turnReversalMode(1);
	//enable_GoingInMode();//BANI //turn around to face excavation area
	
	// Arm.PitchGo(MID_ROW_ANGLE);
	// delay(200);				
	Arm.GripperGo(CLOSED_POS); //close gripper again
	delay(100);
	Arm.PitchGo(HIGH_ROW_ANGLE); //raise again
	// Arm.GripperGo(CLOSED_POS);
		
	Backward(BASE_SPEED);
	delay(1000); //back out
	Stop();

	WDT_Restart(WDT);

	// if (CheckPower())
	// {
	// 	enable_GoingCharging();
	// 	return;
	// }

	// unsigned long forcedFollowingStart =  millis();
	// WDT_Restart(WDT);
	// while( (millis() - forcedFollowingStart < 700) ){
	// FollowLane();
	// }
	current_target_heading = IN_DIRECTION;
	enable_turnReversalMode(1);

	TurnHeadingRoss(current_target_heading);	
	// TurnHeading(current_target_heading);
	// enable_turnReversalMode(1);
	enable_GoingInMode(); 

	return;
}

//----------------------------------------------------

void goToDumpingSite()
{
	WDT_Restart(WDT);

	current_target_heading = OUT_DIRECTION;
	unsigned long whenForcedBackwardKick = millis(); //remember last forced backward kick
	unsigned long whenCheckedPayload = millis();	 //used to occasionally check if payload is still there
													
	// bool maskHeadTrigger=false;
	// if(HEADON){
	// maskHeadTrigger=true; //if excavated load happens to block this sensor, disable it
	// }
	while (1)
	{
		WDT_Restart(WDT);
#ifdef MANUAL_ON
		handleManualOverride();
#endif

		//--IR contacts

		//IRactionHandled=handleIRcontacts();
		//checkIfBackwardKickNeeded(&IRactionHandled,&whenIRactionHandled,&whenForcedBackwardKick);
		//timeoutForceBackwardKick(&whenForcedBackwardKick); //force backward kick if the robot has not been backing out in a long time
		FollowLane(); //poll camera and call PD

		//---contact handling
		if (CONTACT)
		{
			Stop();

			WDT_Restart(WDT);
			handleContact();

			// startTime = millis(); //start time
			// timeDiff = startTime - RobotStartTime;
			// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Useless [Contact]", startTime);
	
			FollowLane(); //poll camera and call PD
		}

		if (CHARGER)
		{   //bani
			//if(DUMPING_SWITCH){
			WDT_Restart(WDT);
			Stop();
			return;
			// Stop(); delay(100);
			// //--- copied from chargingMode backing out routine
			// // unsigned long backingOutStart=millis();
			// Backward(BASE_SPEED); bumpDelay(3000); //force backout for short time
			// Stop(); delay(200); //quick stop
			// TurnHeading(OUT_DIRECTION); //turn to go in a tunnel
			// Stop(); delay(100);
			// // myDelay(1000); //debug myDelay;
			// Forward(BASE_SPEED); //start slowly driving forward
			// whenForcedBackwardKick=millis(); //note that robot drove backward
		}
		FollowLane(); //call PD controller

		//--- NEED TO HANDLE THIS CASE
		// if(checkWrongDirections()){
		// WDT_Restart(WDT);
		// //whenForcedBackwardKick=millis(); //reset timer to prevent immediate backup
		// FollowLane();//poll camera and call PD
		// }

		// if (DUMPING_SWITCH) {
		// WDT_Restart(WDT);
		// return;
		// }
	} //end while(goingOut)
}
//--------------------
int seekCharging()
{
	//---
	WDT_Restart(WDT);

	// startTime = millis(); //start time
	// timeDiff = startTime - RobotStartTime;
	
	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Seek Charging", startTime);

	float BatVoltage = Voltage.GrabAvg(50); //double check;
	float C = Current.ReadAvg(50);
	startTime = millis(); //start time
	timeDiff = startTime - RobotStartTime;
	writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Seek Charging", startTime);
	writeSDcard('V', String(noOfTrips) + "," + String(timeDiff) + ",Seek Charging," + String(BatVoltage), startTime);
	writeSDcard('C', String(noOfTrips) + "," + String(timeDiff) + ",Seek Charging," + String(C), startTime);

	unsigned long whenCheckedPayload = millis(); //used to occasionally check if payload is still there

	// current_target_heading = OUT_DIRECTION;

	while (!CHARGER)
	{
		WDT_Restart(WDT);

		printFresh("Seek Charging"); // Write something to the LCD

		if (millis() - globalTimerDiff > 2000)
		{
			sendPowerUsage("Seek Charging");
			globalTimerDiff = millis();
		}
	
		//---contact handling
		// if (CONTACT)
		// {
		// 	WDT_Restart(WDT);
		// 	handleContact();
		// }

		//--- handle wrong way directions
		// Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
		if (!isWantedHeading(OUT_DIRECTION))
		{
			Serial.println("Not facing the correct direction");
			Stop();
			enable_turnReversalMode(3);
			current_target_heading = OUT_DIRECTION;
			TurnHeadingRoss(OUT_DIRECTION);
			//whenForcedBackwardKick=millis(); //reset timer to prevent immediate backup
		}	
		
		
		// //Kehinde: check if robot is stalled
		// if (millis() - startTime > 3000) //stationaryThreash
		// {			
		// 	startTime = millis();
			
		// 	if (IsStationary())
		// 	{
		// 		int l_or_r = random(0, 100);

		// 		if (l_or_r >= 50)
		// 		{
		// 			Stop();
		// 			Backward(BASE_SPEED);
		// 			delay(500);
		// 			Left(DEFAULT_TURNING_SPEED);
		// 			delay(700);
		// 			Stop();				
		// 		}
		// 		else
		// 		{
		// 			Stop();
		// 			Backward(BASE_SPEED);
		// 			delay(500);
		// 			Right(DEFAULT_TURNING_SPEED);
		// 			delay(700);
		// 			Stop();				
		// 		}
		// 		//Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement			
		// 	}
		// }
		
		FollowLane(); //poll camera and call 
		if(nolane)
			Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
				
		WDT_Restart(WDT);

		Arm.PitchGo(HIGH_ROW_ANGLE);
		Arm.GripperGo(CLOSED_POS); //JSP
		
		// Checks for Contact -- will be calling followLane() within the contact check loop
		if (CONTACT)
		{
			Stop();

			WDT_Restart(WDT);
			handleContact();
		}

	FollowLane(); //poll camera and call 
		if(nolane)
			Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
				// FollowLane(); //poll camera and call PD
		
		WDT_Restart(WDT);

		// if(CheckPower()){
		// enable_GoingCharging();
		// return;
		// }
		// Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
		// FollowLane();
	
	} //end while(1)

	Stop();
	return 1;

}

//---

void leaveChargingStation()
{
	printFresh("Leave Charging"); // Write something to the LCD
	//drive back and prepare for GoingIn
	WDT_Restart(WDT);
	Backward(BASE_SPEED);
	delay(1000);
	WDT_Restart(WDT);
	delay(3000);
	Stop();
	delay(50);
	Forward(BASE_SPEED);
	delay(200); //nudge forward a bit in case you hit a wall or something
	Left(DEFAULT_TURNING_SPEED);
	WDT_Restart(WDT);
	delay(1000); //force overshoot
	current_target_heading = IN_DIRECTION;
	//TurnHeading(current_target_heading);
	enable_turnReversalMode(1);
	//enable_GoingInMode();
	//GetDetectedSigs();
	// if(CHARGING_TRAIL){
	// }
}

void GoingCharging()
{
	WDT_Restart(WDT);

	printFresh("Going Charging"); // Write something to the LCD
	
	if (millis() - globalTimerDiff > 2000)
	{
		sendPowerUsage("Going Charging");
		globalTimerDiff = millis();
	}
	
	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Going Charging", startTime);

	numOfConsequitiveBackwardKicks = 0;

	int seekch=seekCharging();
	if (seekch)
	{
		enable_ChargingMode();
		return;
	}

} //end going charging

//----------------------------------------------------
void ChargingMode()
{
//NOT GOING TO WORK. WE CUT THE CHARGING + wire
	WDT_Restart(WDT);

	printFresh("Charging mode"); // Write something to the LCD	
	if (millis() - globalTimerDiff > 2000)
	{
		sendPowerUsage("Charging mode");
		globalTimerDiff = millis();
	}
	

	//Serial.println("Entered Charging Mode");
	Stop();
	// delay(1000);
	//note: delay is used here instead of myDelay
	//need to add a little statement that checks if the voltage is actually increasing
	// WDT_Restart(WDT);
	Relay.PowerOff();
	delay(2000);
	
	//need to add manual stuff. need to be able to turn relay on and off
	float C;  //variable for storing current
	C=Current.ReadAvg(100); //debug

	float BatVoltage = Voltage.GrabAvg(100);
	float previousMaxVoltage = BatVoltage;

	unsigned long startCharge = millis();
	unsigned long whenIncreasedVoltage = millis();

	while (BatVoltage < CHARGED_VOLTAGE)
	{
		WDT_Restart(WDT);
		printFresh("V: " + String(BatVoltage)+" | C: " + String(C) + " | T: "+ String((millis() - startCharge)/1000)); // Write something to the LCD
		
		if((millis() - startCharge)%600000 <= 1500) //sample every 10min
			writeSDcard('W', String(noOfTrips) + ",Charging," + String(millis() - startCharge) + 
			"," + String(BatVoltage) + "," + String(C), millis());

		delay(1000);
		C=Current.ReadAvg(100); //debug
		BatVoltage = Voltage.GrabAvg(100); //read current voltage
		if ((BatVoltage - previousMaxVoltage)>=0.01)
		{
			printFresh("Increment: " + String(BatVoltage-previousMaxVoltage) + " | T: " + String((millis() - whenIncreasedVoltage)/1000));
			delay(1000);
			whenIncreasedVoltage = millis(); //reset timer
			previousMaxVoltage = BatVoltage; //record new high
		}

		WDT_Restart(WDT);

		if ((millis() - whenIncreasedVoltage) > 1800000)
		{ //5 minutes
			printFresh("No voltage increase in 30min: " + String(BatVoltage));
			delay(1000);
			WDT_Restart(WDT);
			break; //havent seen voltage increase in a while
		}

		if (!CHARGER)
		{ //not touching charging station
			WDT_Restart(WDT);
			Relay.PowerOn();
			delay(1000); //turn power back on
			// checkCamera();
			printFresh("seek charge station");

			int chr = seekCharging();
			Relay.PowerOff();
			// bool redock_success = redock(); //if success, should we reset voltage tracking variables?
			// if (!redock_success)
			// {
			// 	// enable_GoingCharging(); //need to find charging beacon again
			// 	// return; //exit this mode
			// 	seekCharging(); //find charging station again
			// 	Relay.PowerOff();
			// }
		}

		if( C > -0.1 && C <=0.0 && BatVoltage >3.9) //( C > -0.099 && C <=0)
		{ //good enough
			printFresh("forced Charging exit: ");
			break; //exit this loop
		}

		delay(1000); //small waiting delay
	}

	printFresh("Charging mode top up"); // Write something to the LCD	
	if (millis() - globalTimerDiff > 2000)
	{
		sendPowerUsage("Charging mode");
		globalTimerDiff = millis();
	}
	
	//now the voltage level has jumped up, but we need to add juce
	unsigned long chargingStart = millis();
	//                                m  s   ms
	unsigned long desiredChargingTime = 1800000; //Kehinde:10mins 9000000;  //2.5hours of charging
	while (millis() - chargingStart < desiredChargingTime)
	{
		BatVoltage = Voltage.GrabAvg(100);
		C=Current.ReadAvg(100);

		printFresh("Extra charging: " + String(BatVoltage) + " | " + String(C));

		if((millis() - startCharge)%600000 <= 1500) //sample every 10min
			writeSDcard('W', String(noOfTrips) + ",Charging," + String(millis() - chargingStart) + 
			"," + String(BatVoltage) + "," + String(C), millis());

		WDT_Restart(WDT);
		delay(1000); //do nothing
		
		if (!CHARGER)
		{ //not touching charging station
			Relay.PowerOn();
			delay(1000); //turn power back on
			// printFresh("Redocking charge station");

			printFresh("seek charge station");
			
			int chr = seekCharging();
			Relay.PowerOff();
			desiredChargingTime += 5 * 60 * 1000; //add some time to compensate
			// bool redock_success = redock();
			// WDT_Restart(WDT);
			// if (!redock_success)
			// {
			// 	// enable_GoingCharging(); //need to find charging beacon again
			// 	// return; //exit this mode
			// 	seekCharging(); //find charging station again
			// 	Relay.PowerOff();
			// 	desiredChargingTime += 5 * 60 * 1000; //add some time to compensate
			// }
		}
		
		// if( C > -0.1 && C <=0.0 && BatVoltage >3.9) //( C > -0.099 && C <=0)
		// { //good enough
		// 	printFresh("forced Charging exit: ");
		// 	break; //exit this loop
		// }
	}

	Relay.PowerOn(); //turn the power to the robot on
	delay(1000);
	WDT_Restart(WDT);

	// WDT_Restart(WDT);
	//checkCamera();
	// checkIMU();
	// checkCamera();

		// leaveChargingStation();
		// Forward(BASE_SPEED); //start slowly driving forward, anti stuck kick

	// isNewTrip = false;
	//Kehinde 10/30
	// startTime = millis(); //start time
	// timeDiff = startTime - RobotStartTime;
	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) +
	// 		",Done Charging (V: " + BatVoltage + " - C: "+ C + ")", startTime);

	Backward(BASE_SPEED);
	delay(1000);

	current_target_heading = IN_DIRECTION;
	// TurnHeading(current_target_heading);
	enable_turnReversalMode(1);
	 enable_GoingInMode(); 
	 //go back to digging
	return;
}

void RestingMode()
{
	//writeSDcard('M', "Resting", millis());
	// startTime = millis(); //start time
	// timeDiff = startTime - RobotStartTime;
	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Resting", startTime);

	numOfConsequitiveBackwardKicks = 0;
	WDT_Restart(WDT);
//this mode is exclusive to probabilistic digging
#if PROBABILITY_DIG

	printFresh("Resting");

	int code = seekCharging(); //get to the charging station
	if (code == 2)
	{
		return;
	}
	Stop(); //hit the breaks
#if !ALLOW_CHARGING_ON_REST
	Backward(BASE_SPEED); //back up so that we are not charging
	while (CHARGER || DUMPING_SWITCH)
	{
		//do nothing. Add timeout here too
	}
	delay(200);
	Stop();
#endif
	WDT_Restart(WDT);
	//--roll a dice here if we want more digging
	if (!rollDice())
	{
		printFresh("Rolled dig");

		leaveChargingStation();
		return;
	}
	else
	{
		WDT_Restart(WDT);

		printFresh("Rolled rest");


#if ALLOW_POWER_SAVINGS
		Relay.PowerOff();
#endif
#if ALLOW_CHARGING_ON_REST
		Relay.PowerOff();
#endif
	}

	//--- at this point we are charging if not backed out

	bool resting = true;				   //flow control
	unsigned long restingStart = millis(); //timer to keep the robot waiting/resting
	// probability digging
	while (resting)
	{
		printFresh("Resting");

		if (millis() - globalTimerDiff > 2000)
		{
			sendPowerUsage("Resting");
			globalTimerDiff = millis();
		}
		WDT_Restart(WDT);
		if (millis() - restingStart > RESTING_TIME)
		{ //determine if its time to roll a dice
			if (!rollDice())
			{
				printFresh("Rolled dig");

				Relay.PowerOn(); //can be packaged with #if
				leaveChargingStation();
				return;
			}
			else
			{
				WDT_Restart(WDT);
				restingStart = millis(); //reset timer, and wait for another roll
				printFresh("Rolled rest"); //maybe report a different byte for consecutive rests
			}
		}

#if ALLOW_CHARGING_ON_REST
		//re-dock if charging is allowed, found, and lost

		if (!CHARGER)
		{ //bani

			//if(!CHARGER || !DUMPING_SWITCH){ //not touching charging station, redock, may need to break out of here earlier?
			WDT_Restart(WDT);
			Relay.PowerOn();
			delay(1000); //turn power back on
			//this can be probably replaced with "go forward for a while, if not, do stuff below
			printFresh("Redocking charge station");
			bool redock_success = redock(); //if success, should we reset voltage tracking variables?
			if (!redock_success)
			{

				code = seekCharging(); //find charging station again
				if (code == 2)
				{
					return;
				} //
				Relay.PowerOff();
			}
		}
#endif

#if !ALLOW_CHARGING_ON_REST //case where charging is NOT RFF

		if (CHARGER)
		{ //bani
			//if(CHARGER || DUMPING_SWITCH){//bani
			WDT_Restart(WDT);
			Relay.PowerOn();
			delay(1000); //turn power back on
			Backward(BASE_SPEED);
			delay(500); //back out, we dont want to be touching the charging station
			Stop();
#if ALLOW_POWER_SAVINGS
			Relay.PowerOff();
			delay(500); //turn power back off
#endif
		}
#endif

		if (CheckPower())
		{
			WDT_Restart(WDT);
			enable_GoingCharging();
			return;
		}
	}		//end of while //bani removed
	return; //bani
// #else//bani

// seekCharging();// get to the charger//bani removed this else snippet as it screws with the rest of the code
// delay(10000); //10 seconds//bani
// leaveDumpingSite();//bani
// return;//bani

#endif

	goToDumpingSite();
	delay(RESTING_TIME); //wait
	leaveDumpingSite();

	//enable_GoingInMode;//BANI
	//return;//BANI...TO MOVE IT OUT OF MONOTONOUS LOOP
	// #ifdef ALLOW_USELESS_RUNS //and not Probability dig
	// goToDumpingSite();
	// leaveDumpingSite();
	// #endif
}

// ********** END   {MODE DEFINITION} ----------

//// ********** BEGIN (SUPPORT METHODS} **********

void lorenzMode()
{
	//writeSDcard('M', "Lorenz", millis());
	startTime = millis(); //start time
	timeDiff = startTime - RobotStartTime;
	writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Lorenz", startTime);

	// Arm.PitchGo(HIGH_ROW_ANGLE);
	// Arm.GripperGo(CLOSED_POS); //JSP

	Relay.PowerOff();
	printFresh("Resting");

	bool resting = true;				   //flow control
	unsigned long restingStart = millis(); //timer to keep the robot waiting/resting

	// probability digging
	while (resting)
	{
		if (millis() - globalTimerDiff > 2000)
		{
			sendPowerUsage("Lorenz");
			globalTimerDiff = millis();
		}
		WDT_Restart(WDT);
		if (millis() - restingStart > RESTING_TIME)
		{ //determine if its time to roll a dice
			if (rollDiceProb(lorenzProb))
			{ // time to dig
				printFresh("Rolled dig");

				wakeUp(); //can be packaged with #if
				current_target_heading = IN_DIRECTION;
				enable_GoingInMode();
				// leaveChargingStation();
				return;
			}
			else
			{ // keep resting
				WDT_Restart(WDT);
				restingStart = millis(); //reset timer, and wait for another roll
				printFresh("Rolled rest"); //maybe report a different byte for consecutive rests
			}
		}
	} //end of while
	current_target_heading = IN_DIRECTION;
	enable_GoingInMode(); // not sure we will ever get here, but need to ensure a next state exists
	return;
} //end setup loop

void wakeUp()
{

	pullResetPinLow(); //pull reset pin low
	// MotorBoard Drive; //sets up motor drive, calling class MotorBoard to create an object "Drive"

	Relay.PowerOn();

	initiateChargingDetector(); //set up contact charger detector
	delay(1000);

	CapSensor.setup();
	delay(1000);

	FSensor.setup();
	delay(1000);

	pixy.init(); //Starts I2C communication with a camera
	delay(1000);

	WDT_Restart(WDT);
	uint16_t status = dof.begin(); //starts I2C communication with a IMU. status variable should have a hex value of 0x49D4. This should def be called, has calibration functions inside
	dof.setMagScale(dof.M_SCALE_2GS);
	dof.setMagODR(dof.M_ODR_125); //sets up magnetometers output data rate to the highest (fastest) available setting

	Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
	Serial.println(status, HEX);
	Serial.println("Should be 0x49D4");
	Serial.println();

	delay(1000);
	WDT_Restart(WDT);
	/* power electronics */

	delay(1000); // time to allow the comms to be established
	return;
}

//----------------------------------------------------
bool checkHeadSensor()
{

	WDT_Restart(WDT);
	if (FSensor.Detected())
	{			  //am I detecting something in front of me with my head sensor? Is it a false positive?
				  //goingIn condition is necessary because it could have been changed by contact switches
		return 1; //JSP
	}

	else
	{
		return 0; //nothing is detected
	}
}

#define DIR_CHECK_TIME 500 //2500

/**
checkWrongDirections() is a method that takes in no parameters, and returns a boolean value of 0 or 1.

0 is returned if current_target_heading is either IN_DIRECTION or OUT_DIRECTION and
the robot is facing the wrong direction and dirCheckFlag is false. A side result is that dirCheckFlag is set to true,

1 is returned if IN_DIRECTION or OUT_DIRECTION and dirCheckFlag is true

Important parameters in this method:
dirCheckFlag
dirCheckTimer

Issues with this method:

I would not be surprise if this method will likely cause a silent error. isWantedHeading has a tolerance, and you need to make sure
that all of your direction headings are at least this tolerance apart.


bool checkWrongDirections(){
	 // checks if going in a wrong directions
	// adjusts if needed and returns 1 
	WDT_Restart(WDT);
	Serial.println("checkWrongDirections");
	if(current_target_heading == IN_DIRECTION){
		// if( isWantedHeading(OUT_DIRECTION) || isWantedHeading(PORT_DIRECTION) || isWantedHeading(STARBOARD_DIRECTION)  ){ //wrong way
		if(!isWantedHeading(IN_DIRECTION) ){ //wrong way
			if(!dirCheckFlag){ //flag is not set
				dirCheckFlag=true;
				dirCheckTimer=millis(); //start timer 
				// Serial.println("flag on");
				return 0; 
			}
			if(dirCheckFlag){
				if( millis() - dirCheckTimer > DIR_CHECK_TIME ){
					// Serial.println("turn");
					// Stop();
					// delay(100); //hit the breaks
					//TurnHeading(IN_DIRECTION);
					// if(goingIn){
						// enable_turnReversalMode(1);
					// }
					// GetDetectedSigs(); //poll camera, get latest vision info
					dirCheckFlag=false;
					return 1;	 
				}
				
				else{ // you cannot ever get here
					// Serial.println("tick tock");
					return 0;
				}		
			}	 
		}
		// Serial.println("flag reset");
		dirCheckFlag=false;	
 } //ends IN_DIRECTION
 
 
 //---
	if(current_target_heading == OUT_DIRECTION){
		// if( isWantedHeading(IN_DIRECTION) || isWantedHeading(PORT_DIRECTION) || isWantedHeading(STARBOARD_DIRECTION)  ){ //wrong way
		if(!isWantedHeading(OUT_DIRECTION)){ //wrong way
			
			
			if(!dirCheckFlag){ //flag is not set
				dirCheckFlag=true;
				dirCheckTimer=millis(); //start timer 
				// Serial.println("flag on");
				return 0; 
			}
			
			
			if(dirCheckFlag){
				if( millis() - dirCheckTimer > DIR_CHECK_TIME ){
					//Serial.println("turn");
					// Stop();
					// delay(100); //hit the breaks
					//TurnHeading(OUT_DIRECTION);
					// enable_turnReversalMode(3);
					// GetDetectedSigs(); //poll camera, get latest vision info
					dirCheckFlag=false;
					return 1;	 
				}
				
				else{ // you cannot ever get to this else statement
					// Serial.println("tick tock");
					return 0;
				}		
			}	 
		}
		// Serial.println("flag reset");
		dirCheckFlag=false;	
	} //ends OUT_DIRECTION
	return 0;
}

**/

void turnIMUon()
{
	pinMode(IMUpower, OUTPUT);	//put the pin in low impeadence
	digitalWrite(IMUpower, HIGH); //turn the pullup resistor on to source voltage
}

void turnIMUoff()
{
	// pinMode(IMUpower,OUTPUT);
	digitalWrite(IMUpower, LOW);
}

// void checkIMU(){
// //commented function calls out. Looks like it does more bad than good
// WDT_Restart(WDT);
// /* this method checks if there is a connection with IMU. If not,
// this method will make the robot attempt to re-establish connection */
// //Rewrite this. have a variable in the IMU class which starts a timer if status is bad. then if the !=0x49D4 is seen for a long time, mash reset button. No more instantaneous crap
// uint16_t IMUstatus = dof.checkStatus(); //write to whoAmI register and see if there is response
// if( IMUstatus == 0x49D4){
// return; //no problem, exit this method
// }
// Stop(); //hold on a second... something is not right with IMU
// // char powerCycles=0;
// while(IMUstatus != 0x49D4){

// unsigned long statusTroubleStart=millis(); //grab time when this reading started
// while( millis() - statusTroubleStart < 2000){//wait and hope that this problem is resolved
// IMUstatus = dof.checkStatus(); //check status again
// if(IMUstatus == 0x49D4){
// Forward(BASE_SPEED); //anti stuck kick
// WDT_Restart(WDT);
// return; //problem solved
// }
// delay(100);
// }
// turnIMUoff(); //turn the pin off
// delay(3000);
// delay(3000);

// while(1){
// //watchdog timer, do your thing
// }
// delay(3000);
// delay(3000);
// delay(3000); */

// // arduinoReset(); //situation is hopeless. Mash reset button
// // Relay.PowerOff(); //turn the power off
// // delay(1000); //wait a little
// // Wire.endTransmission();    // stop transmitting, force the bus to relax
// // delay(3000);
// // Relay.PowerOn(); //turn the power back on
// // delay(1000);
// // IMUstatus = dof.begin(); // attempt rebooting
// // delay(1000);
// // powerCycles++; //record attempt
// }
// return;
// }

//----------------------------------------------------
//----------------------------------------------------
void DumpPayload()
{
	/* this robot will make the robot drop payload and shake its head once to make sure that GM is not stuck to the gripper */
	// Serial.print(goingIn); Serial.print(diggingMode); Serial.print(goingOut); Serial.print(dumpingMode); Serial.print(goingCharging); Serial.println(chargingMode); //debug

	int dumpAngle = MID_ROW_ANGLE - 20;

	while (CheckPayload())  //; //do
	{
		WDT_Restart(WDT);
		Stop();
		delay(1000); //stop Note: delay is used (not myDelay). we dont want to react to bumps
		Arm.PitchGo(dumpAngle);
		delay(100);
		Arm.GripperGo(OPEN_POS-20);
		delay(500); //straighten the arm out and release grip
		WDT_Restart(WDT);
		Arm.PitchGo(dumpAngle + 10);
		delay(200);
		Arm.PitchGo(dumpAngle - 5);
		delay(200); //shake the arm
		WDT_Restart(WDT);
		Arm.PitchGo(dumpAngle + 10);
		delay(200); //shake the arm
		Arm.PitchGo(dumpAngle - 5);
		delay(200); //shake the arm
		WDT_Restart(WDT);
		Arm.PitchGo(dumpAngle + 10);
		delay(200);
		Arm.PitchGo(dumpAngle - 5);
		delay(200); //shake the arm
		WDT_Restart(WDT);
		Arm.PitchGo(dumpAngle + 10);
		delay(200); //shake the arm
		Arm.PitchGo(dumpAngle - 5);
		delay(200); //shake the arm
		WDT_Restart(WDT);
		Arm.PitchGo(dumpAngle + 10);
		delay(200);
		Arm.PitchGo(dumpAngle - 5);
		delay(200); //shake the arm
		WDT_Restart(WDT);
		Arm.PitchGo(dumpAngle + 10);
		delay(200); //shake the arm
		Arm.PitchGo(dumpAngle);
		delay(200); //shake the arm
		WDT_Restart(WDT);
		Arm.GripperGo(CLOSED_POS);

	}
	// while (CheckPayload());
	// DumpPayload(); // in case the cohesive material got stuck inside gripper, do DumpPayload again
	// }
	//Right(255); myDelay(2000); //make a u-turn. THIS SHOULD LATER BE REPLACED WITH A WHILE LOOP with camera feedback
	//Forward(100); myDelay(200); //start slowly driving forward, camera guiding algorithm should kick in
}

//----------------------------------------------------
bool CheckPower()
{
	/* this method enables robot to decide whether or not it should go charge itself  */
	printFresh("Checking Power");
	WDT_Restart(WDT);

	// delay(1000);

	float BatVoltage = Voltage.GrabAvg(50); //double check;
	float C = Current.ReadAvg(50);			 //debug;

	checkPowerCount=0;

	for(int p=0; p<CHECK_POWER_CNT_THRESH; p++)
	{
		// Serial.print("C=  "); Serial.println(C); //debug
		// Serial.print("V=  ");Serial.print(BatVoltage); Serial.print('\t');//debug
		WDT_Restart(WDT);
		if (BatVoltage <= LOWEST_ALLOWABLE_VOLTAGE)
		{
			// if (checkPowerCount > CHECK_POWER_CNT_THRESH)
			// {
			checkPowerCount++;
			// 	return 1;
			// }
			// else
			// {
			// 	checkPowerCount++;
			// 	return 0;
			// }
		}
		// else
		// {
		// 	checkPowerCount = 0;
		// 	// return 0;
		// }	

		delay(50);
		
		BatVoltage = Voltage.GrabAvg(50);
		C = Current.ReadAvg(50);	
		// WDT_Restart(WDT);
	}

	printFresh("Check Power: "+String(checkPowerCount) + " | V: "+String(BatVoltage) +" | C: "+String(C) );
	delay(1000);

	if (checkPowerCount >= 3)
	{
			// checkPowerCount = 0;
		return 1;
	}
	else
	{
			// checkPowerCount = 0;
		return 0;
	}
	
}

/** this method will make the robot turn to a given angle bearing using
		magnetometer  **/
void TurnHeadingRoss(float desired_heading)
{
	//writeSDcard('M', "Turn heading", millis());
	
	//Kehinde
	 startTime = millis(); //start time
	// timeDiff = startTime - RobotStartTime;
	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Turn Heading", startTime);


	Serial.println("In TurnHeadingRoss()");

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TurnHeading Setup Process Start ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	WDT_Restart(WDT);

	printFresh("Turn heading"); // Write something to the LCD

	unsigned long watchdog2Timer = millis(); // 2 minute time out timer, exit out of here if the robot fails to turn in 2 minutes

	int instructionChanges = 0; //initiate variable to keep track of how many

	// Ross: is the following line still necessary? interrupt_mask_timer is only updated, but the value is not used
	// bool turning = true; //local flag to keep turning, really not needed anymore
	unsigned long interrupt_mask_timer = millis(); //used to make sure switches interrupts are not serviced too fast

	// dof.readMag(); //update magnetometer registers
	float current_heading = getHeading(); //get a compass direction, value returned is from 0 to 360 degrees
	
	// printFresh("heading: "+String(current_heading)); // Write something to the LCD

	//autonomously pick the best direction to initiate turning
	turn_reversal_direction = pickDirection(current_heading, desired_heading); //choose direction for turn reversal, 0 for left turn, 1 for right turn
	unsigned long time_start = millis();									   // initiate timer to keep robot turning if there is a turning progress
	unsigned long action_timeout = millis();								   // watchdog timer, make sure that the robot is not stuck, forcing new actions if the robot have not done anything in a while
	Arm.PitchGo(HIGH_ROW_ANGLE);											   // raise the arm to decrease overall length // JSP
	WDT_Restart(WDT);

	float diff_heading = desired_heading - current_heading;
	Serial.print("Initial diff_heading = ");
	Serial.println(diff_heading);

	unsigned long switchTurnDirection = millis();
	unsigned long switchTime = 10000; //7000
	
	printFresh("Turn heading");

	// unsigned long currentTime = millis();
	Stop();
	Arm.GripperGo(CLOSED_POS); //close gripper again
	delay(300);
	Arm.PitchGo(HIGH_ROW_ANGLE);

	do
	{
		printFresh("Turn heading");

		if (millis() - globalTimerDiff > 2000)
		{
			sendPowerUsage("Turn Heading");
			globalTimerDiff = millis();
		}


		if (dof.checkStatus() != 0x49D4 || millis() - time_start > 22500)
		{
			Stop();

			// Relay.PowerOff();
			delay(1000);
			// wakeUp();
			time_start = millis();
		}

		// update heading differences
		// dof.readMag(); //update magnetometer registers
		current_heading = getHeading();
		turn_reversal_direction = pickDirection(current_heading, desired_heading); //choose direction for turn reversal, 0 for left turn, 1 for right turn

		// Serial.println("---------------------------------------------------------");
		// Serial.print("desired_heading = "); 	Serial.println(desired_heading);
		// Serial.print("current_heading = "); 	Serial.println(current_heading);
		// turn_reversal_direction = pickDirection(current_heading, desired_heading); // pickDirection does not work for the current iteration of IMUs.

		// move a tiny bit in the correct direction
		if (turn_reversal_direction)
		{
			// Serial.println("Turning right");
			printFresh("Turning right");

			// Drive.LeftForward(255);
			// Drive.RightForward(50); //50 -
			// delay(1000);
			Right(DEFAULT_TURNING_SPEED); //Kehinde.BASE_SPEED
			delay(1000); //Kehinde: 180 degrees turn
			Stop();
			contactSide = 1; //right
		}
		else
		{
			printFresh("Turning left");
			// Drive.RightForward(255);
			// Drive.LeftForward(50);
			// delay(1000);
			Left(DEFAULT_TURNING_SPEED); //Kehinde.			
			delay(1000); //Kehinde: 180 degrees turn
			Stop();
			contactSide = 2; // left
		}

		// Checks for Contact
		if (CONTACT)
		{
			Stop();
			// Serial.println("---------------------------------------------------------Something contacted");
			WDT_Restart(WDT);
			handleContact();
			
			//writeSDcard('M', "Turn heading", millis());
			// startTime = millis(); //start time
			// timeDiff = startTime - RobotStartTime;
			// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Turn Heading [Contact]", startTime);

			// switchTurnDirection = millis();
		}


		if (CHARGER)  // || checkHeadSensor()
		{
			Serial.println("Charging if-statement");

			Stop();
			delay(100);

			Backward(BASE_SPEED);
			delay(1000); //force backout for short time
			Stop();
			WDT_Restart(WDT);
		}

		if ((millis() - switchTurnDirection) > switchTime)
		{
			// Stop();

			// Stop();
			// delay(3000);
			Serial.println("Switching Directions");

			Drive.LeftBackward(255); //255 BASE_SPEED
			Drive.RightForward(255);
			delay(1000); //Kehinde: 1000 - 400
			Stop(); // I dont like the way this is being handled

			// WDT_Restart(WDT); // Reset the WDT
			switchTurnDirection = millis(); // update the switchTurnDirection timer
			turn_reversal_direction = !turn_reversal_direction;
		}

		
		// //Kehinde: check if robot is stalled
		// if (millis() - startTime > 6000) //stationaryThreash
		// {	
		// 	if (IsStationary())
		// 	{
		// 		startTime = millis();

		// 		//stationaryThreash+=3000;
				
		// 		int l_or_r = random(0, 100);
		// 		if (l_or_r >= 50)
		// 		{
		// 			Stop();
		// 			Backward(BASE_SPEED);
		// 			delay(500);
		// 			Left(DEFAULT_TURNING_SPEED);
		// 			delay(700);
		// 			Stop();				
		// 		}
		// 		else
		// 		{
		// 			Stop();
		// 			Backward(BASE_SPEED);
		// 			delay(500);
		// 			Right(DEFAULT_TURNING_SPEED);
		// 			delay(700);
		// 			Stop();				
		// 		}
		// 		//Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement			
		// 	}
		// }

		//printFresh("Turn heading");
		

		// Stop();
		delay(300);

	}
	while (!isWantedHeading(desired_heading));
	Stop();
	// delay(3000);

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TurnHeading Setup Process End ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	// while(!isWantedHeading(desired_heading)){ // replace diff_heading > 0 with isWantedHeading()?
	// Serial.println("---------------------------------------------------------");
	// Serial.print("desired_heading = "); 	Serial.println(desired_heading);
	// Serial.print("current_heading = "); 	Serial.println(current_heading);
	// Serial.print("diff_heading = "); 	Serial.println(diff_heading);

	// delay(50); Stop();
	// Serial.println("Turn complete");

	// // update heading differences
	// dof.readMag(); //update magnetometer registers
	// current_heading = getHeading((float) dof.mx, (float) dof.my);
	// diff_heading = desired_heading - current_heading;

	// WDT_Restart(WDT);
	// // delay(10);
	// }

	Serial.println("Exiting TurnHeadingRoss");

	//Determines the next mode in which the robot will go in, and set it accordingly
	switch (nextMode)
	{
	case 1:
		enable_GoingInMode(); //1 denotes the robot will go into going_in mode after turn reversal
		break;

	case 2:
		enable_DiggingMode(); //2 denotes the robot will go into digging mode after turn reversal //never used
		break;

	case 3:
		enable_GoingOutMode(); //3 denotes the robot will go into going_out mode after turn reversal
		break;

	case 4:
		enable_DumpingMode(); //4 denotes the robot will go into dumping mode after turn reversal //never used
		break;

	case 5:
		enable_GoingCharging(); //5 denotes the robot will go into going_charging mode after turn reversal
		break;

	case 6:
		enable_RestingMode(); //6 denotes the robot will go into resting mode after turn reversal //never used
		break;

	case 7:
		enable_exitTunnelMode(); //7 denotes the robot will go into exit tunnel mode after turn reversal //never used
		break;

	case 10:
		//does not change the mode
		Serial.println(F("Case 10 used - mode not updated"));
		break;

	default:
		//does not change the mode
		Serial.println(F("Default used - mode not updated"));
		break;
	}

	return;
}

//----------------------------------------------------
bool isWantedHeading(float desired_heading)
{
	/* this method accepts a desired heading angle and returns a boolean variable
	whether or not the robot is facing in desired angle, give or take 
	a pre-defined tolerance (+- DIRECTION_UNCERTAINTY). This method grabs a latest magnetometer data
	for comparison purposes
	IT IS ASSUMED THAT DIRECTION UNCERTAINTY IS SMALL, meaning cant have low bound <0 and high bound >360 at the same time
	tested with fake data and it works. 11/5/2014
	*/
	//get heading
	WDT_Restart(WDT);
	// dof.readMag(); //update magnetometer registers
	float heading = getHeading();

	// printFresh("Is wanted: "+String(desired_heading));
	// delay(1500);
	// printFresh("Get heading: "+String(heading));
	// delay(500);

	// float heading=getHeading((float) dof.calcMag(dof.mx), (float) dof.calcMag(dof.my));

	float lowBound = desired_heading - DIRECTION_UNCERTAINTY;
	float highBound = desired_heading + DIRECTION_UNCERTAINTY;

	if (lowBound >= 0 && highBound <= 360)
	{ //case without wrap-around singularities
		if (heading >= lowBound && heading <= highBound)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	if (highBound > 360)
	{ //such as desired 355+- 10 degrees
		if (heading >= lowBound && heading <= 360)
		{
			return true;
		}
		if (heading >= 0 && heading <= highBound - 360)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	if (lowBound < 0)
	{ //such as 5+-10 degrees
		if (heading >= 0 && heading <= highBound)
		{
			return true;
		}
		if (heading - 360 >= lowBound && heading - 360 <= 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	WDT_Restart(WDT);
}

int pickDirection(float current_heading, float desired_heading)
{
	/* this method will autonomously pick the best direction to turn based 
	on angular distance */

	// choose direction for turn reversal
	// 0 for left turn, 1 for right turn
	// flipped original 0's to 1's

	WDT_Restart(WDT);
	int turning_case = -1; // initialize to nonsense value
	// all of these are computed assuming that the heading values increase as the robot rotates clockwise.
	// Flip the value at the end if this is not true.

	if (current_heading < 180)
	{
		if (desired_heading < 180)
		{
			if (desired_heading < current_heading)
			{
				turning_case = 0; // turn left
								  // return turning_case;
			}
			else
			{
				turning_case = 1; // turn right
								  // return turning_case;
			}
		}

		// if desired_heading is greater than 180
		if (desired_heading > 180)
		{
			if (desired_heading > (180 + current_heading))
			{
				turning_case = 0; // turn left
								  // return turning_case;
			}
			else
			{
				turning_case = 1; // turn right
								  // return turning_case;
			}
		}
	}

	if (current_heading > 180)
	{
		if (desired_heading < 180)
		{ //check this <
			if ((current_heading - desired_heading) > 180)
			{
				turning_case = 1; // turn right

				// return turning_case;
			}
			else
			{
				turning_case = 0; // turn left
								  // turning_case = 2; // turn left

				// return turning_case;
			}
		}

		if (desired_heading > 180)
		{
			if (desired_heading > current_heading)
			{
				turning_case = 1;
				// turning_case = 3; // right

				// return turning_case;
			}
			else
			{
				turning_case = 0;
				// return turning_case;
			}
		}
	}

	// flip the directions if a CCW rotation of the robot increases the heading value
	if (PHD && turning_case == 0)
	{
		turning_case = 1;
	}

	else if (PHD && turning_case == 1)
	{
		turning_case = 0;
	}

	return turning_case;
}

//----------------------------------------------------
float getHeading()
{
	/* this method grabs magnetometer data from gyroscope and returns a bearing angle
the bearing angle goes from 0 to 360 degrees and wraps around back to 0
call this method like so
 dof.readMag(); //update gyro registers
 int heading=getHeading((float) dof.mx, (float) dof.my);

note from the original library:
 [this method] only works if the sensor is flat (z-axis normal to Earth).
 Additionally, you may need to add or subtract a declination
 angle to get the heading normalized to your location.
 See: http://www.ngdc.noaa.gov/geomag/declination.shtml
  */
	float heading;
	dof.readMag(); //update gyro registers

	// // Get the raw readings from the IMU sensor
	float hx = dof.calcMag(dof.mx);
	float hy = dof.calcMag(dof.my);

	// Serial.print(hx); Serial.print(", "); Serial.println(hy);

	// Map them to a unit circle using a measured bias unique to each IMU
	hx = (hx - HX_MIN) * (2 / (HX_MAX - HX_MIN)) - 1;
	hy = (hy - HY_MIN) * (2 / (HY_MAX - HY_MIN)) - 1;

	if (hy > 0)
	{
		heading = 270 - (atan(hx / hy) * (180 / PI));
	}
	else if (hy < 0)
	{
		// heading = - (atan(hx / hy) * (180 / PI));
		heading = 90 - (atan(hx / hy) * (180 / PI));
	}
	else // hy = 0
	{
		if (hx < 0)
			heading = 360;
		else
			heading = 180;
	}

	// Serial.println(heading); //print capacitive sensor value for only one pin, in this case, pin 0. Change the number to choose other pins.
	// printFresh("heading: " + String(heading));
	// delay(200);

	return heading;
}

//----------------------------------------------------
// float getGyroZ(){
// //pass in gyro bias and substract it
// // return filtered gyro
// //maybe even saturate to zero
// }
//----------------------------------------------------

void handleContact()
{

	// WDT_Restart(WDT);
	//this is the simplified, "back out" method

	// Serial.println("Something contacted");
	// writeSDcard('M', "Contact", millis());
	// writeSDcard('N', String(switchState), millis());

	// startTime = millis(); //start time
	// timeDiff = startTime - RobotStartTime;
	// writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Contact", startTime);

	// float BatVoltage = Voltage.GrabAvg(50); //double check;
	// float C = Current.ReadAvg(50);
	startTime = millis(); //start time
	timeDiff = startTime - RobotStartTime;
	writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Contact", startTime);
	// writeSDcard('V', String(noOfTrips) + "," + String(timeDiff) + ",Contact," + String(BatVoltage), startTime);
	// writeSDcard('C', String(noOfTrips) + "," + String(timeDiff) + ",Contact," + String(C), startTime);

	printFresh("Handle Contact");

	if (millis() - globalTimerDiff > 2000)
	{
		sendPowerUsage("Handle Contact");
		globalTimerDiff = millis();
	}

	unsigned long start_of_contact = millis(); //record length of contact
	// logContacts(start_of_contact); //stick this before every return

	/*
	//if disable flag is set a long time ago
	if(disableContacts){
		if( millis()- whenDisabledContacts >= CONTACT_RESET_TIME){
			disableContacts=false; //reset the flag
		}
		else if (millis()- whenDisabledContacts < CONTACT_RESET_TIME){ //if set recently, take no action, switches are disabled 
			return;//exit and do nothing
		}
	}
	*/

	//Recording Contact Information to Arduino Fio
	//if(goingIn || diggingMode){ //bani THIS LOOP IS FOR SENDING BYTES TO FIO FOR LOGGING THE CONTACTS...added digging mode temporarily
	RFC = switchState & FR_ANT;
	LFC = switchState & FL_ANT;
	RSBC = switchState & RSB_ANT;
	RSFC = switchState & RSF_ANT;
	LSBC = switchState & LSB_ANT;
	LSFC = switchState & LSF_ANT;
	LBC = switchState & BL_ANT;
	RBC = switchState & BR_ANT;
	FS = RFC | LFC;
	RS = RSBC | RSFC;
	LS = LSBC | LSFC;
	BS = LBC | RBC;

	if (FS != 0b0000000000000000)
	{
		if ((FS & SWITCH_ANT_MASK) != 0b0000000000000000)
		{ //is the contact ant or wall?
// the contact is ant
			printFresh("Front side - Ant");
// writeSDcard('N', "Front side - Ant", millis());
noOfAntContact +=1;
writeSDcard('N', String(noOfTrips) + "," + String(timeDiff) + ",Front side,Ant,"+String(noOfAntContact)+","+String(switchState), startTime);
		}

		else if ((FS & SWITCH_ANT_MASK) == 0b0000000000000000)
		{
// the contact is wall
			printFresh("Front side - Wall");
// writeSDcard('N', "Front side - Wall", millis());

noOfWallContact+=1;
writeSDcard('N', String(noOfTrips) + "," + String(timeDiff) + ",Front side,Wall,"+String(noOfWallContact)+","+String(switchState), startTime);
		}
	}

	if (RS != 0b0000000000000000)
	{
		if ((RS & SWITCH_ANT_MASK) != 0b0000000000000000)
		{ //is the contact ant or wall?
// the contact is ant
			printFresh("Right side - Ant");
// writeSDcard('N', "Right side - Ant", millis());
noOfAntContact+=1;
writeSDcard('N', String(noOfTrips) + "," + String(timeDiff) + ",Right side,Ant,"+String(noOfAntContact)+","+String(switchState), startTime);

}

		else
		{
// the contact is wall
			printFresh("Right side - Wall");
// writeSDcard('N', "Right side - Wall", millis());
noOfWallContact+=1;
writeSDcard('N', String(noOfTrips) + "," + String(timeDiff) + ",Right side,Wall,"+String(noOfWallContact)+","+String(switchState), startTime);
		}
	}

	if (LS != 0b0000000000000000)
	{
		if ((LS & SWITCH_ANT_MASK) != 0b0000000000000000)
		{ //is the contact ant or wall?
// the contact is ant
			printFresh("Left side - Ant");
// writeSDcard('N', "Left side - Ant", millis());
noOfAntContact+=1;
writeSDcard('N', String(noOfTrips) + "," + String(timeDiff) + ",Left side,Ant,"+String(noOfAntContact)+","+String(switchState), startTime);

		}
		else
		{
// the contact is wall
			printFresh("Left side - Wall");
// writeSDcard('N', "Left side - Wall", millis());
noOfWallContact+=1;
writeSDcard('N', String(noOfTrips) + "," + String(timeDiff) + ",Left side,Wall,"+String(noOfWallContact)+","+String(switchState), startTime);

}
	}

	if (BS != 0b0000000000000000)
	{
		if ((BS & SWITCH_ANT_MASK) != 0b0000000000000000)
		{ //is the contact ant or wall?
// the contact is ant
			printFresh("Back side - Ant");
// writeSDcard('N', "Back side - Ant", millis());
noOfAntContact +=1;
writeSDcard('N', String(noOfTrips) + "," + String(timeDiff) + ",Back side,Ant,"+String(noOfAntContact)+","+String(switchState), startTime);
		}
		else
		{
// the contact is wall
			printFresh("Back side - Wall");
// writeSDcard('N', "Back side - Wall", millis());
noOfWallContact +=1;
writeSDcard('N', String(noOfTrips) + "," + String(timeDiff) + ",Back side,Wall,"+String(noOfWallContact)+","+String(switchState), startTime);

		}
	}
	//}

	//------------------------------

	//Contact Response
	if (goingIn || goingOut || goingCharging)
	{ //if roaming around
		switch (switchState & SWITCH_WALL_MASK)
		{
		case FL:
			//Serial.println("FL");

			Backward(BASE_SPEED);
			delay(1000);
			Drive.LeftForward(255);
			Drive.RightForward(75);
			delay(500);
			//Drive.LeftForward(75);
			//Drive.RightForward(255);
			//delay(500);

			break;
		case FR:
			//Serial.println("FR");
			Backward(BASE_SPEED);
			delay(1000);
			Drive.RightForward(255);
			Drive.LeftForward(75);
			delay(500);
			//Drive.RightForward(75);
			//Drive.LeftForward(255);
			//delay(500);

			break;
		case (FL | FR):
			//Serial.println("Front Both");
			Backward(BASE_SPEED);
			delay(1000);
			Drive.RightForward(255);
			Drive.LeftForward(75);
			delay(600);

			Forward(BASE_SPEED);
			delay(500);
			Drive.RightForward(75);
			Drive.LeftForward(255);
			delay(600);

			break;

		case RSF: //Right side is hit
		case RSB:
		case (RSF | RSB):
			Backward(BASE_SPEED);
			delay(1000);

			Drive.RightForward(255);
			Drive.LeftForward(75);
			delay(500); // added a delay so this actually does something
			// if(goingIn){
			// #ifdef FIO_LINK
			// #endif
			// }
			break;

		case LSF: //Left side is hit
		case LSB:
		case (LSF | LSB):
			Backward(BASE_SPEED);
			delay(1000);

			Drive.RightForward(75);
			Drive.LeftForward(255);
			delay(500); // added a delay so this actually does something
			// if(goingIn){
			// #ifdef FIO_LINK
			// #endif
			// }
			break;

		case (FL | FR | BL | BR): //robot is squished from both sides
		case (FR | BL | BR):
		case (FL | BL | BR):
		case (FL | FR | BR):
		case (FL | FR | BL):
		case (FL | BL):
		case (FL | BR):
		case (FR | BL):
		case (FR | BR):
		case (RSF | RSB | LSF | LSB):
		case (RSF | LSF):
		case (RSB | LSB):
		case (RSF | RSB | LSF):
		case (RSF | RSB | LSB):
		case (LSF | LSB | RSF):
		case (LSF | LSB | RSB):
			// case
			Stop();

			Backward(BASE_SPEED);
			delay(1000);
			Drive.RightForward(255);
			Drive.LeftForward(75);
			delay(500);

			Forward(BASE_SPEED);
			delay(500);
			Drive.RightForward(75);
			Drive.LeftForward(255);
			delay(500);

			// delay(1000);
			WDT_Restart(WDT);
			break;

		case BR: //robot is bumped on the back
		case BL:
		case (BR | BL):
			Forward(255); //kick forward
			delay(1000);				  //kick delay
			WDT_Restart(WDT);
			// if(goingIn){
			// #ifdef FIO_LINK
			// #endif
			// }
			break;

		case (FL | LSF | LSB):
		case (FL | LSF | BL):
		case (FL | LSB | BL):
		case (LSF | LSB | BL):
		case (FL | LSF | LSB | BL):
			//Serial.println("FL");

			Backward(BASE_SPEED);
			delay(1000);

			Drive.LeftForward(255);
			Drive.RightForward(75);
			delay(500);
			//Drive.LeftForward(75);
			//Drive.RightForward(255);
			//delay(500);

			break;

		case (FR | RSF | RSB):
		case (FR | RSF | BR):
		case (FR | RSB | BR):
		case (RSF | RSB | BR):
		case (FR | RSF | RSB | BR):
			//Serial.println("FL");

			Backward(BASE_SPEED);
			delay(1000);

			Drive.RightForward(255);
			Drive.LeftForward(75);
			delay(500);
			//Drive.LeftForward(75);
			//Drive.RightForward(255);
			//delay(500);

			break;

		default:
			Backward(BASE_SPEED);
			delay(1000);
			Stop();
			break;
		}
		// logContacts(start_of_contact);
		return;
	}
	//------------------------------
	else if (diggingMode)
	{
		int currentDriveState = lastDriveState; //grab current state
		switch (switchState & SWITCH_WALL_MASK)
		{
		case BR: //back
				 // IMPLEMENT SOMETHING LIKE THIS
				 // //Serial.println("FL");

		// Backward(BASE_SPEED);
		// delay(500);
		// Drive.LeftForward(255);
		// Drive.RightForward(75);
		// delay(500);
		// //Drive.LeftForward(75);
		// //Drive.RightForward(255);
		// //delay(500);

		// break;
		case BL:
		// IMPLEMENT SOMETHING LIKE THIS
		// //Serial.println("FL");

		// Backward(BASE_SPEED);
		// delay(500);
		// Drive.LeftForward(255);
		// Drive.RightForward(75);
		// delay(500);
		// //Drive.LeftForward(75);
		// //Drive.RightForward(255);
		// //delay(500);
		case (BR | BL):
		case (BR | RSB): //including back side
		case (BL | LSB):
		case (BR | BL | RSB | LSB):
			WDT_Restart(WDT);
			// Stop();
			// delay(100); //DIGGING_INTERRUPT_DELAY //this will affect lastDriveState, thats why we use another variable
			break;
			//switch(currentDriveState){
			//case drivingForward:
			//Forward(BASE_SPEED); //can add speed memory as well
			//break;
			//case drivingBackward:
			//Backward(BASE_SPEED);
			//break;
			//case turningRight:
			//Right(BASE_SPEED);
			//break;
			//case turningLeft:
			//Left(BASE_SPEED);
			//break;
			//case stopped:
			//Stop();
			//break;
			//}
		}
		// logContacts(start_of_contact);
		return;
	}
	//------------------------------
	else if (dumpingMode)
	{
		// do nothing, actually its not even called
		// logContacts(start_of_contact);
		return;
	}
	else if (chargingMode)
	{
		// int currentDriveState=lastDriveState; //grab current state
		// switch(switchState){
		// // case RB: //back
		// // case LB:
		// // case (RB | LB):
		// // case (RB | RSB): //including back side
		// // case (LB | LSB):
		// // case (RB | LB | RSB | LSB):
		// default :
		// Stop(); delay(CHARGINGMODE_INTERRUPT_DELAY); //this will affect lastDriveState, thats why we use another variable
		// switch(currentDriveState){
		// // case drivingForward:
		// // Forward(200); //can add speed memory as well
		// // break;
		// case drivingBackward:
		// Backward(200);
		// break;

		// case stopped:
		// Stop();
		// break;
		// }
		// break;
		// }
		// logContacts(start_of_contact);
		return;
	}

	else if (exitTunnelMode)
	{
		switch (switchState & SWITCH_WALL_MASK)
		{
		case FL:
			Stop();
			delay(100);
			Backward(255);			
			delay(1000);		
						  //Kehinde 100 -200
			Right(255); //Kehinde: Changed from left to right DEFAULT_TURNING_SPEED
			delay(500);					  //Kehinde 100 -300
			
			Stop();
			break;
		case FR:
			Stop();
			delay(100);
			Backward(255);			
			delay(1000);

			Left(255); //Kehinde: Changed from right to left
			delay(500);

			Stop();
			break;
		case BR: //something hit back
			Stop();
			delay(100);
			Forward(BASE_SPEED);
			delay(1000); //Kehinde 100 -300
			
			Stop();
			Drive.RightBackward(BASE_SPEED);
			Drive.LeftBackward(50);
			// Right(DEFAULT_TURNING_SPEED);
			delay(500);
			Stop();
			// delay(100);
			// Backward(BASE_SPEED);
			// delay(500); //force new action
			break;
		case BL:
			Stop();
			delay(100);
			Forward(BASE_SPEED);
			delay(1000);
			
			Stop();
			Drive.LeftBackward(BASE_SPEED);
			Drive.RightBackward(50);
			// Right(DEFAULT_TURNING_SPEED);
			delay(500);
			// Right(DEFAULT_TURNING_SPEED);
			// delay(700);
			Stop();
			// delay(100);
			// Backward(BASE_SPEED);
			// delay(500); //force new action
			break;
		case BR | BL:
			Stop();
			delay(100);
			Forward(BASE_SPEED);
			delay(1000);
			Stop();

			int l_or_r = random(0, 100);
			if (l_or_r >= 50)
				Left(DEFAULT_TURNING_SPEED);
			else	
				Right(DEFAULT_TURNING_SPEED);
			
			delay(1000);
			Stop();							
			// Backward(BASE_SPEED);
			// delay(500); //force new action
			break;
		}
	}

	else if (turnReversalMode)
	{
		if (!turn_reversal_direction) //left
		{ //0 for left, 1 for right
			//initial turning direction is left
			switch (switchState & SWITCH_WALL_MASK)
			{

			case FL:
			case FR:
			case (FL | FR):
				Backward(BASE_SPEED);
				delay(1000);

				Left(255); //no space, turn on the spot
				// Drive.RightBackward(75);
				// Drive.LeftBackward(255);
				delay(500);
				Stop();
				break;

			case RSF: //Right side is hit
			case RSB:
			case (RSF | RSB): // Ross 10/26 added in behaviour here. Previously this contact was ignored
			case (FR | RSF | RSB):
			case (FR | RSF | BR):
			case (FR | RSB | BR):
			case (RSF | RSB | BR):
			case (FR | RSF | RSB | BR):
				//Serial.println("FL");

				Backward(BASE_SPEED);
				delay(1000);

				Left(255); //no space, turn on the spot

				// Drive.RightForward(255);
				// Drive.LeftForward(75);
				delay(500);
				//Drive.LeftForward(75);
				//Drive.RightForward(255);
				//delay(500);
				Stop();
				break;

			case LSF: //Left side is hit
			case LSB:
			case (LSF | LSB):
			case (FL | LSF | LSB):
			case (FL | LSF | BL):
			case (FL | LSB | BL):
			case (LSF | LSB | BL):
			case (FL | LSF | LSB | BL):
				//Serial.println("FL");

				Backward(BASE_SPEED);
				delay(1000);

				Right(255); //no space, turn on the spot

				// Drive.LeftForward(255);
				// Drive.RightForward(75);
				
				delay(1000);
				//Drive.LeftForward(75);
				//Drive.RightForward(255);
				//delay(500);
				Stop();
				break;

			case (FL | FR | BL | BR): //robot is squished from both sides
			case (FR | BL | BR):
			case (FL | BL | BR):
			case (FL | FR | BR):
			case (FL | FR | BL):
			case (FL | BL):
			case (FL | BR):
			case (FR | BL):
			case (FR | BR):
			case (RSF | RSB | LSF | LSB):
			case (RSF | LSF):
			case (RSB | LSB):
			case (RSF | RSB | LSF):
			case (RSF | RSB | LSB):
			case (LSF | LSB | RSF):
			case (LSF | LSB | RSB):
				// case
				Stop();
				delay(1000);
				WDT_Restart(WDT);
				break;

			case BR: //robot is bumped on the back
			case BL:
			case (BR | BL):
				Forward(255); //kick forward
				delay(1000);   //kick delay

				Left(255); //no space, turn on the spot

				// Drive.RightForward(255);
				// Drive.LeftForward(75);
				delay(500);
				Stop();

				break;

			default:
				//do nothing
				break;
			}
		}

		else //right
		{
			//initial turning direction is right
			switch (switchState & SWITCH_WALL_MASK)
			{

			case FL:
			case FR:
			case (FL | FR):
				Backward(255);
				delay(1000);

				Right(255); //no space, turn on the spot
				// Drive.RightBackward(75);
				// Drive.LeftBackward(255);
				delay(500);
				Stop();

				// Drive.RightBackward(255);
				//Drive.LeftBackward(75);
				// delay(500);
				break;

			case RSF: //Right side is hit
			case RSB:
			case (RSF | RSB): // Ross 10/26 added in behaviour here. Previously this contact was ignored
			case (FR | RSF | RSB):
			case (FR | RSF | BR):
			case (FR | RSB | BR):
			case (RSF | RSB | BR):
			case (FR | RSF | RSB | BR):
				//Serial.println("FL");

				Backward(255);
				delay(1000);

				Left(255); //no space, turn on the spot
				// Drive.RightBackward(75);
				// Drive.LeftBackward(255);
				delay(1000);
				Stop();

				// Drive.RightForward(255);
				// Drive.LeftForward(75);
				// delay(500);
				//Drive.LeftForward(75);
				//Drive.RightForward(255);
				//delay(500);

				break;

			case LSF: //Left side is hit
			case LSB:
			case (LSF | LSB):
			case (FL | LSF | LSB):
			case (FL | LSF | BL):
			case (FL | LSB | BL):
			case (LSF | LSB | BL):
			case (FL | LSF | LSB | BL):
				//Serial.println("FL");
				// printFresh("Front side - Wall"); // added to debug

				Backward(255);
				delay(1000);
				
				Right(255); //no space, turn on the spot
				// Drive.RightBackward(75);
				// Drive.LeftBackward(255);
				delay(500);
				Stop();

				// Drive.LeftForward(255);
				// Drive.RightForward(75);
				// delay(500);
				//Drive.LeftForward(75);
				//Drive.RightForward(255);
				//delay(500);

				break;

			case (FL | FR | BL | BR): //robot is squished from both sides
			case (FR | BL | BR):
			case (FL | BL | BR):
			case (FL | FR | BR):
			case (FL | FR | BL):
			case (FL | BL):
			case (FL | BR):
			case (FR | BL):
			case (FR | BR):
			case (RSF | RSB | LSF | LSB):
			case (RSF | LSF):
			case (RSB | LSB):
			case (RSF | RSB | LSF):
			case (RSF | RSB | LSB):
			case (LSF | LSB | RSF):
			case (LSF | LSB | RSB):
				// case
				Stop();
				delay(1000);
				WDT_Restart(WDT);
				break;

			case BR: //robot is bumped on the back
			case BL:
			case (BR | BL):
				Forward(255); //kick forward
				delay(1000);   //kick delay
				
				Right(255); //no space, turn on the spot
				// Drive.RightBackward(75);
				// Drive.LeftBackward(255);
				delay(500);
				Stop();

				//Drive.RightForward(75);
				// Drive.LeftForward(255);
				// delay(500);

				break;

			default:
				Backward(BASE_SPEED);
				delay(1000);
				Stop();
				break;
			}
		}
	}
	// logContacts(start_of_contact);
	switchState = 0; //reset switchState
	return;			 //default return
}

// int handleTurningContact(int turning_case){ // not used anywhere
// /* this method will change current turning_case turning direction,
// log the start of the contact, */

// // Serial.print("case \t");
// // Serial.print(turning_case);
// // Serial.print("switches \t");
// // Serial.println(switchState);

// /* case 0: initially started with left  turn
// case 1: initially started with right turn
// case 3: simple backing out routine

// case 4: back left
// case 5: front right
// case 6: back right
// case 7: front left

// delay() should be used here instead of myDelay to avoid program getting stuck
// */

// //if left is pressed,  move 5
// //if right is pressed, move 7

// //can embed anti jamming kick in a if(!Headon), but it must not be masked

// unsigned long start_of_contact=millis(); //record length of contact
// /* // logContacts(start_of_contact); //stick this before every return */

// if(disableContacts){//if disable flag is set.
// if( millis()- whenDisabledContacts >= CONTACT_RESET_TIME){
// disableContacts=false; //reset the flag
// }
// else if (millis()- whenDisabledContacts < CONTACT_RESET_TIME)  {
// return turning_case;//exit and do nothing
// }
// WDT_Restart(WDT);
// }

// switch(turning_case){
// case 0:
// case 1:
// switch(switchState){
// case LSF: //if bumped anywhere on a left side
// case LSB:
// case(LSF | LSB):
// turning_case=5;
// logContacts(start_of_contact);
// // return turning_case;
// break;

// case RSF: //if bumped anywhere on a right side
// case RSB:
// case(RSF | RSB):
// turning_case=7;
// logContacts(start_of_contact);
// // return turning_case;
// break;

// default:
// logContacts(start_of_contact);
// // return turning_case;
// break;
// }
// break;

// case 5:
// switch(switchState){
// case LSF: //if bumped anywhere on a left side
// case LSB:
// case(LSF | LSB):
// turning_case=5;
// logContacts(start_of_contact);
// // return turning_case; //do nothing
// break;

// case RSF: //if bumped anywhere on a right side
// case RSB:
// case(RSF | RSB):
// turning_case=7;
// logContacts(start_of_contact);
// // return turning_case;
// break;

// case (BR | BL):
// turning_case=5;
// WDT_Restart(WDT);
// Forward(255); delay(300); //anti-jamming kick
// Stop(); delay(50);
// logContacts(start_of_contact);
// // return turning_case;
// break;

// default:
// logContacts(start_of_contact);
// // return turning_case;
// break;
// }
// break;

// case 7:
// switch(switchState){
// case LSF: //if bumped anywhere on a left side
// case LSB:
// case(LSF | LSB):
// turning_case=5; //mistake, i think, shouldnt be 5
// logContacts(start_of_contact);
// // return turning_case;
// break;

// case RSF: //if bumped anywhere on a right side
// case RSB:
// case(RSF | RSB):
// turning_case=7;
// logContacts(start_of_contact);
// // return turning_case; //do nothing
// break;

// case (BR | BL):
// turning_case=7;
// Forward(255); delay(300); //anti-jamming kick
// Stop(); delay(50);
// logContacts(start_of_contact);
// // return turning_case;
// break;

// default:
// // Stop(); delay(200);
// logContacts(start_of_contact);
// // return turning_case;
// break;
// }
// break;
// //-----

// case 4:
// switch(switchState){
// case LSF: //if bumped anywhere on a left side
// case LSB:
// case(LSF | LSB):
// turning_case=5;
// logContacts(start_of_contact);
// // return turning_case;
// break;

// case RSF: //if bumped anywhere on a right side
// case RSB:
// case(RSF | RSB):
// turning_case=6;
// logContacts(start_of_contact);
// // return turning_case;
// break;

// case BR:
// case BL:
// case (BR | BL):
// turning_case=5;
// Forward(255); delay(300); //anti-jamming kick
// WDT_Restart(WDT);
// Stop(); delay(50);
// logContacts(start_of_contact);
// // return turning_case;
// break;

// default:
// logContacts(start_of_contact);
// // return turning_case;
// break;
// }
// break;

// case 6:
// switch(switchState){
// case LSF: //if bumped anywhere on a left side
// case LSB:
// case(LSF | LSB):
// turning_case=4;
// logContacts(start_of_contact);
// // return turning_case;
// break;

// case RSF: //if bumped anywhere on a right side
// case RSB:
// case(RSF | RSB):
// turning_case=7;
// logContacts(start_of_contact);
// // return turning_case;
// break;

// case BR:
// case BL:
// case (BR | BL):
// turning_case=7;
// WDT_Restart(WDT);
// Forward(255); delay(300); //anti-jamming kick
// Stop(); delay(50);
// logContacts(start_of_contact);
// // return turning_case;
// break;

// default:
// logContacts(start_of_contact);
// // return turning_case;
// break;
// }
// break;
// }
// return turning_case;//exit
// }

// ********** END   {SUPPORT METHODS} ----------

// ********** BEGIN {INTERRUPT ROUTINES} **********
// ********** END   {INTERRUPT ROUTINES} ----------

// ********** BEGIN {TEST AND DEBUG} **********
//#ifdef TEST_MODE
void TestDriveMotors()
{
	// captures the program here and repeats
	while (1)
	{

		WDT_Restart(WDT);

		// FollowLaneBackward();
		Serial.println(F("Forward"));
		Forward(BASE_SPEED);
		delay(2000);
		Stop();
		delay(1000);

		WDT_Restart(WDT);
		Serial.println(F("Backward"));
		Backward(BASE_SPEED);
		delay(2000);
		Stop();
		delay(1000);

		WDT_Restart(WDT);
		Serial.println(F("Right"));
		Right(DEFAULT_TURNING_SPEED);
		delay(2000);
		Stop();
		delay(1000);

		WDT_Restart(WDT);
		Serial.println(F("Left"));
		Left(DEFAULT_TURNING_SPEED);
		delay(2000);
		Stop();
		delay(1000);
	}
}

void TestPDController()
{
	while (1)
	{
		WDT_Restart(WDT);
		FollowLane();
		// GetDetectedSigs(); //poll camera
		// if(TRAIL1){
		// Input=x1; //grab input
		// PD.Compute(); //updates Rout
		// Serial.print("X-Coor: ");
		// Serial.print(Input);

		// Serial.print("             Area: ");
		// Serial.print(Area1);

		// Serial.print("             Output: \t");
		// Serial.println(Output);
		// delay(1000);
		// }
	}
}

/**

This method will run the robot servos through a pattern while the motors are contantly on. Becuase the current draw from the
motors and the servos is rather high when stalled, if the servo range of motion is too large then they will stall causing the motor
rotation rate to drop. This allows you to make an observation to determine the appropraite range of servo values.

**/
void TestServoMotors()
{
	//DiggingMode();

	Forward(BASE_SPEED);
	while (1)
	{
		Arm.PitchGo(LOW_ROW_ANGLE);
		delay(1000);

		WDT_Restart(WDT);
		Serial.println(F("Opening gripper"));
		printFresh(String(Current.ReadAvg(CURRENT_SAMPLE_SIZE)) + " | "+ String(Voltage.GrabAvg(20)));

		Arm.GripperGo(OPEN_POS);
		delay(1000);

		Serial.println(F("Closing gripper"));
		//printFresh(String(Current.ReadAvg(CURRENT_SAMPLE_SIZE)));
		printFresh(String(Current.ReadAvg(CURRENT_SAMPLE_SIZE)) + " | "+ String(Voltage.GrabAvg(20)));	

		Arm.GripperGo(MID_POS);
		delay(1000);
		Arm.GripperGo(CLOSED_POS);
		delay(1000);
		Serial.println(F("Raising arm"));
		//printFresh(String(Current.ReadAvg(CURRENT_SAMPLE_SIZE)));
		printFresh(String(Current.ReadAvg(CURRENT_SAMPLE_SIZE)) + " | "+ String(Voltage.GrabAvg(20)));		

		WDT_Restart(WDT);
		//kehinde: raise arm gradually up
		for (int pos = LOW_ROW_ANGLE + 10; pos <= HIGH_ROW_ANGLE; pos += 10)
		{
			Arm.PitchGo(pos);
			delay(300);
			//Arm.PitchGo(HIGH_ROW_ANGLE);
		}
		//Arm.PitchGo(HIGH_ROW_ANGLE);
		delay(1500);
		Serial.println(F("Lowering arm"));
		//printFresh(String(Current.ReadAvg(CURRENT_SAMPLE_SIZE)));
		printFresh(String(Current.ReadAvg(CURRENT_SAMPLE_SIZE)) + " - "+ String(Voltage.GrabAvg(20)));		

		Arm.PitchGo(LOW_ROW_ANGLE);
		delay(1500); //Arm.PitchGo(100); //JSP Delete
		//printFresh(String(Current.ReadAvg(CURRENT_SAMPLE_SIZE)));
		printFresh(String(Current.ReadAvg(CURRENT_SAMPLE_SIZE)) + " - "+ String(Voltage.GrabAvg(20)));
		
	}
}

void TestCamera()
{
	Serial.println("In TestCamera()...");
	uint16_t signature;
	while (1)
	{
		GetDetectedSigs(); //poll camera
		Serial.print("Pink: ");
		Serial.println(Area1);

		Serial.print("Green: ");
		Serial.println(Areat);
		Serial.print("COTTON: ");
		Serial.println(COTTON);
		delay(1000);

		WDT_Restart(WDT);
	}
}

//kEHINDE
void TestCharging()
{
	Serial.println("In Test Charging mode...");
	Serial.println("Charging mode"); // Write something to the LCD
	//note: delay is used here instead of myDelay
	//need to add a little statement that checks if the voltage is actually increasing
	WDT_Restart(WDT);
	Relay.PowerOff();
	delay(2000);

	float BatVoltage = Voltage.GrabAvg(100);
	delay(1000);
	float C = Current.ReadAvg(100); //debug
	float previousMaxVoltage = BatVoltage;

	unsigned long whenIncreasedVoltage = millis();


	while (1)
	{
		WDT_Restart(WDT);

		// if (BatVoltage > previousMaxVoltage)
		// {
		// 	printFresh("Previous val: " + String(previousMaxVoltage));
		// 	delay(1000);
		// 	whenIncreasedVoltage = millis(); //reset timer
		// 	previousMaxVoltage = BatVoltage; //record new high
		// }
		// WDT_Restart(WDT);
		// if ((millis() - whenIncreasedVoltage) > 300000)
		// { //5 minutes
		// 	printFresh("Havent seen voltage increase in 5min: " + String(BatVoltage));
		// 	delay(1000);
		// 	WDT_Restart(WDT);
		// 	break; //havent seen voltage increase in a while
		// }

		if (!CHARGER)
		{ 
			//not touching charging station
			// Serial.println("Charger disconnected");
			printFresh("Charger disconnected");
			// WDT_Restart(WDT);
			// Relay.PowerOn();
			delay(1000); //turn power back on
			
			// checkCamera();
			// printFresh("seek charge station");

			// int chr = seekCharging();
			// Relay.PowerOff();
			// bool redock_success = redock(); //if success, should we reset voltage tracking variables?
			// if (!redock_success)
			// {
			// 	// enable_GoingCharging(); //need to find charging beacon again
			// 	// return; //exit this mode
			// 	seekCharging(); //find charging station again
			// 	Relay.PowerOff();
			// }
		}
		else
		{
			WDT_Restart(WDT);

			unsigned long chr_time = (millis() - whenIncreasedVoltage)/1000.0;

			// Serial.println("Bat Voltage: " + String(BatVoltage));
			// Serial.println("Prev BatVoltage: " + String(previousMaxVoltage)); 
			// Serial.println("Bat Current: " + String(C)); 

			// Serial.println("Charged Voltage: " + String(CHARGED_VOLTAGE)); 
			// Serial.println("Charging time: " + String(chr_time)); 
			
			printFresh("V: "+ String(BatVoltage) + " | C: "+ String(C) + " | T: "+ String(chr_time));

			delay(5000);
			WDT_Restart(WDT);

			C = Current.ReadAvg(100); //debug
			BatVoltage = Voltage.GrabAvg(100); //read current voltage
		}

		// delay(1000); //small waiting delay
	}

	// //now the voltage level has jumped up, but we need to add juce
	// unsigned long chargingStart = millis();
	// //                                m  s   ms
	// unsigned long desiredChargingTime = 1200000; //Kehinde:10mins 9000000;  //2.5hours of charging
	// while (millis() - chargingStart < desiredChargingTime)
	// {
	// 	BatVoltage = Voltage.GrabAvg(100);
	// 	printFresh("Add extra charging: " + String(BatVoltage));

	// 	WDT_Restart(WDT);
	// 	delay(1000); //do nothing
		
	// 	if (!CHARGER)
	// 	{ //not touching charging station
	// 		Relay.PowerOn();
	// 		delay(1000); //turn power back on
	// 		// printFresh("Redocking charge station");

	// 		printFresh("seek charge station");
			
	// 		int chr = seekCharging();
	// 		Relay.PowerOff();
	// 		desiredChargingTime += 5 * 60 * 1000; //add some time to compensate
	// 		// bool redock_success = redock();
	// 		// WDT_Restart(WDT);
	// 		// if (!redock_success)
	// 		// {
	// 		// 	// enable_GoingCharging(); //need to find charging beacon again
	// 		// 	// return; //exit this mode
	// 		// 	seekCharging(); //find charging station again
	// 		// 	Relay.PowerOff();
	// 		// 	desiredChargingTime += 5 * 60 * 1000; //add some time to compensate
	// 		// }
	// 	}

	// 	C=Current.ReadAvg(100);
	// 	printFresh("Charging whileloop: " + String(C)); // Write something to the LCD

	// 	if( C > -0.099 && C <=0)
	// 	{ //good enough
	// 		printFresh("forced Charging exit: ");
	// 		break; //exit this loop
	// 	}
	// }
	// Relay.PowerOn(); //turn the power to the robot on
	// WDT_Restart(WDT);
	// delay(3000);
	// WDT_Restart(WDT);
	// //checkCamera();
	// // checkIMU();
	// // checkCamera();

	// 	// leaveChargingStation();
	// 	// Forward(BASE_SPEED); //start slowly driving forward, anti stuck kick

	// isNewTrip = false;

	// current_target_heading = IN_DIRECTION;
	// // TurnHeading(current_target_heading);
	// enable_turnReversalMode(1);
	// enable_GoingInMode(); 
	//  //go back to digging
	// 	//return;
	// }
}
//Kehinde
void TestAccelReading()
{
	Serial.println("In TestAccel()...");
	// Using the calcAccel helper function, we can get the
	// accelerometer readings in g's.
	while (1)
	{
		Stop();
		Serial.println(F("Forward"));
		Forward(BASE_SPEED);
		delay(2000);
		Stop();
		delay(1000);

		WDT_Restart(WDT);
		Serial.println(F("Backward"));
		Backward(BASE_SPEED);
		delay(2000);
		Stop();
		delay(1000);

		WDT_Restart(WDT);
		Serial.println(F("Right"));
		Right(DEFAULT_TURNING_SPEED);
		delay(2000);
		Stop();
		delay(1000);

		WDT_Restart(WDT);
		Serial.println(F("Left"));
		Left(DEFAULT_TURNING_SPEED);
		delay(2000);
		
		delay(1000);

		dof.readAccel();

		float x = dof.calcAccel(dof.ax);
		float y = dof.calcAccel(dof.ay);
		float z = dof.calcAccel(dof.az);

		Serial.print(x);
		Serial.print(", ");
		Serial.print(y);
		Serial.print(", ");
		Serial.println(z);
		Serial.print("Is stall: ");
		Serial.println(IsStationary());

		delay(1000);
	}
}

//Kehinde
bool IsStationary()
{
	//printFresh("checking if robot is stalled...");
	// Using the calcAccel helper function, we can get the
	// accelerometer readings in g's.
	//unsigned long timeNow = millis();
	//Kehinde
	startTime = millis(); //start time
	timeDiff = startTime - RobotStartTime;
	
	int sampleTime = 4;
	float epsilon = 0.3;
	int counter = 0;

	for (int i = 0; i < sampleTime; i++)
	{
		dof.readAccel();

		float x1 = dof.calcAccel(dof.ax);
		float y1 = dof.calcAccel(dof.ay);

		if (abs(x1) <= epsilon && abs(y1) <= epsilon)
			counter++;

		delay(500); //check if stationary for 2 secs
	}

	//printFresh("Counter: " + String(counter));

	if (counter >= sampleTime)
	{
		writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Stalled", startTime);	
		printFresh("Robot stalled");
		return true;
	}
	else
	{
		//writeSDcard('M', String(noOfTrips) + "," + String(timeDiff) + ",Stalled - False", startTime);		
		//printFresh("Robot not stalled");		
		return false;
	}
}

void TestPickDirection()
{
	// Drive.RightForward(255);
	// Drive.LeftForward(50);
	float desired_heading = 270;
	float current_heading;
	while (1)
	{
		WDT_Restart(WDT);
		current_heading = getHeading();
		Serial.print("desired_heading = ");
		Serial.println(desired_heading);
		Serial.print("current_heading = ");
		Serial.println(current_heading);
		Serial.print("pickDirection = ");
		Serial.println(pickDirection(getHeading(), desired_heading));

		delay(1000);
	}
}

void TestIMUcal()
{
	// Drive.RightForward(255);
	// Drive.LeftForward(50);
	Serial.println("hx, hy");

	float heading;
	float hx;
	float hy;
	while (1)
	{
		WDT_Restart(WDT);

		dof.readMag(); //update registers

		// // Get the raw readings from the IMU sensor
		hx = dof.calcMag(dof.mx);
		hy = dof.calcMag(dof.my);
		Serial.print(hx);
		Serial.print(", ");
		Serial.println(hy);
	}
}

/**

This method is used to read data from the LSM9DS0 magnetometer over the serial.

In order to run this method, in RobotSelector.h set TEST_MODE to TEST_IMU
When the robot turns on, setup() will run, and then upon entering loop() the testing-switch cases will call TestIMU()

I suggest connnected the Due to serial, plugging in the batteries and turning the robot on. Then open the serial monitor
in the Arduino IDE environment. This will cause the robot to reboot and then begin printing heading values.

**/
void TestIMU()
{
	float heading;
	while (1)
	{
		WDT_Restart(WDT);
		heading = getHeading();
		Serial.println(heading);
		delay(1000);
	}
}

void TestGyro()
{
	WDT_Restart(WDT);
	Serial.print(F("G  "));
	dof.readGyro(); //update gyro registers
	float gyroZ = dof.calcGyro(dof.gz);
	Serial.println(gyroZ);
}

void TestSwitches()
{
	WDT_Restart(WDT);
	Serial.println(F("Testing Switches Cycle"));
	if (CONTACT)
	{
		Serial.println(F("Contact: TRUE"));
		RFC = switchState & FR;
		LFC = switchState & FL;
		RSBC = switchState & RSB;
		RSFC = switchState & RSF;
		LSBC = switchState & LSB;
		LSFC = switchState & LSF;
		LBC = switchState & BL;
		RBC = switchState & BR;
		FS = RFC | LFC;
		RS = RSBC | RSFC;
		LS = LSBC | LSFC;
		BS = LBC | RBC;
		//Serial.print(switchState,HEX);//BANI COMMENTED
		//Serial.print(F("\t"));//BANI COMMENTED
		Serial.println(switchState, BIN);
		// Serial.print(F("\t"));
		// Serial.println(LSFC,BIN); //BANI LOGGING CONTACT CHECK
		// Serial.print(F("\t"));s
		// Serial.println(LSBC,BIN); //BANI ,,
		// Serial.print(F("\t"));
		// Serial.println(RSFC,BIN); //BANI ,,
		// Serial.print(F("\t"));
		// Serial.println(RSBC,BIN); //BANI ,,
		// Serial.print(F("\t"));
		// Serial.println(LBC,BIN); //BANI ,,
		// Serial.print(F("\t"));
		// Serial.println(RBC,BIN); //BANI ,,
		Serial.println(LS, BIN); //BANI
		Serial.println(RS, BIN); //BANI
		Serial.println(BS, BIN); //BANI

		delay(100);
	}

	else
	{
		Serial.println(F("Contact: FALSE"));
	}

	if (DUMPING_SWITCH)
	{
		Serial.println(F("dumping switch!"));
	}
}

void TestPowerRelay()
{
	WDT_Restart(WDT);
	Relay.PowerOff(); //turn the power to the robot on
	delay(5000);
	WDT_Restart(WDT);
	Relay.PowerOn(); //turn the power to the robot on
	delay(5000);
}

void TestGripperSensor()
{
	while (1)
	{
		WDT_Restart(WDT);
		Serial.println(analogRead(ForceSensor));
		delay(1000);
	}
}
void TestTurnHeading()
{

	while (1)
	{

		WDT_Restart(WDT); // prevent from resetting

		printFresh("Going in"); //report over radio
		enable_turnReversalMode(1);
		TurnHeadingRoss(IN_DIRECTION); // turn towards the bed
		Forward(BASE_SPEED);
		delay(4000);
		Stop(); // drive forward for a total of one second

		WDT_Restart(WDT); // prevent from resetting - not sure how long TurnHeading() will take

		printFresh("Going out"); //report over radio
		enable_turnReversalMode(3);
		TurnHeadingRoss(OUT_DIRECTION); // turn towards the exit of the tunnel
		Forward(BASE_SPEED);
		delay(4000);
		Stop(); // drive forward for a total of one second
	}
}

void TestPowerSensors()
{
	WDT_Restart(WDT);
	float V = Voltage.Read();
	float C = Current.Read();
	Serial.print(F("Current \t"));
	Serial.print(C);
	Serial.print(F('\t'));
	Serial.print(F("Voltage \t"));
	Serial.println(V);
}
//#endif

void testCap(int panel)
{
	while (1)
	{
		WDT_Restart(WDT);
		Serial.println(CapSensor.getOneContact(panel));		//print capacitive sensor value for only one pin, in this case, pin 0. Change the number to choose other pins.
		printFresh(String(CapSensor.getOneContact(panel))); //send the capacitive sensor value to fio.
		delay(1000);
	}
}

// void myDelay(unsigned long delayTime){
// /* this function is an improvement of delay() function, but non blocking
// so that interrupts can be serviced
// this method is probably a few milliseconds off
// */

// unsigned long maxDelayTime=delayTime*5; //cap this
// unsigned long start=millis(); //remember time this function call started
// while(1){
// WDT_Restart(WDT);
// // Serial.println("!");//debug
// if(CONTACT){
// if (howLongWasContact >= CONTACT_TIME_LIMIT){ //guard against getting contact stuck
// disableContacts=true; //turn off contacts, set global flag
// howLongWasContact=0; //reset contact
// whenDisabledContacts=millis(); //record this time
// }
// handleContact(); //need to make this is not interferers with turning
// //if contacts are not disabled, add some time to the delay to compensate for the time used in
// //servicing interrupts
// if(!disableContacts){
// delayTime=delayTime+howLongWasContact; //compensate
// }
// /*
// if(!disableContacts && diggingMode){
// delayTime=delayTime+DIGGING_INTERRUPT_DELAY;
// }

// if(!disableContacts && goingCharging){
// delayTime=delayTime+GOINGCHARGING_INTERRUPT_DELAY;
// }

// if(!disableContacts && chargingMode){
// delayTime=delayTime+CHARGINGMODE_INTERRUPT_DELAY;
// }
// */
// }

// if(delayTime > maxDelayTime){ //protection against infinite loop. written for goingCharging bug
// if(millis()-start >= maxDelayTime){
// return;
// break;
// }
// }

// if(millis()-start >= delayTime){ //achieved desired delay time
// return;
// break;
// }
// }
// }

// void bumpDelay(unsigned long delayTime){
// /* in this method the robot will execute a non blocking delay function.
// The function will terminate if one of the contact switches is depressed  */
// unsigned long start=millis(); //record start time
// WDT_Restart(WDT);
// while(1){
// if(CONTACT){
// return; // returns and breaks?
// break;
// }

// if(millis()-start >= delayTime){
// return; // returns and breaks?
// break;
// }
// }
// }

void sendPowerUsage(String _state)
{
	unsigned long now; //declare storage var
	float C = Current.ReadAvg(50); //CURRENT_SAMPLE_SIZE
	float V = Voltage.GrabAvg(50); //VOLTAGE_SAMPLE_SIZE
	now = millis();		 //grab time
	float Power = C * V; //compute power, P= current times voltage

	startTime = millis(); //start time
	timeDiff = startTime - RobotStartTime;

	// writeSDcard('C', String(C), now);
	// writeSDcard('V', String(V), now);
	// writeSDcard('W', String(Power), now);

	// writeSDcard('C', String(noOfTrips) + "," + String(timeDiff) + ","+_state+","+String(C)+",", startTime);
	// writeSDcard('V', String(noOfTrips) + "," + String(timeDiff) + ","+_state+","+String(V)+",", startTime);
	writeSDcard('W', String(noOfTrips) + "," + String(timeDiff) + ","+_state+","+String(Power), startTime);
	
	// Serial.print("C \t"); Serial.print(C); Serial.print("\t"); Serial.println(String(now));
	String payload = String(now) + " | Power: " + String(Power);
	Serial.println(payload);

	// Serial.print("V \t"); Serial.println(V);
	//Comm.Send('C',C,now); //send out current
	//Comm.Send('V',V,now); //send out voltage
	//Comm.Send('W',Power,now);
	//delay(1000); //wait to avoid buffer overflow
}

// void writeSDcard(char tag, String data, unsigned long time)
// {
// 	// choose file to write
// 	switch (tag)
// 	{
// 	case 'N':
// 		myFile = SD.open("conlog.csv", FILE_WRITE); //BANI
// 		break;
// 	case 'M':
// 		myFile = SD.open("statelog.csv", FILE_WRITE);
// 		break;
// 	case 'C':
// 		// Serial.println("Writing current----------------------");
// 		myFile = SD.open("currlog.csv", FILE_WRITE);
// 		break;
// 	case 'V':
// 		myFile = SD.open("voltlog.csv", FILE_WRITE);
// 		break;
// 	case 'W':
// 		myFile = SD.open("powerlog.csv", FILE_WRITE);
// 		break;
// 	case 'D':
// 		myFile = SD.open("debuglog.csv", FILE_WRITE);
// 		break;
// 	}

// 	// form payload
// 	String payload = String(time) + "," + data;
// 	Serial.println(payload);
// 	// payload = payload + data + '\t' + time;

// 	// write to SD card
// 	if (myFile)
// 	{
// 		myFile.println(payload);
// 		myFile.close();
// 	}
// 	else
// 	{
// #if DEBUG
// 		Serial.println(F("error opening file"));
// #endif
// 	}
// }

bool CheckPayload()
{
	/* this method will check if the robot is still carrying payload.
		inteded to be used in goingOut mode
		need to return false */

	WDT_Restart(WDT);
	int NumCheckPayload = 0;
	while (NumCheckPayload < 3)
	{
		if (analogRead(ForceSensor) > ForceSensorThresh)
		{
			return true;
		}
		NumCheckPayload++;
	}
	return false;
}

// void logContacts(unsigned long start_of_contact){
// /* this method remembers how long the switches were compressed
// and sets a flag for disabling purposes if the switches were
// compressed for too long*/
// WDT_Restart(WDT);
// if( start_of_contact - whenWasLastContact < CONTINUOUS_CONTACT){//if short duration between counters
// howLongWasContact=howLongWasContact+(millis()-start_of_contact); //add time to contact counter
// if(howLongWasContact > CONTACT_TIME_LIMIT){
// disableContacts=true; //mask them
// whenDisabledContacts=millis(); //record time of disabling contacts
// }
// }
// else{
// howLongWasContact=(millis()-start_of_contact); //reset and update counter
// }
// whenWasLastContact=millis(); //remember when last contact was
// }

// bool checkWatchdog(){
// if(watchdogFlag){ //everything is good)
// //watchdogFlag=0; //reset the flag
// watchdogReset=millis();
// return 0; //return false
// }
// else{
// if( millis() - watchdogReset > 7000 ){
// return 1;
// }
// else{
// return 0;
// }
// }
// }

// void handleTrouble(){
// Backward(BASE_SPEED); delay(1000);
// bumpDelay(1000); //back out
// TurnHeading(current_target_heading);
// watchdogReset=millis();
// return;
// }

bool checkCharger()
{
	WDT_Restart(WDT);
	if (CHARGER && !goingCharging)
	{
		Backward(BASE_SPEED);
		delay(1000);
		delay(1000);
		return true;
	}
	if (CHARGER && goingCharging)
	{
		enable_ChargingMode();
		return true;
	}
	if (CHARGER && restingMode)
	{
		return true;
	}
	return false;
}

// void checkCamera(){
// //maybe get rid of this crap

// // poll camera, see if we get anything back. If not, something is wrong. RESTART
// while(1){//dumb fix
// unsigned long now = millis();
// while ( millis() - now < 16000){
// GetDetectedSigs();
// if(blocks){
// return; //everything is cool
// }
// }
// // arduinoReset();
// Backward(BASE_SPEED); delay(1000); Stop(); //drive back, maybe we are shorted at charging station
// Right(DEFAULT_TURNING_SPEED); delay(200); Stop();
// Relay.PowerOff();
// delay(1000);

// delay(1000);
// while(1){ //keep resetting
// delay(5000);
// arduinoReset();// reset arduino
// delay(5000);
// }
// }
// }

void handleManualOverride()
{
	WDT_Restart(WDT);
	if (checkManualOverride())
	{
		Stop();
		ManualControl();
	}
}

bool checkManualOverride()
{
	WDT_Restart(WDT);
	byte code = masterRead();
	//Serial.print(code); Serial.print('\t'); Serial.println(code,HEX); //debug
	if (code == MANUAL_OVERRIDE_START)
	{
		printFresh("Manual override start"); //send back acknowledgement
		manualMode = true;
		return true;
	}
	else
	{
		return false;
	}
}

void ManualControl()
{
	WDT_Restart(WDT);
	while (1)
	{
		WDT_Restart(WDT);
		byte code = masterRead();
		// Serial.print(F("loop code:   ")); Serial.println(code,HEX);
		switch (code)
		{
		case MANUAL_OVERRIDE_END:
			printFresh("Manual override end"); //send back acknowledgement
			manualMode = false;
			return;
			break;

		case MANUAL_STOP:
			Stop();
			break;
		case MANUAL_GO_FORWARD:
			Forward(BASE_SPEED);
			delay(200);
			Stop();
			break;
		case MANUAL_GO_BACKWARD:
			Backward(BASE_SPEED);
			delay(200);
			Stop();
			break;
		case MANUAL_RIGHT:
			Right(DEFAULT_TURNING_SPEED);
			delay(200);
			Stop();
			break;
		case MANUAL_LEFT:
			Left(DEFAULT_TURNING_SPEED);
			delay(200);
			Stop();
			break;

		//mode changing, acknowledgement is inside functions
		case MANUAL_GOING_IN:
			enable_GoingInMode();
			break;
		case MANUAL_DIGGING:
			enable_DiggingMode();
			break;
		case MANUAL_GOING_OUT:
			enable_GoingOutMode();
			break;
		case MANUAL_DUMPING:
			enable_DumpingMode();
			break;
		case MANUAL_GOING_CHARGING:
			enable_GoingCharging();
			break;
		case MANUAL_CHARGING:
			enable_ChargingMode();
			break;

		//power and reset
		case MANUAL_RESET:
			arduinoReset();
			break;
		case MANUAL_RELAY_ON:
			Relay.PowerOn();
			break;
		case MANUAL_RELAY_OFF:
			Relay.PowerOff();
			break;
		}
	}
}
//end for capacitive sensor

// void TestIRsensorReadings(){
// // Serial.print("R"); Serial.print("\t");
// // Serial.print( IRright.ReadCal() ); Serial.print("\t");
// // Serial.print("L"); Serial.print("\t");
// // Serial.println( IRleft.ReadCal() );
// // delay(100);
// while(1){
// //these are no longer working
// // // Serial.print("R"); Serial.print("\t");
// // // Serial.print( IRright.isDetected() ); Serial.print("\t"); Serial.print( IRright.Read() ); Serial.print("\t");
// // // Serial.print("L"); Serial.print("\t"); Serial.print( IRleft.Read() ); Serial.print("\t");
// // // Serial.println( IRleft.isDetected() );
// Serial.print("R"); Serial.print("\t");
// Serial.print( IRright.Read() ); Serial.print("\t");
// Serial.print("L"); Serial.print("\t"); Serial.println( IRleft.Read() );
// // Serial.println(IRright.isBlindzone);
// }
// }
// ********** END   (TEST AND DEBUG} **********
