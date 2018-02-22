#ifndef enableModes_h
#define enableModes_h
#include "Arduino.h"
#include "MasterSlaveProtocol.h"
/** This file sets up commands to change operation modes  */

extern bool goingIn;
extern bool diggingMode;
extern bool goingOut;
extern bool dumpingMode;
extern bool goingCharging;
extern bool chargingMode;
extern bool restingMode;
extern bool exitTunnelMode; // Mode added by JSP, happens after diggingMode, backs up the robot after successful digging
extern bool turnReversalMode; // Mode added by JSP, does 180 degree turn reversal after dumping mode and exit tunnel mode
extern int nextMode;

//extern void masterWrite(int protocolCode);
void enable_GoingInMode(){
	goingIn       = true;
	diggingMode   = false;
	goingOut      = false;
	dumpingMode   = false;
	goingCharging = false;
	chargingMode  = false;
	restingMode   = false;
	exitTunnelMode = false;
	turnReversalMode = false;
}

void enable_DiggingMode(){
	goingIn       = false;
	diggingMode   = true;
	goingOut      = false;
	dumpingMode   = false;
	goingCharging = false;
	chargingMode  = false;
	restingMode   = false;
	exitTunnelMode = false;
	turnReversalMode = false;
}

void enable_GoingOutMode(){
	goingIn       = false;
	diggingMode   = false;
	goingOut      = true;
	dumpingMode   = false;
	goingCharging = false;
	chargingMode  = false;
	restingMode   = false;
	exitTunnelMode = false;
	turnReversalMode = false;
}

void enable_DumpingMode(){
	goingIn       = false;
	diggingMode   = false;
	goingOut      = false;
	dumpingMode   = true;
	goingCharging = false;
	chargingMode  = false;
	exitTunnelMode = false;
	turnReversalMode = false;
}

void enable_GoingCharging(){
	goingIn       = false;
	diggingMode   = false;
	goingOut      = false;
	dumpingMode   = false;
	goingCharging = true;
	chargingMode  = false;
	restingMode   = false;
	exitTunnelMode = false;
	turnReversalMode = false;
}

void enable_ChargingMode(){
	goingIn       = false;
	diggingMode   = false;
	goingOut      = false;
	dumpingMode   = false;
	goingCharging = false;
	chargingMode  = true; //ensure proper state variables since ISR interrupt routine is used
	restingMode   = false;
	exitTunnelMode = false;
	turnReversalMode = false;
}


void enable_RestingMode(){
	goingIn       = false;
	diggingMode   = false;
	goingOut      = false;
	dumpingMode   = false;
	goingCharging = false;
	chargingMode  = false;
	restingMode   = true;
	exitTunnelMode = false;
	turnReversalMode = false;
}

void enable_exitTunnelMode(){
	goingIn       = false;
	diggingMode   = false;
	goingOut      = false;
	dumpingMode   = false;
	goingCharging = false;
	chargingMode  = false;
	restingMode   = false;
	exitTunnelMode = true;
	turnReversalMode = false;
}

void enable_turnReversalMode(int modeNumber){
	//int modeNumber shows the next mode the robot will take after the turn reversal mode is completed
	goingIn       = false;
	diggingMode   = false;
	goingOut      = false;
	dumpingMode   = false;
	goingCharging = false;
	chargingMode  = false;
	restingMode   = false;
	exitTunnelMode = false;
	turnReversalMode = true;
	nextMode = modeNumber;
}


#endif

