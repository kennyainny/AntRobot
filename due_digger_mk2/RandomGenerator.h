#ifndef RandomGenerator_h
#define RandomGenerator_h
#include "Arduino.h"
// #include "enableModes.h"
#include "RobotSelector.h"
#define PROB_SCALE_MIN 0
#define PROB_SCALE_MAX 100

// #define GO_DIG 1
// #define GO_REST 2
extern void enable_RestingMode();
extern void enable_GoingInMode();

bool rollDice(){
	int randNumber = random(PROB_SCALE_MIN, PROB_SCALE_MAX);
	// Serial.println(randNumber); //debug
	//go_dig_prob eventually needs to be a variable so that it can be reduced!!!
	if( randNumber >= PROB_SCALE_MIN && randNumber <= GO_DIG_PROB){
		// enable_GoingInMode(); 
		// Serial.println("in dice");
		// Serial.println("in dice"); //debug 
		return 0;
	}
	if( randNumber > GO_DIG_PROB && randNumber <= PROB_SCALE_MAX){
		// enable_RestingMode(); 
		// Serial.println("Rest dice"); //debug
		return 1;
	}
}

/**

Returns true if the random number generated is less than the theshold value passed to the method
Returns false if the random number generated is greater than the theshold value passed to the method

**/
bool rollDiceProb(int thresh){
	int randNumber = random(0,100);
	if (randNumber < thresh ){
		return 1;
	}
	else{
		return 0;
	}
}
#endif
