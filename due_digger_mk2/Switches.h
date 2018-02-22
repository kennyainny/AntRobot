#include "Arduino.h"
#include "RobotSelector.h"

// #include ""
//may neet to put function prototypes 
extern int switchState;
void initiateSwitches(){
//this function will turn on pullup resistor necessary to read voltage
switchState=0x0; //assume nothing is pressed
//set pins to input
pinMode(SwitchPinA,INPUT);
pinMode(SwitchPinB,INPUT);
pinMode(SwitchPinC,INPUT);
pinMode(SwitchPinD,INPUT);
pinMode(SwitchPinE,INPUT);
pinMode(SwitchPinF,INPUT);
pinMode(SwitchPinG,INPUT);
//pinMode(SwitchPinH,INPUT);

// digitalWrite(SwitchPinA,HIGH); //turn on pull up resistors 
// digitalWrite(SwitchPinB,HIGH); //DO NOT ACTUALLY DO THIS 11/11/2014
// digitalWrite(SwitchPinC,HIGH); //THIS WILL MAKE PINS SUCK CHARGING CURRENT 
// digitalWrite(SwitchPinD,HIGH);
// digitalWrite(SwitchPinE,HIGH);
// digitalWrite(SwitchPinF,HIGH);
// digitalWrite(SwitchPinG,HIGH);
// digitalWrite(SwitchPinH,HIGH);
}

int getDetectedContacts(){
//this method will return a hex number representing which switches are turned on or off using polling method
bool switchA=digitalRead(SwitchPinA); //read in switch data
bool switchB=digitalRead(SwitchPinB);
bool switchC=digitalRead(SwitchPinC);
bool switchD=digitalRead(SwitchPinD);
bool switchE=digitalRead(SwitchPinE);
bool switchF=digitalRead(SwitchPinF);
bool switchG=digitalRead(SwitchPinG);
//bool switchH=digitalRead(SwitchPinH);
switchState=0x0;
// switchState= switchA | switchB | switchC | switchD | switchE | switchF | switchG | switchH ;
if(switchA){switchState=switchState | SWITCH_A_MASK;} //bitwise OR  operation
// else{       switchState=switchState ^ SWITCH_A_MASK;} //bitwise XOR operation
if(switchB){switchState=switchState | SWITCH_B_MASK;}
// else{       switchState=switchState ^ SWITCH_B_MASK;}
if(switchC){switchState=switchState | SWITCH_C_MASK;}
// else{       switchState=switchState ^ SWITCH_C_MASK;}
if(switchD){switchState=switchState | SWITCH_D_MASK;}
// else{       switchState=switchState ^ SWITCH_D_MASK;}
if(switchE){switchState=switchState | SWITCH_E_MASK;}
// else{       switchState=switchState ^ SWITCH_E_MASK;}
if(switchF){switchState=switchState | SWITCH_F_MASK;}
// else{       switchState=switchState ^ SWITCH_F_MASK;}
if(switchG){switchState=switchState | SWITCH_G_MASK;}
// else{       switchState=switchState ^ SWITCH_G_MASK;}
// if(switchH){switchState=switchState | SWITCH_H_MASK;}
// else{       switchState=switchState ^ SWITCH_H_MASK;}
return switchState;
}

/*
void DefineSwitchInterrupts(){
attachInterrupt(SwitchPinA, UpdateSwitchA, CHANGE);
attachInterrupt(SwitchPinB, UpdateSwitchB, CHANGE);
attachInterrupt(SwitchPinC, UpdateSwitchC, CHANGE);
attachInterrupt(SwitchPinD, UpdateSwitchD, CHANGE);
attachInterrupt(SwitchPinE, UpdateSwitchE, CHANGE);
attachInterrupt(SwitchPinF, UpdateSwitchF, CHANGE);
attachInterrupt(SwitchPinG, UpdateSwitchG, CHANGE);
attachInterrupt(SwitchPinH, UpdateSwitchH, CHANGE);

}

void UpdateSwitchA(){
switchA=digitalRead(SwitchPinA);
if(switchA){switchState=switchState | SWITCH_A_MASK;}
else{       switchState=switchState ^ SWITCH_A_MASK;}
}

void UpdateSwitchB(){
switchB=digitalRead(SwitchPinB);
if(switchB){switchState=switchState | SWITCH_B_MASK;}
else{       switchState=switchState ^ SWITCH_B_MASK;}
}

void UpdateSwitchC(){
switchC=digitalRead(SwitchPinC);
if(switchC){switchState=switchState | SWITCH_C_MASK;}
else{       switchState=switchState ^ SWITCH_C_MASK;}
}

void UpdateSwitchD(){
switchD=digitalRead(SwitchPinD);
if(switchD){switchState=switchState | SWITCH_D_MASK;}
else{       switchState=switchState ^ SWITCH_D_MASK;}
}

void UpdateSwitchE(){
switchE=digitalRead(SwitchPinE);
if(switchE){switchState=switchState | SWITCH_E_MASK;}
else{       switchState=switchState ^ SWITCH_E_MASK;}
}

void UpdateSwitchF(){
switchF=digitalRead(SwitchPinF);
if(switchF){switchState=switchState | SWITCH_F_MASK;}
else{       switchState=switchState ^ SWITCH_F_MASK;}
}

void UpdateSwitchG(){
switchG=digitalRead(SwitchPinG);
if(switchG){switchState=switchState | SWITCH_G_MASK;}
else{       switchState=switchState ^ SWITCH_G_MASK;}
}

void UpdateSwitchH(){
switchH=digitalRead(SwitchPinH);
if(switchH){switchState=switchState | SWITCH_H_MASK;}
else{       switchState=switchState ^ SWITCH_H_MASK;} 
*/
