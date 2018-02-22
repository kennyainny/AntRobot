#include "def.h"
#include "constants.h"
//---encoder 1 vars
extern volatile long encoder1Value; 
extern volatile long encoder1LastValue; 
extern volatile unsigned long encoder1LastTime;
extern volatile char lastBitsAB1;
extern OpticalEncoder encoder1;
//---encoder 2 vars
extern volatile long encoder2Value; 
extern volatile long encoder2LastValue; 
extern volatile unsigned long encoder2LastTime;
extern volatile char lastBitsAB2; // 
extern OpticalEncoder encoder2;
// --- timers

void updateEncoder1(){
//this is work in progress
// digitalWrite(13,HIGH); //debug. used oscilloscope to check how fast this function was executed by measuring square pulse 
char bitA=digitalRead(EncoderPinA1); // store instantaneous HIGH or LOW reading of pin A
char bitB=digitalRead(EncoderPinB1); // store instantaneous HIGH or LOW reading of pin B
char bitsAB=(bitA << 1) |bitB ; //concatenation of bits A and B. 
char stateSum1=( lastBitsAB1 << 2) | bitsAB; //shifting previous bits AB and concatenating with new bits AB 
encoder1LastTime=millis(); //grab interrupt time for velocity calculation
 if ( stateSum1==0b0001 || stateSum1==0b0111 || stateSum1==0b1110 || stateSum1==0b0001){
 encoder1LastValue=encoder1Value; //record last value for velocity calculation
 encoder1Value++; //update global variable
 }
 
 if ( stateSum1==0b0010 || stateSum1==0b1011 || stateSum1==0b1101 || stateSum1==0b0100){
 encoder1LastValue=encoder1Value; //record last value for velocity calculation
 encoder1Value--; //update global variable
 }
 lastBitsAB1=bitsAB; //should this be inside an if statement?? hmmm. possible error tracking?
// digitalWrite(13,LOW);
} 

void updateEncoder2(){
char bitA=digitalRead(EncoderPinA1); // store instantaneous HIGH or LOW reading of pin A
char bitB=digitalRead(EncoderPinB1); // store instantaneous HIGH or LOW reading of pin B
char bitsAB=(bitA << 1) |bitB ; //concatenation of bits A and B. 
char stateSum=( lastBitsAB2 << 2) | bitsAB;
encoder2LastTime=millis(); //grab interrupt time for velocity calculation
 if ( stateSum==0b0001 || stateSum==0b0111 || stateSum==0b1110 || stateSum==0b0001){
 encoder2LastValue=encoder2Value;
 encoder2Value++;
 }
 if ( stateSum==0b0010 || stateSum==0b1011 || stateSum==0b1101 || stateSum==0b0100){
 encoder2LastValue=encoder2Value;
 encoder2Value--;
 }
 lastBitsAB2=bitsAB;
} 