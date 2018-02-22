/* 
This class sets up optical encoders, manufactured by Pololu.com , item# 1217
These encoders were designed for a 42x19mm wheel with 48 counts per revolution

http://www.pololu.com/product/1217

this code is written exclusivity for Arduino Due and wont work on other arduinos
 */
 
 #ifndef OpticalEncoder_h
 #define OpticalEncoder_h
 #include "Arduino.h"
 class OpticalEncoder{
 
 public:
 // static void foo(); //thats how you do interrupts
 // have to get rid of public vars
 OpticalEncoder(int encoderOutA, int encoderOutB); //constructor
 void Initialize();
 // static void serviceEncoder(); //this function will be called when encoder interrupt is serviced
 // int bar=10;
 private:
 int _encoderOutA; //variable to store passed in pin number
 int _encoderOutB; //variable to store passed in pin number
 };
 


 
 #endif