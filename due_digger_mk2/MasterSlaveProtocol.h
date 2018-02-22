/** this header contains 1 byte definitions, 254 commands can be exchanged (255 - FF) */
//needs to be copied over to the fio!!!!
#define NO_DATA 0xF0       //slave has nothing to sent, there is a connection 
#define DUE_ALIVE 0xFF //slave is not sending, connection is broken or something is wrong
#define CAMERA_WORKING 0xFC //This will be sent out every pixy successfully gets something 
#define RESET_REQUEST  0x99
#define CHECK_START	   0x77 //send request to fio to start watchdog counting  
#define CHECK_END      0x88 //send request to fio to stop  watchdog counting 
#define DISP_GOING_IN      0x44 //BANI LCD
#define DISP_DIGGING      0x33 //BANI LCD
#define FRONT_SIDE_ANT      0x1A //CONTACTS//26
#define RIGHT_SIDE_ANT      0x1B //CONTACTS//27
#define LEFT_SIDE_ANT      0x1C //CONTACTS//28
#define BACK_SIDE_ANT      0x1D //CONTACTS//29
#define FRONT_SIDE_WALL     0x2A //CONTACTS//42
#define RIGHT_SIDE_WALL      0x2B //CONTACTS//43
#define LEFT_SIDE_WALL      0x2C //CONTACTS//44
#define BACK_SIDE_WALL      0x2D //CONTACTS//45
#define ROLLED_DIG     0x66 //decision was made to dig//102
#define ROLLED_REST    0x55 //decision was made to rest//85
#define MASTER_GOING_IN         0xA0 //160
#define MASTER_DIGGING          0XB0 //176
#define MASTER_GOING_OUT        0XC0 //192
#define MASTER_DUMPING          0xD0 //208
#define MASTER_GOING_CHARGING   0xE0 //224
#define MASTER_CHARGING         0xAA //170
#define MASTER_RESTING          0xEA //234
#define MASTER_USELESS_RUN		0xED //DEFINED IN DRIVE METHODS, 
#define MASTER_TURN_REVERSAL	0xBA
#define MASTER_EXIT_TUNNEL	0xDA

#define MASTER_OVERRIDE_START   0xEE
#define MASTER_OVERRIDE_END     0xDE
#define MASTER_STOPPED          0x00    
#define MASTER_DRIVING_FORWARD  0x01
#define MASTER_DRIVING_BACKWARD 0x02
#define MASTER_TURNING_RIGHT    0x03
#define MASTER_TURNING_LEFT     0x04

#define MANUAL_OVERRIDE_START   0xEE
#define MANUAL_OVERRIDE_END     0xDE
#define MANUAL_STOP             0x00 //note same as master commands
#define MANUAL_GO_FORWARD       0x01
#define MANUAL_GO_BACKWARD      0x02
#define MANUAL_RIGHT            0x03
#define MANUAL_LEFT             0x04
#define MANUAL_PITCH_UP         0x05
#define MANUAL_PITCH_DOWN       0x06
#define MANUAL_GRIP_UP          0x07
#define MANUAL_GRIP_DOWN        0x08
#define MANUAL_GOING_IN         0xA0
#define MANUAL_DIGGING          0XB0
#define MANUAL_GOING_OUT        0XC0
#define MANUAL_DUMPING          0xD0
#define MANUAL_GOING_CHARGING   0xE0
#define MANUAL_CHARGING         0xAA

#define MANUAL_RESET            0xCC
#define MANUAL_RELAY_ON         0xCA
#define MANUAL_RELAY_OFF        0xCB

