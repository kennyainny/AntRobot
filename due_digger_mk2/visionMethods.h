/* this file contains pixy camera stuff */
#include "Arduino.h"
#include "RobotSelector.h"
#include "MasterSlaveProtocol.h"
#define NUMBER_OF_SIGS 5

/* NEED TO CLEAN UP , manage storage arrays and global vars  */

/* Note to programmer: PAY ATTENTION to the array indices . */
extern PixyUART pixy;       //import pixy object 
extern uint16_t blocks;    //store number of blocks detected when frames are sampled
extern bool Object[NUMBER_OF_SIGS];     //array used to set up global detection flags 
extern uint16_t x1, xc, x7, xt, xCharging; //declare storage variables
extern uint16_t Area1, Areac, Area7, Areat, AreaCharging; //declare storage variables;
extern void fioWrite(int); // 
// extern void WDT_Setup(); //appearently not needed to be included 

// extern unsigned long when_saw_trails;
 struct SigInfo {
 //this structure stores information of a biggest block detected for a particular color
 //uint16_t signature; //store color signature, takes on values from 1 to 7
 int Area=0; //int MaxArea; uint8_t MaxAreaIndex; 
 uint16_t x=0; 
 uint16_t y=0;
 // uint16_t Width; //do i really need this? dont think so 
 // int Angle; //can be added to the struct and used with color codes
 };
SigInfo Sigs[NUMBER_OF_SIGS]; //declare an array of struct for storage note that the length of this array is 7 cause 0 counts

void GetDetectedSigs(){
	WDT_Restart(WDT);
	#ifdef FIO_LINK
		// fioWrite(CHECK_START); //disabled
	#endif

	/*this method performs filtering of incoming detected color data
	only the biggest block of each color signature is detected
	returns pointer to an array struct */
	/* pixy has 320x240 output resolution  */
	
	/* Declare storage space and clear previous information */


	int Area[NUMBER_OF_SIGS];
	int MaxArea[NUMBER_OF_SIGS]; //declare area storage arrays. This is where the number of objects recognized by pixy is initated
	uint16_t signature; //store color signature, takes on values from 1 to 7
	//int i; //dummy variable

	for(int k=0; k<NUMBER_OF_SIGS;k++){ //first, previous information is erased or zeroed

		Area[k]=0;
		MaxArea[k]=0;
		Sigs[k].x=0;
		Sigs[k].y=0;
		Sigs[k].Area=0;
		// Sigs[k].Width=0;
		Object[k]=false; //probably should be enough to just do this, and do if(Object) before trying to access data. Everything would still have to be initialized to zero in constructor
	}

	// Serial.println("In getDetectedSigs()...");
	blocks = pixy.getBlocks();       //Poll camera, get blocks. CODE CAN GET STUCK HERE
	// Serial.println("Right after pixy.getBlocks()"); // debug

	if(blocks){ //start pulling information
		for(int j=0; j<blocks; j++){	//cycle trough the blocks
			signature=pixy.blocks[j].signature; //read color signature
			// Serial.println(signature); //debug 
			switch(signature){ //assign storage array index
				//watch out for "off by one bug!"
				
				case 1: //pheromone trail 1, storage array index 0
					//when_saw_trails=millis(); //update timer
					Area[0]=pixy.blocks[j].width * pixy.blocks[j].height; //calculate block area of a current match
					if(pixy.blocks[j].y > 100) { //object detected. FIlters out stuff ahead
					// Area[i]=pixy.blocks[j].width * pixy.blocks[j].height; //calculate block area
						if( (Area[0] > MaxArea[0])  ){//if this block has the biggest area comparing to the blocks of the same color detected in this instance, store information
							MaxArea[0]=Area[0];         //bump up area
							Sigs[0].x=pixy.blocks[j].x; //store x
							Sigs[0].y=pixy.blocks[j].y; //store y
							// Sigs[i].Width=pixy.blocks[j].width; //store width
							Sigs[0].Area=Area[0];       //store Area
							Object[0]=true;             //activate a flag to indicate detected color
							x1=Sigs[0].x; //grab x coordinate of detected pheromone trail 1 block
							Area1=Sigs[0].Area;
						}   
					} 
					break;
				
				case 2: //charging pheromone trail, storage array index 1
					Area[1]=pixy.blocks[j].width * pixy.blocks[j].height; //calculate block area of a current match
					// if(pixy.blocks[j].y > 100) { //object detected. FIlters out stuff ahead
					// Area[i]=pixy.blocks[j].width * pixy.blocks[j].height; //calculate block area
					if( (Area[1] > MaxArea[1])  ){//if this block has the biggest area comparing to the blocks of the same color detected in this instance, store information
						MaxArea[1]=Area[1];         //bump up area
						Sigs[1].x=pixy.blocks[j].x; //store x
						Sigs[1].y=pixy.blocks[j].y; //store y
						// Sigs[i].Width=pixy.blocks[j].width; //store width
						Sigs[1].Area=Area[1];       //store Area
						Object[1]=true;             //activate a flag to indicate detected color
						xc=Sigs[1].x; //grab x coordinate of detected pheromone trail 2 block
						Areac=Sigs[1].Area;
					}   
					break;
				//color coded signature is returned in octal number scale
				//5*8^0+3*8^1+5*8^2=349. 
				// case 349: //dumping area beacon. should be 535
				// case 29:  //dumping area beacon but with 1 sig missing
				// i=2;
				// Area[i]=pixy.blocks[j].width * pixy.blocks[j].height; //calculate block area of a current match
				// break;
				// case 284: //charging area. 
				// case 28:
				// i=3;
				// Area[i]=pixy.blocks[j].width * pixy.blocks[j].height; //calculate block area of a current match
				// break; 
				// case 6:   //other ant
				// i=5;
				// Area[i]=pixy.blocks[j].width * pixy.blocks[j].height; //calculate block area of a current match
				// break; 
   
				//SHOULD BE case 6. disabling this feature
				case 3: // GREEN
					//i=3;
					Area[3]=pixy.blocks[j].width * pixy.blocks[j].height; //calculate block area of a current match
					if( (Area[3] > MaxArea[3]) ){//if this block has the biggest area comparing to the blocks of the same color detected in this instance, store information
						MaxArea[3]=Area[3];         //bump up area
						Sigs[3].x=pixy.blocks[j].x; //store x
						Sigs[3].y=pixy.blocks[j].y; //store y
						Sigs[3].Area=Area[3];       //store Area
						Object[3]=true;             //activate a flag to indicate detected color
						//UPDATE GLOBAL VARS
						xt=Sigs[3].x; //grab x coordinate of detected tunnel start
						Areat=Sigs[3].Area;
						// Serial.println(Area7);
					} 
					break;

				case 4: //charging area
					//i=4;
					Area[4]=pixy.blocks[j].width * pixy.blocks[j].height; //calculate block area of a current match
					if( (Area[4] > MaxArea[4])  ){//if this block has the biggest area comparing to the blocks of the same color detected in this instance, store information
						MaxArea[4]=Area[4];         //bump up area
						Sigs[4].x=pixy.blocks[j].x; //store x
						Sigs[4].y=pixy.blocks[j].y; //store y
						Sigs[4].Area=Area[4];       //store Area
						Object[4]=true;             //activate a flag to indicate detected color
						//UPDATE GLOBAL VARS
						xCharging=Sigs[4].x; //grab x coordinate of detected charging area
						AreaCharging=Sigs[4].Area;
						// Serial.println(Area7);
					} 
					break;
   
				case 7:   //colored cotton, storage array index 2, USED TO BE 7
					//i=2;
					Area[2]=pixy.blocks[j].width * pixy.blocks[j].height; //calculate block area of a current match
					if( (Area[2] > MaxArea[2])  ){//if this block has the biggest area comparing to the blocks of the same color detected in this instance, store information
						MaxArea[2]=Area[2];         //bump up area
						Sigs[2].x=pixy.blocks[j].x; //store x
						Sigs[2].y=pixy.blocks[j].y; //store y
						Sigs[2].Area=Area[2];       //store Area
						Object[2]=true;             //activate a flag to indicate detected color
						//UPDATE GLOBAL VARS
						x7=Sigs[2].x; //grab x coordinate of detected cotton wall
						Area7=Sigs[2].Area;
						// Serial.println(Area7);
					}     
					break;
			}// ends switch(signature)
		} //ends for loop
	}//ends if (blocks)
	WDT_Restart(WDT);
	#ifdef FIO_LINK
		// fioWrite(CHECK_END);
	#endif
}