/*
Pseudocode derived from https://github.com/openhardwarerobots/arduino-robot-compass/blob/master/robot_mag/robot_mag.ino

	// Global definition of variables for direction control
	float initial_heading;
	float target_heading;
	float diff_heading;
	bool turn_init;
	bool target_reached;

void setup(){
  // Initialise direction variables
  turn_init = false;
  target_reached = false;
}

void turning_method(float desired_angle, bool turn_direction){

	// if we have not started the turn, then we need to set initial_heading
	if (!turn_init){
		initial_heading = get_heading();
		turn_init = true;
		target_reached = false;
		diff_heading = deired_heading - initial_heading;
	}

		float current_heading = get_heading();
		
		while(diff_heading > 0 && !target_reached){
			// move a tiny bit in the direction you want
			
			//update current_heading
			// update diff_heading
			
			// if diff_heading is small (zero or within your tolerance) and !target_reached
				target_reached = true;
				turn_init = false;
				diff_heading = 0;
		
		
		
		}
	
	


}



*/