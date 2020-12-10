//////Facility for Aerial Systems and Technology (FAST) Real Time Simulator/////////
/////Initial edited by Carlos Montalvo


///Revisions created - 12/10/2020 - Added Loop timer

#include <stdlib.h>
#include <iostream>
#include "timer.h"

#ifdef REALTIME
TIMER timer;
#endif

//////Main Loop/////////////
int main() {
	
	//Define time parameters for Integration
	double t = 0;
	double tfinal = 50000;
	double timestep = .01; //only if we're not doing HIL
	double PRINT = 0;
	double PRINTRATE = 1.0;

	//Initialize timer if simulating in realtime
	#ifdef REALTIME
	double startTime = timer.getTimeSinceStart();
	double current_time = timer.getTimeSinceStart() - startTime;
	#endif

    //Kick off integration loop
	while (t < tfinal) {

		#ifdef REALTIME
		//Get current time 
		while (current_time < t) {
			current_time = timer.getTimeSinceStart()-startTime;		
		}
		#endif

		//Integrate time
	  	t += timestep;

		if (PRINT<t) {
			printf("%lf ",t);
			printf("\n");
			PRINT+=PRINTRATE;
		}

	} //End while loop on main loop

} //end main loop desktop computer
