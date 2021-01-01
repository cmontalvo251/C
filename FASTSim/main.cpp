//////Facility for Aerial Systems and Technology (FAST) Real Time Simulator/////////
/////Initial edited by Carlos Montalvo

//See the following Issue on Github - https://github.com/cmontalvo251/C/issues/3

///Revisions created - 12/10/2020 - Added Loop timer

//Revisions Needed 
//Datalogger
//Once we are intialized we start to initialize some things if and only if we need them
//If running on computer import the following
//Rk4 routine including initial conditions and mass properties
//aerodynamic model
//openGL if requested
//Environment
//Vehicle 
//Joystick if manual mode
//Sensor block
//Once everything is imported it's time to kick off the main loop which depends on the algorithm.
//My vote is we do it in this order
//Send state vector to openGL if rendering and desktop
//Send state vector via serial if HIL
//Send state vector to sensor routine if desktop
//Render OpenGL if turned on
//Call the sensor block which polls fictitious sensors on desktop
//Read Joystick or Hardcoded guidance commands (skip if HIL and desktop)
//Pass Commands and Measurements to autopilot specific to drone being simulated (skip if HIL and desktop)
//If desktop pass control signals to RK4 routine
//Integrate RK4 1 step If desktop

#include <stdlib.h>
#include <iostream>
#include "timer.h"
#include "rates.h"
#include "MATLAB.h"
#include "Datalogger.h"

///REALTIME VARS
#ifdef REALTIME
TIMER timer;
#endif

///DATALOGGER IS ALWAYS RUNNING
Datalogger logger;
MATLAB logvars;
#define NUMVARS 1 //num vars to log

//////Main Loop/////////////
int main() {

	//Initialize Datalogger
	logger.findfile("logs/");
	logger.open();
	logvars.zeros(NUMVARS,1,"Vars to Log");
	
	//Define time parameters for Integration
	double t = 0;
	double tfinal = 50;
	double PRINT = 0;
	double LOG = 0;

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
	  	t += INTEGRATIONRATE;

		if (PRINT<t) {
			printf("%lf ",t);
			printf("\n");
			PRINT+=PRINTRATE;
		}

		if (LOG<t) {
			logvars.set(1,1,t);
			logger.println(logvars);
		}

	} //End while loop on main loop

} //end main loop desktop computer
