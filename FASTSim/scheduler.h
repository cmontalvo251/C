#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "Datalogger.h" //Always need a data logger
#include "Dynamics.h"; //Always need vehicle dynamics
//Always need timer for the pause function and the error function. 
//May not need to create a TIMER class but still need the built-in functions
#include "timer.h" 
#include <iostream>
#include <stdlib.h>

#if defined (SIL) || (SIMONLY) || (HIL)
//If you're simulating the vehicle you have to have an RK4 Engine
//that integrates the equations of motion
#include "RK4.h"
#endif

//If we are running SIL/HIL we are simulating the flight control board
//which means we need to run in realtime.
//Obviously AUTO must run in realtime as well because that will help with all our timing
#if defined (SIL) || (HIL) || (AUTO)
#define REALTIME
#endif

using namespace std;

class scheduler {
	private:
		//fine time parameters 
  		double t = 0;
  		double PRINT = 0;
  		double LOG = 0;
  		int ok;
  		double startTime,current_time;
  		double tfinal,INTEGRATIONRATE,PRINTRATE;
  		double LOGRATE;

		///REALTIME VARS
		#ifdef REALTIME
		TIMER timer;
		#endif

		#ifdef RK4_H
		RK4 integrator;
		#endif

		//Vehicle Specifics (Every vehicle will have its own dynamic model but everything
		//will be in a Dynamics class)
		Dynamics vehicle;

		///DATALOGGER IS ALWAYS RUNNING
		Datalogger logger;
		MATLAB logvars;
	public:
		//constructor
		scheduler();
		void init();
		void run();
};

#endif