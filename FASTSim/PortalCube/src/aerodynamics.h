#ifndef AERODYNAMICS_H
#define AERODYNAMICS_H

/* Aerodynamics Template 2021

This aerodynamics file is a template for a fictitious portalcube
with thrusters and a simple aero model. The Dynamics.cpp module
will call a few candidate functions. If you make your own aero
file with header and cpp file you must conform to the following
functions otherwise the software will completely break.

*/

#include <MATLAB/MATLAB.h> //This is needed for variable length arrays as inputs
#include <Mathp/mathp.h> //this is for density at sea-level
#include <Timer/timer.h> //for pause function
#include <RCIO/RCInput.h> //for stick min values

class aerodynamics {
	private:
		//You can put any private functions or vars in here that you like but it 
		//must adhere to the standards below
		double ACTUATOR_ERROR_PERCENT;
		MATLAB ACTUATOR_ERROR;
	public:
		//This variable turns the aerodynamics model on and off and is set in 
		//Input_Files/Simulation_Flags.txt
		int AERODYNAMICS_FLAG=0;
		int NUMACTUATORS=4; //you also need to include the number of actuators
		//These are 3x1 MATLAB vectors that must be in units of Newtons
		//and in the body frame
		MATLAB FAEROB,MAEROB; 
		void setup(MATLAB var); //this will do any setup required. The input var is....
		//There the inputs are the current time, the current MATLAB state vector,
		//the current derivative state vector and the control vector.
		//The control vector is in units of PWM signal b/c this is an RC A/C sim
		//TAERA1A2 (Throttle, Aileron, Elevator, Rudder, Aux1, Aux2)
		//time is in seconds
		//state is a 13x1 using quaternions and using standard aerospace convention
		//statedot is the derivatives of the state vector
		void ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB control); 
		//Constructor
		aerodynamics();
};

#endif