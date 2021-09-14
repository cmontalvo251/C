/* Aerodynamics Template 2021

This aerodynamics file is a template for a fictitious portalcube
with thrusters and a simple aero model. The Dynamics.cpp module
will call a few candidate functions. If you make your own aero
file with header and cpp file you must conform to the following
functions otherwise the software will completely break.

*/

#include "aerodynamics.h"

//Constructor
aerodynamics::aerodynamics() {
	//The constructor must create these 3x1 vectors
	FAEROB.zeros(3,1,"Force Aero in Body Frame");
	MAEROB.zeros(3,1,"Moment Aero in Body Frame");
}

void aerodynamics::setup(MATLAB var) {
	//This function is called once at the beginning of the simulation.
	AERODYNAMICS_FLAG = var.get(1,1); //In this case the first variable is whether we run the model or not
	#ifdef DEBUG
	printf("Aerodynamics Initialized \n");
	#endif
}

void aerodynamics::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB control) {
	//The only thing this function needs to do is populate FAEROB and MAEROB. 
	//You can do whatever you want in here but you must create those two vectors.
	FAEROB.mult_eq(0); //Zero these out just to make sure something is in here
	MAEROB.mult_eq(0);
}


