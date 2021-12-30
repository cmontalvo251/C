/* Forces Template 2021

This forces file is a template for a fictitious portalcube
with thrusters and a simple  model. The Dynamics.cpp module
will call a few candidate functions. If you make your own 
file with header and cpp file you must conform to the following
functions otherwise the software will completely break.

*/

#include "forces.h"

//Constructor
forces::forces() {
	//The constructor must create these 3x1 vectors
	FB.zeros(3,1,"Force in Body Frame");
	MB.zeros(3,1,"Moment in Body Frame");
}

void forces::setup(MATLAB var) {
	//This function is called once at the beginning of the simulation.
	FORCES_FLAG = var.get(1,1); //In this case the first variable is whether we run the model or not
	printf("Forces Initialized \n");
}

void forces::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB actuators) {
	//The only thing this function needs to do is populate FB and MB. 
	//You can do whatever you want in here but you must create those two vectors.
	FB.mult_eq(0); //Zero these out just to make sure something is in here
	MB.mult_eq(0);

	if (FORCES_FLAG == 1) {
		//This is where magnetorquer torque is computed
	}
}


