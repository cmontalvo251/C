/* Aerodynamics Template 2021

This aerodynamics file is a template for a fictitious portalcube
with thrusters and a simple aero model. The Dynamics.cpp module
will call a few candidate functions. If you make your own aero
file with header and cpp file you must conform to the following
functions otherwise the software will completely break.

*/


#include "aerodynamics.h"
#include "mathp.h" //this is for density at sea-level
#include "timer.h" //for pause function

//Constructor
aerodynamics::aerodynamics() {
	//The constructor must create these 3x1 vectors
	FAEROB.zeros(3,1,"Force Aero in Body Frame");
	MAEROB.zeros(3,1,"Moment Aero in Body Frame");
}

void aerodynamics::setup(MATLAB var) {
	//This function is called once at the beginning of the simulation.
	//Right now there are no standards for var but it is here in case we need it
	#ifdef DEBUG
	printf("Aerodynamics Initialized \n");
	#endif
}

void aerodynamics::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB control) {
	//The only thing this function needs to do is populate FAEROB and MAEROB. 
	//You can do whatever you want in here but you must create those two vectors.
	FAEROB.mult_eq(0); //Zero these out just to make sure something is in here
	MAEROB.mult_eq(0);

	//Aero Parameters
	double S = 0.1; //m^2
	double c = 0.1; //mean chord
	double CD = 1.0; //Linear Drag Coefficient
	double CM = 1.0; //Rotational Drag Coefficient

	//Extract States
	double u = state.get(8,1);
	double v = state.get(9,1);
	double w = state.get(10,1);
	double p = state.get(11,1);
	double q = state.get(12,1);
	double r = state.get(13,1);

	//Total Velocity
	double V = sqrt(u*u + v*v + w*w);

	//Dynamic Pressure
	double qinf = 0.5*RHOSLSI*V*S;

	//Non-Dimensional Angular Velocity
	double pbar=0,qbar=0,rbar=0;
	if (abs(V)>0) {
		pbar = p*c/(2*V);
		qbar = q*c/(2*V);
		rbar = r*c/(2*V);
	}

	//Forces
	FAEROB.plus_eq(qinf);
	FAEROB.mult_eq1(1,1,-u*CD);
	FAEROB.mult_eq1(2,1,-v*CD);
	FAEROB.mult_eq1(3,1,-w*CD); 

	//Moments
	MAEROB.plus_eq(qinf);
	MAEROB.mult_eq1(1,1,-V*pbar*c*CM);
	MAEROB.mult_eq1(2,1,-V*qbar*c*CM);
	MAEROB.mult_eq1(3,1,-V*rbar*c*CM);

	//FAEROB.disp();
	//MAEROB.disp();
	//PAUSE();
}


