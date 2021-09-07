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
	printf("Aerodynamics Initialized \n");
}

void aerodynamics::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB actuators) {
	//The only thing this function needs to do is populate FAEROB and MAEROB. 
	//You can do whatever you want in here but you must create those two vectors.
	FAEROB.mult_eq(0); //Zero these out just to make sure something is in here
	MAEROB.mult_eq(0);

	if (AERODYNAMICS_FLAG == 1) {
		//Extract Actuator Values
		//Remember that control is in PWM (us)
		double throttleUS = actuators.get(1,1);
		double aileronUS = actuators.get(2,1);
		double elevatorUS = actuators.get(3,1);
		double rudderUS = actuators.get(4,1);

		//Convert throttle signals to thruster value
		double TMAX = 1000;
		double TORQUEMAX = 10.0;
		double max_slope = (STICK_MAX-STICK_MIN);
		double mid_slope = (STICK_MAX-STICK_MID);
		double Zthrust = -(throttleUS - STICK_MIN)/max_slope*TMAX;
		double Lthrust = (aileronUS - STICK_MID)/mid_slope*TORQUEMAX;
		double Mthrust = (elevatorUS - STICK_MID)/mid_slope*TORQUEMAX;
		double Nthrust = (rudderUS - STICK_MID)/mid_slope*TORQUEMAX;

		//Aero Parameters
		double S = 0.1; //m^2
		double c = 1.0; //mean chord
		double CD = 1.0; //Linear Drag Coefficient
		double CM = 3.0; //Rotational Drag Coefficient

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
		} else {
			pbar = p*c;
			qbar = q*c;
			rbar = r*c;
		}

		//Forces
		FAEROB.plus_eq(qinf);
		FAEROB.mult_eq1(1,1,-u*CD);
		FAEROB.mult_eq1(2,1,-v*CD);
		FAEROB.mult_eq1(3,1,-w*CD); 
		//Add Thrust
		FAEROB.plus_eq1(3,1,Zthrust);

		//Moments
		MAEROB.plus_eq(qinf);
		MAEROB.mult_eq1(1,1,-V*pbar*c*CM);
		MAEROB.mult_eq1(2,1,-V*qbar*c*CM);
		MAEROB.mult_eq1(3,1,-V*rbar*c*CM);
		//Add Thruster
		MAEROB.plus_eq1(1,1,Lthrust);
		MAEROB.plus_eq1(2,1,Mthrust);
		MAEROB.plus_eq1(3,1,Nthrust);

		//FAEROB.disp();
		//MAEROB.disp();
		//PAUSE();
	}
}


