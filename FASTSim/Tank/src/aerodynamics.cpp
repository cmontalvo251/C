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

void aerodynamics::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB ctlcomms) {
	//The only thing this function needs to do is populate FAEROB and MAEROB. 
	//You can do whatever you want in here but you must create those two vectors.
	FAEROB.mult_eq(0); //Zero these out just to make sure something is in here
	MAEROB.mult_eq(0);

	if (AERODYNAMICS_FLAG == 1) {
        //Friction Parameters
        double XDAMPCOEFF = 7.26;
        double DAMPCOEFF = 50.0; //Guess and Check (y-direction)
		double DAMPROTCOEFF = 1.0; //Guess and Check (yaw)
		double d = 0.13335; //(m) - From wheel to center
		double force1;
		double force2;
		double Vmax = 1.0; //(m/s) Need to find the max speed of the tank
		
		//Extract Actuator Values
		//Remember that control is in PWM (us)
		double motor1_US = ctlcomms.get(1,1);
		double motor2_US = ctlcomms.get(2,1);

		//Extract States
		double x = state.get(1,1);
		//printf("x = %lf \n",x);
		double u = state.get(8,1);
		//printf("u = %lf \n",u);
		double v = state.get(9,1);
		double w = state.get(10,1);
		double p = state.get(11,1);
		double q = state.get(12,1);
		double r = state.get(13,1);

		//Calculate Forces
		double force_max = (((1.7)/2.2)*9.81 + 4.454)/2.0;
		double s = 0.007681;
		//motor1_US = STICK_MAX;
		//motor2_US = STICK_MAX;
		double dpwm1 = (motor1_US-STICK_MID);
		double dpwm2 = (motor2_US-STICK_MID); //Need to find equation by plotting microsec vs force
		force1 = -copysign(1.0,dpwm1)*force_max*(1-exp(-s*fabs(dpwm1)));
		force2 = copysign(1.0,dpwm2)*force_max*(1-exp(-s*fabs(dpwm2)));
		//printf("forces before = %lf %lf \n",force1,force2);

		/*double vf=1.0;
		//First check and see if the user is trying to accelerate and moving forward
		if ((u > 0) && (force1+force2 > 0)) {
			vf = 1-u/Vmax;
		}
		if ((u < 0) && (force1 + force2 < 0)) {
			vf = 1+u/Vmax;
		}

		if (vf < 0) {
			force1 = 0.0;
			force2 = 0.0;
		} else {
			force1 *= vf;
			force2 *= vf;
		}*/

		//printf("forces after = %lf %lf \n",force1,force2);
		double xforce = force1 + force2 - XDAMPCOEFF*u;
		double yforce = -DAMPCOEFF*v;

		//Calculate Moments
		double Nmoment = (force1 - force2)*d - DAMPROTCOEFF*r;
		
		//Forces
		FAEROB.plus_eq1(1,1,xforce);
		FAEROB.plus_eq1(2,1,yforce);
		FAEROB.plus_eq1(3,1,0.0); 

		//Moments
		MAEROB.plus_eq1(1,1,0.0);
		MAEROB.plus_eq1(2,1,0.0);
		MAEROB.plus_eq1(3,1,Nmoment);

		//FAEROB.disp();
		//MAEROB.disp();
		//PAUSE();
	}
}


