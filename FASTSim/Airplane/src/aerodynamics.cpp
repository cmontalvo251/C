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
		
		//Get States
		double z = state.get(3,1);
		double u = state.get(8,1);
  		double v = state.get(9,1);
  		double w = state.get(10,1);
		double p = state.get(11,1);
  		double q = state.get(12,1);
  		double r = state.get(13,1);
  		//state.disp();

  		//Total Velocity
		double vinf=sqrt(u*u+v*v+w*w); //#Stream line velocity: Total velocity
  
  		//Angle of Attack and sideslip
  		double beta=asin(v/vinf); //#Equation 21: Sideslip angle beta
  		double alpha=atan(w/u); //#Equation 20: Angle of attack alpha

  		//printf("AOA,BETA = %lf %lf \n",beta,alpha);

  		//Non-deminnsion angular velocity
  		double phat=(p*aeropack.bws)/(2*vinf);
  		double qhat=(q*aeropack.cbar)/(2*vinf);
  		double rhat=(r*aeropack.bws)/(2*vinf);

  		//printf("pqrhat = %lf %lf %lf \n",phat,qhat,rhat);

		//Extract Controls
		//Extract Actuator Values
		//Remember that control is in PWM (us)
		double mewt = actuators.get(1,1);
		double delaUS = actuators.get(2,1);
		double deleUS = actuators.get(3,1);
		double delrUS = actuators.get(4,1);

		//Convert US to degrees
		double dela = 2*30*PI/(180.0*(STICK_MAX-STICK_MIN))*(delaUS-STICK_MID);
		double dele = 2*30*PI/(180.0*(STICK_MAX-STICK_MIN))*(deleUS-STICK_MID); 
		double delr = 2*30*PI/(180.0*(STICK_MAX-STICK_MIN))*(delrUS-STICK_MID);

		//printf("DELR = %lf \n",delr);

		//actuators.disp();

		//Compute Thrust
		double omega = aeropack.spin_slope*(mewt - STICK_MIN);
		double Thrust = 0.5*RHOSLSI*aeropack.AREA*pow(omega*aeropack.Rrotor,2.0)*aeropack.ct;
		if (vinf < aeropack.max_speed) {
			Thrust *= vinf/aeropack.max_speed;
		} else {
			Thrust = 0.0;
			//printf("Velocity = %lf \n",vinf);
		}
		//printf("Thrust = %lf \n",Thrust);
		

  		//Aerodynamics Coefficients
  		double CL = aeropack.CLzero + (aeropack.CLalpha*alpha) + (aeropack.CLq*qhat) + (aeropack.CLdele*dele); //#Equation 19: Coefficient of Lift
  		double CD = aeropack.CDzero + (aeropack.CDalpha*(alpha*alpha)); //#Equation 19: Coefficient of Drag
  		double CY = (aeropack.Cybeta*beta)+(aeropack.Cydelr*delr)+(aeropack.Cyp*phat)+(aeropack.Cyr*rhat);
  		//printf("CY = %lf \n",CY);
  		double Cm = (aeropack.Cmzero)+(aeropack.Cmalpha*alpha)+(aeropack.Cmq*qhat)+(aeropack.Cmdele*dele);
  		double Cl = (aeropack.CLbeta*beta)+(aeropack.CLp*phat)+(aeropack.CLr*rhat)+(aeropack.CLdela*dela)+(aeropack.CLdelr*delr);
  		double Cn = (aeropack.Cnp*phat)+(aeropack.Cnbeta*beta)+(aeropack.Cnr*rhat)+(aeropack.Cndela*dela)+(aeropack.Cndelr*delr);
  
  		//Forces and Moments
  		FAEROB.set(1,1,CL*sin(alpha)-(CD*cos(alpha)));
  		FAEROB.set(2,1,CY);
  		FAEROB.set(3,1,-CL*cos(alpha)-CD*sin(alpha)); //#Equation 18: Aerodynamic forces
  		FAEROB.mult_eq(0.5*RHOSLSI*(vinf*vinf)*aeropack.S);
  		FAEROB.plus_eq1(1,1,Thrust);
		MAEROB.set(1,1,aeropack.bws*Cl);
  		MAEROB.set(2,1,aeropack.cbar*Cm);
  		MAEROB.set(3,1,aeropack.bws*Cn);
  		MAEROB.mult_eq(0.5*RHOSLSI*(vinf*vinf)*aeropack.S);

  		//FAEROB.disp();
  		//MAEROB.disp();
  		//PAUSE();
	}
}

