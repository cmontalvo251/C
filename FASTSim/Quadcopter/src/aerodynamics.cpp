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
	thrust_motors.zeros(4,1,"Force Motors");
	torque_motors.zeros(4,1,"Torque Motors");

	//Quadcopter Aerodynamic Parameters
	//These come from data sheets
  	//at a signal of
  	double pwm_datapt = 1766.0;
  	//thrust is
  	double Tdatapt = 3.5; //Newtons to kg to lbf
  	//angular velocity is
  	double omegaRPMdatapt = 3500.0;
  	//Rotor Size
	Rrotor = (5.0/12.0)/3.28; //5 inch props to meters

	//Compute Kt
  	double dpwm = pwm_datapt - STICK_MIN;
  	kt = Tdatapt/(dpwm*dpwm);
  
  	//Angular Velocity computation
  	double a = (omegaRPMdatapt*2*PI/60.0)/dpwm;

  	//Constants
  	double A = PI*pow(Rrotor,2.0);  //m^2

  	//Compute ct and cq
	ct = kt/(RHOSLSI*A*pow(Rrotor*a,2.0)); //.0335
    cq = sqrt(pow(ct,3.0)/2.0); 

    //Distance from Cg to rotor
    rx = (3.0/12.0)/3.28; //meters
    ry = (3.0/12.0)/3.28; 
    rz = 0.0;
}

void aerodynamics::setup(MATLAB var) {
	//This function is called once at the beginning of the simulation.
	AERODYNAMICS_FLAG = var.get(1,1); //In this case the first variable is whether we run the model or not
	#ifdef DEBUG
	printf("Aerodynamics Initialized \n");
	#endif
}

void aerodynamics::compute_thrust_and_torque(MATLAB ctlcomms) {
	for (int i = 1;i<=4;i++) {
		double thrust = kt*pow((ctlcomms.get(i,1) - STICK_MIN),2.0);
		thrust_motors.set(i,1,thrust);
		torque_motors.set(i,1,thrust*Rrotor*cq/ct);
	}
}

void aerodynamics::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB ctlcomms) {
	//The only thing this function needs to do is populate FAEROB and MAEROB. 
	//You can do whatever you want in here but you must create those two vectors.
	FAEROB.mult_eq(0); //Zero these out just to make sure something is in here
	MAEROB.mult_eq(0);

	//First we need to convert the microsecond pulse to Newtons
	compute_thrust_and_torque(ctlcomms);

	//Thrust on the body is simply the total thrust
	double thrust = thrust_motors.sum();
	//printf("thrust = %lf \n",thrust);
	FAEROB.set(3,1,-thrust);

	//Torque is a bit more complex
	//First we add up all the torques in a specific way
	double aero_torque = torque_motors.get(1,1) - torque_motors.get(2,1) - torque_motors.get(3,1) + torque_motors.get(4,1);
	MAEROB.set(3,1,aero_torque);

	//Then we extract the four forces
    //From the controller - us signals
	//ctlcomms.set(1,1,motor_upper_left);
	//ctlcomms.set(2,1,motor_upper_right);
	//ctlcomms.set(3,1,motor_lower_left);
	//ctlcomms.set(4,1,motor_lower_right);
	//thrust_motors.disp();
	double motor_upper_left = thrust_motors.get(1,1);
	double motor_upper_right = thrust_motors.get(2,1);
	double motor_lower_left = thrust_motors.get(3,1);
	double motor_lower_right = thrust_motors.get(4,1);

	//Now we compute torque on roll and pitch
	double roll_torque = (motor_upper_left+motor_lower_left)*ry - (motor_upper_right+motor_lower_right)*ry;
	double pitch_torque = (motor_upper_left+motor_upper_right)*rx - (motor_lower_right+motor_lower_left)*rx;
	MAEROB.set(1,1,roll_torque);
	MAEROB.set(2,1,pitch_torque);	
	//MAEROB.disp();
}


