//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"

controller::controller() {
	ctlcomms.zeros(NUMSIGNALS,1,"PWM Control Signals"); //The standards must be TAERA1A2A3A4
};

void controller::setup(MATLAB var) {
	CONTROLLER_FLAG = var.get(1,1);
}

void controller::loop(double t,MATLAB state,MATLAB statedot,int* rxcomms) {
	//I want to keep track of timeElapsed so that I can run integrators
	timeElapsed = t - lastTime;
	lastTime = t;
	//printf("Time Elapsed = %lf \n",timeElapsed);

	//At a minimum you need to compute the 4 motor signals based on 
	//the standard acro mode from the rx comms
	double motor_upper_left = STICK_MIN;
	double motor_upper_right = STICK_MIN;
	double motor_lower_left = STICK_MIN;
	double motor_lower_right = STICK_MIN;		

	//First extract the relavent commands from the receiver.
	double throttle = rxcomms[0];
	double aileron = rxcomms[1];
	double elevator = rxcomms[2];
	double rudder = rxcomms[3];
	double autopilot = rxcomms[4];
	//printf("autopilot = %lf \n",autopilot);

	//Determine if the user wants the controller on or not
	if (autopilot > STICK_MID) {
		CONTROLLER_FLAG = 1;
	} else {
		CONTROLLER_FLAG = 0;
	}

	//Then you can run any control loop you want.
	if (CONTROLLER_FLAG == 1) {
		//STABILIZE MODE
	} else {
		//ACRO MODE
	}

	//Send the motor commands to the ctlcomms values
	ctlcomms.set(1,1,motor_upper_left);
	ctlcomms.set(2,1,motor_upper_right);
	ctlcomms.set(3,1,motor_lower_left);
	ctlcomms.set(4,1,motor_lower_right);
}
