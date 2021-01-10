//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"
#include "RCInput.h" //this is for STICK values

controller::controller() {
	NUMSIGNALS = 8;
	ctlcomms.zeros(NUMSIGNALS,1,"PWM Control Signals"); //The standards must be TAERA1A2A3A4
};

void controller::setup(MATLAB var) {
	CONTROLLER_FLAG = var.get(1,1);
}

void controller::loop(double t,MATLAB state,MATLAB statedot,int* rxcomms) {
	//At a minimum you need to just feed through the rxcomms into the ctlcomms
	for (int idx=0;idx<NUMSIGNALS;idx++){
		ctlcomms.set(idx+1,1,rxcomms[idx]);
	}

	//Then you can run any control loop you want.
	if (CONTROLLER_FLAG == 1) {
		//For this portal cube we want an altitude controller
		double z = state.get(3,1);
		double zdot = statedot.get(3,1);
		double zcommand = -50;
		double kpz = 100;
		double kdz = 50;
		double thrust_comm = kpz*(z-zcommand) + kdz*(zdot) + STICK_MIN;
		ctlcomms.set(1,1,thrust_comm);
	}
}