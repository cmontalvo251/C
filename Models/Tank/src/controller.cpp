//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"

controller::controller() {
	ctlcomms.zeros(NUMSIGNALS,1,"PWM Control Signals"); //The standards must be TAERA1A2A3A4
};

void controller::setup(MATLAB var) {
	CONTROLLER_FLAG = var.get(1,1);
}

void controller::print() {
	for (int i = 1;i<=NUMSIGNALS;i++) {
		printf("%d ",int(ctlcomms.get(i,1)));
	}
}

void controller::loop(double t,MATLAB sensor_state,MATLAB sensor_statedot,int* rxcomms) {
	//I also want to keep track of timeElapsed so that I can run integrators
	timeElapsed = t - lastTime;
	lastTime = t;
	//printf("Time Elapsed = %lf \n",timeElapsed);

	double motor1_us = STICK_MID; //Left Motor
	double motor2_us = STICK_MID; //Right Motor
	double DIFFERENTIAL = 1.0;

	//First extract the relavent commands from the receiver.
	double aileron = rxcomms[1];
	double elevator = rxcomms[2];
	//printf("Elevator = %lf \n",elevator);
	double autopilot = rxcomms[4];

	//Determine if the user wants the controller on or not
	if (autopilot > STICK_MID) {
		CONTROLLER_FLAG = 1;
	} else {
		CONTROLLER_FLAG = 0;
	}

	//Then you can run any control loop you want.
	if (CONTROLLER_FLAG == 1) {
	  //printf("Auto ON \n");
	  motor1_us = STICK_MID + (STICK_MID - elevator) + DIFFERENTIAL*(aileron - STICK_MID);
	  if (motor1_us < STICK_MIN) {
	    motor1_us = STICK_MIN;
	  }
	  if (motor1_us > STICK_MAX) {
	    motor1_us = STICK_MAX;
	  }
	  motor2_us = elevator + DIFFERENTIAL*(aileron - STICK_MID);
	  if (motor2_us < STICK_MIN) {
	    motor2_us = STICK_MIN;
	  }
	  if (motor2_us > STICK_MAX) {
	    motor2_us = STICK_MAX;
	  }
	} else {
	  motor1_us = STICK_MID + (STICK_MID - elevator) - DIFFERENTIAL*(aileron - STICK_MID);
	  motor2_us = elevator - DIFFERENTIAL*(aileron - STICK_MID);

	  if (motor1_us < STICK_MIN) {
	    motor1_us = STICK_MIN;
	  }
	  if (motor1_us > STICK_MAX) {
	    motor1_us = STICK_MAX;
	  }
	  if (motor2_us < STICK_MIN) {
	    motor2_us = STICK_MIN;
	  }
	  if (motor2_us > STICK_MAX) {
	    motor2_us = STICK_MAX;
	  }
	}

	//Set motor commands to the ctlcomms values
	ctlcomms.set(1,1,motor1_us);
	ctlcomms.set(2,1,motor2_us);
	//ctlcomms.disp();
}
