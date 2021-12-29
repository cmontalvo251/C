//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"

controller::controller() {
	ctlcomms.zeros(NUMSIGNALS,1,"PWM Control Signals"); //The standards must be TAERA1A2A3A4
};

void controller::setup(MATLAB var) {
	CONTROLLER_FLAG = var.get(1,1);
	//Commented out but you can use these if you'd like
	//Just make sure to create vars in the *.h file
	//mass = var.get(2,1);
	//Ixx = var.get(3,1);
	//Iyy = var.get(4,1);
	//Izz = var.get(5,1);
	////You can always add some noise to these parameters as they are 
	///not connected to the vehicle state
}

void controller::print() {
	for (int i = 1;i<=NUMSIGNALS;i++) {
		printf("%d ",int(ctlcomms.get(i,1)));
	}
}

void controller::loop(double t,MATLAB sensor_state,MATLAB sensor_statedot,int* rxcomms) {
	//The sensor state is a 12x1 of standard 6DOF sensors
	//At a minimum you need to just feed through the rxcomms into the ctlcomms
	//Which means you can't have more control signals than receiver signals
	for (int idx=0;idx<NUMSIGNALS;idx++){
		ctlcomms.set(idx+1,1,rxcomms[idx]);
	}

	//I also want to keep track of timeElapsed so that I can run integrators
	timeElapsed = t - lastTime;
	lastTime = t;
	//printf("Time Elapsed = %lf \n",timeElapsed);

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
		#ifdef SIL 
		printf("Auto ON \n");
		#endif
		//For this portal cube we want an altitude controller
		double z = sensor_state.get(3,1);
		double zdot = sensor_statedot.get(3,1);
		double zcommand = -50;
		double kpz = 100;
		double kdz = 50;
		double thrust_comm = kpz*(z-zcommand) + kdz*(zdot) + STICK_MIN;
		ctlcomms.set(1,1,thrust_comm);
		//We are then going to code a roll and pitch contoller
		double roll = sensor_state.get(4,1)*PI/180.0;
		double pitch = sensor_state.get(5,1)*PI/180.0;  //convert to radians
		double yaw = sensor_state.get(6,1)*PI/180.0;
		double p = sensor_state.get(10,1);
		double q = sensor_state.get(11,1);
		double r = sensor_state.get(12,1)*PI/180.0;
		double kpE = -100;
		double kdE = -1000;
		double rollcommand = 0;
		double pitchcommand = 0;
		double yawcommand = 0;
		double roll_comm = kpE*(roll-rollcommand) + kdE*p + STICK_MID;
		double pitch_comm = kpE*(pitch-pitchcommand) + kdE*q + STICK_MID;
		double yaw_comm = kpE*(yaw-yawcommand) + kdE*r + STICK_MID;
		ctlcomms.set(2,1,roll_comm);
		ctlcomms.set(3,1,pitch_comm);
		ctlcomms.set(4,1,yaw_comm);
		//printf("Yaw Comm = %lf \n",ctlcomms.get(4,1));
	}
	//ctlcomms.disp();
}
