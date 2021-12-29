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

		//For now just pitch and roll commands
		double roll_command = (aileron-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
		double pitch_command = -(elevator-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);

		//Get States
		double roll = sensor_state.get(4,1);
		double pitch = sensor_state.get(5,1);
		double roll_rate = sensor_state.get(10,1); //For SIL/SIMONLY see Sensors.cpp
		double pitch_rate = sensor_state.get(11,1); //These are already in deg/s
		//printf("PQR Rate in Controller %lf %lf %lf \n",roll_rate,pitch_rate,yaw_rate);
		double kp = 10.0;
		double kd = 2.0;
		double aileron = kp*(roll-roll_command) + kd*(roll_rate);
		aileron = -CONSTRAIN(aileron,-500,500) + STICK_MID;
		double elevator = kp*(pitch-pitch_command) + kd*(pitch_rate);
		elevator = CONSTRAIN(elevator,-500,500) + STICK_MID;

		ctlcomms.set(2,1,aileron);
		ctlcomms.set(3,1,elevator);
	}
}
