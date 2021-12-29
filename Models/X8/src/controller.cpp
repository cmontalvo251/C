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

void controller::loop(double t,MATLAB state,MATLAB statedot,int* rxcomms) {
	//I want to keep track of timeElapsed so that I can run integrators
	timeElapsed = t - lastTime;
	lastTime = t;
	//printf("Time Elapsed = %lf \n",timeElapsed);

	//At a minimum you need to compute the 8 motor signals based on 
	double motor_upper_left_top = STICK_MIN;
	double motor_upper_right_top = STICK_MIN;
	double motor_lower_left_top = STICK_MIN;
	double motor_lower_right_top = STICK_MIN;
	double motor_upper_left_bottom = STICK_MIN;
	double motor_upper_right_bottom = STICK_MIN;
	double motor_lower_left_bottom = STICK_MIN;
	double motor_lower_right_bottom = STICK_MIN;

	//First extract the relavent commands from the receiver.
	//printf("HERE: ");
	//for (int i = 0;i<4;i++) {
	//	printf("%d ",rxcomms[i]);
	//}
	//printf("\n");
	double throttle = rxcomms[0];
	double aileron = rxcomms[1];
	double elevator = rxcomms[2];
	double rudder = rxcomms[3];
	double autopilot = rxcomms[4];

	//printf("TAER = %lf %lf %lf %lf \n",throttle,aileron,elevator,rudder,autopilot);
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
		printf(" STAB ");
		double roll_command = (aileron-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
		double pitch_command = -(elevator-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
		double yaw_rate_command = (rudder-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
		double roll = state.get(4,1);
		double pitch = state.get(5,1);
		double yaw = state.get(6,1);
		double roll_rate = state.get(10,1); //For SIL/SIMONLY see Sensors.cpp
		double pitch_rate = state.get(11,1); //These are already in deg/s
		double yaw_rate = state.get(12,1); //Check IMU.cpp to see for HIL
		//state.disp();
		//printf("PQR Rate in Controller %lf %lf %lf \n",roll_rate,pitch_rate,yaw_rate);
		double kp = 10.0;
		double kd = 2.0;
		double kyaw = 0.2;
		double droll = kp*(roll-roll_command) + kd*(roll_rate);
		droll = CONSTRAIN(droll,-500,500);
		double dpitch = kp*(pitch-pitch_command) + kd*(pitch_rate);
		dpitch = CONSTRAIN(dpitch,-500,500);
		double dyaw = kyaw*(yaw_rate-yaw_rate_command);
		//printf("YAW RATE = %lf YAW RATE COMMAND = %lf DYAW = %lf \n",yaw_rate,yaw_rate_command,dyaw);
		dyaw = CONSTRAIN(dyaw,-500,500);
		//printf("d = %lf %lf %lf ",droll,dpitch,dyaw);
		//printf(" Roll Command = %lf ",roll_command);
		motor_upper_left_top = throttle - droll - dpitch - dyaw;
		motor_upper_right_top = throttle + droll - dpitch + dyaw;
		motor_lower_left_top = throttle - droll + dpitch + dyaw;
		motor_lower_right_top = throttle + droll + dpitch - dyaw;

		motor_upper_left_bottom = throttle - droll - dpitch + dyaw;
		motor_upper_right_bottom = throttle + droll - dpitch - dyaw;
		motor_lower_left_bottom = throttle - droll + dpitch - dyaw;
		motor_lower_right_bottom = throttle + droll + dpitch + dyaw;

	} else {
		//ACRO MODE
		motor_upper_left_bottom = throttle + (aileron-STICK_MID) - (elevator-STICK_MID) + (rudder-STICK_MID);
		motor_upper_right_bottom = throttle - (aileron-STICK_MID) - (elevator-STICK_MID) - (rudder-STICK_MID);
		motor_lower_right_bottom = throttle - (aileron-STICK_MID) + (elevator-STICK_MID) + (rudder-STICK_MID);
		motor_lower_left_bottom = throttle + (aileron-STICK_MID) + (elevator-STICK_MID) - (rudder-STICK_MID);
		motor_upper_left_top = throttle + (aileron-STICK_MID) - (elevator-STICK_MID) - (rudder-STICK_MID);
		motor_upper_right_top = throttle - (aileron-STICK_MID) - (elevator-STICK_MID) + (rudder-STICK_MID);
		motor_lower_right_top = throttle - (aileron-STICK_MID) + (elevator-STICK_MID) - (rudder-STICK_MID);
		motor_lower_left_top = throttle + (aileron-STICK_MID) + (elevator-STICK_MID) + (rudder-STICK_MID);		
	}

	//Send the motor commands to the ctlcomms values
	//ctlcomms.mult_eq(0);
	//ctlcomms.plus_eq(STICK_MIN);
	ctlcomms.set(1,1,motor_upper_left_bottom);
	ctlcomms.set(2,1,motor_upper_right_bottom);
    ctlcomms.set(3,1,motor_lower_right_bottom);
	ctlcomms.set(4,1,motor_lower_left_bottom);
	ctlcomms.set(5,1,motor_upper_left_top);
	ctlcomms.set(6,1,motor_upper_right_top);
	ctlcomms.set(7,1,motor_lower_right_top);
	ctlcomms.set(8,1,motor_lower_left_top);
	//ctlcomms.disp();
}
