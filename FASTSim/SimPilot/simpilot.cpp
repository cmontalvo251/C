#include <iostream> //these are standard includes
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
using namespace std;

//Include the RCIN class
#include <RCIO/RCInput.h>
RCInput rcin;

//Include the RCOUT class
#include <RCIO/RCOutput.h>
RCOutput rcout;

//Include the IMU Class
#include <IMU/IMU.h>
IMU orientation;

//Include the Controller Class
#include <controller.h>
controller ctl;
MATLAB state,statedot;

//Get a Timer
//time and clock are reserved variables so I used watch
#include <Timer/timer.h>
TIMER watch;

int main(int argc,char** argv) {
	printf("Running Simple Autopilot (SimPilot) Demo \n");

	//You then need to initialize the RCIN by running initialize
	rcin.initialize(); //The default is 8 input channels

	//Initialize the rcout class because this is outputting signals to the PWM ports
	rcout.initialize(4);

	//Select an IMU
	orientation.init(0); //0 for MPU and 1 for LSM

	//We create a loop to write stuff
	watch.resetStartTime();

	//Create dummy vars for controller
	state.zeros(12,1,"state");
	statedot.zeros(12,1,"statedot");

	while (1) {
		//Update Timer
		watch.updateTime();
		watch.printTime();

		////////////////////USER INPUT (Xc)////////////////////////////
		//Poll Receiver - rcin
		rcin.readRCstate();
		rcin.printRCstate(-5); //to notify user of status (-4 is to only print first 4 rx vals)

		////////////////////SENSOR BLOCK (H)//////////////////////////
		double s = 0.0; //0 for no filtering and 1.0 for overfiltering
		orientation.loop(watch.elapsedTime,s);
		orientation.printALL();

		///Put vars into state
		state.set(4,1,orientation.roll);
		state.set(5,1,orientation.pitch);
		state.set(6,1,orientation.yaw);
		state.set(10,1,orientation.roll_rate);
		state.set(11,1,orientation.pitch_rate);
		state.set(12,1,orientation.yaw_rate);

		////////////////////CONTROL BLOCK (C)////////////////////////
		//PID Controller For Quadcopter
		ctl.loop(watch.currentTime,state,statedot,rcin.rxcomm);
		ctl.print();

		////////////////////ACTUATOR OUTPUT (u)/////////////////////
		///Send the signals from the controller to the rcout class
		for (int i = 0;i<rcout.NUMSIGNALS;i++){
		  rcout.pwmcomms[i] = ctl.ctlcomms.get(i+1,1);
		}
		rcout.write();

		////////////////////////////////////////////////////////////
		printf("\n");
	}

	return 0;
}
