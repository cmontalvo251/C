#include <iostream> //these are standard includes
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
using namespace std;

//Include the RCIN class
#include <RCIO/RCInput.h>
RCInput rcin;

//Include the IMU Class
#include <IMU/IMU.h>
IMU orientation;

//Get a Timer
//time and clock are reserved variables so I used watch
#include <Timer/timer.h>
TIMER watch;

int main(int argc,char** argv) {
	printf("Running Simple Autopilot (SimPilot) Demo \n");

	//You then need to initialize the RCIN by running initialize
	rcin.initialize(); //The default is 8 input channels

	//Select an IMU
	orientation.init(0); //0 for MPU and 1 for LSM

	//We create a loop to write stuff
	watch.resetStartTime();

	while (1) {
		//Update Timer
		watch.updateTime();
		watch.printTime();

		////////////////////USER INPUT (Xc)////////////////////////////
		//Poll Receiver - rcin
		rcin.readRCstate();
		rcin.printRCstate(-4); //to notify user of status

		////////////////////SENSOR BLOCK (H)//////////////////////////
		double s = 0.0; //0 for no filtering and 1.0 for overfiltering
		orientation.loop(watch.elapsedTime,s);
		orientation.printALL();

		////////////////////CONTROL BLOCK (C)////////////////////////
		//PID Controller For Quadcopter

		////////////////////ACTUATOR OUTPUT (u)/////////////////////
		//rcout

		////////////////////////////////////////////////////////////
		printf("\n");
	}

	return 0;
}
