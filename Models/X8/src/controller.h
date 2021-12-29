#ifndef CONTROLLER_H
#define CONTROLLER_H

//This is another class that is craft dependent as as such
//must adhere to specific standards

#include <MATLAB/MATLAB.h>
#include <RCIO/RCInput.h> //this is for STICK values

class controller {
private:
public:
	double timeElapsed = 0; //These are used to keep track of time elapsed.
	double lastTime = 0; //same with this one
	MATLAB ctlcomms; //This is a vector of TAERA1A2 in PWM signals
	//time is in seconds
	//state is a standard 12x1 vector
	//statedot is a standard 12x1 vector of derivatives
	//rxcomms is a TAERA1A2 of PWM signals from a receiver
	int CONTROLLER_FLAG = 0;
	///You must set the number of signals here or the simulation
	//will not work properly.
	int NUMSIGNALS=8; //motor 1,2,3,4,5,6,7,8
	void loop(double t,MATLAB state,MATLAB statedot,int* rxcomms);
	void setup(MATLAB var); //var is open ended right now
	void print();
	controller(); //constructor
};

#endif
