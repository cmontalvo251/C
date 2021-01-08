#ifndef CONTROLLER_H
#define CONTROLLER_H

//This is another class that is craft dependent as as such
//must adhere to specific standards

#include "MATLAB.h"

class controller {
private:
public:
	MATLAB ctlcomms; //This is a vector of TAERA1A2 in PWM signals
	//time is in seconds
	//state is a standard 13x1 vector using quaternions
	//statedot is a standard 13x1 vector of derivatives
	//rxcomms is a TAERA1A2 of PWM signals from a receiver
	void loop(double time,MATLAB state,MATLAB statedot,int* rxcomms);
	void setup(MATLAB var); //var is open ended right now
	controller(); //constructor
};

#endif