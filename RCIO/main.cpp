#include "RCInput.h"

//In the make file you need to have -DJOYSTICK defined

RCInput rcin; //this calls the constructor which does nothing

//Using Microsoft X-Box 360 pad 
//Throttle = 1 (inv)
//Rudder = 0 
//Aileron =  3
//Elevator = 4
//Left Trigger = 2
//Right Trigger = 5
//UD Dpad = 7
//LR Dpad = 6

//Using RCTECH Controller
//Throttle = 2 (inv)
//Rudder = 5
//Aileron =  0
//Elevator = 1
//arm switch = 3
//aux 0 = 4

int main() {

	//You then need to initialize the Controller by running initialize
	rcin.initialize(); //The default is 8 input channels

	while (1) {
		rcin.readRCstate(); // This will read the current state of the USB controller
		rcin.printRCstate(1); //This will print everything including buttons if they have been found
	}
	return 0;
};