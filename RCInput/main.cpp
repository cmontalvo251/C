#include "RCInput.h"

RCInput rcin;

//Using Microsoft X-Box 360 pad 
//Throttle = 1 (inv)
//Rudder = 0 
//Aileron =  3
//Elevator = 4
//Left Trigger = 2
//Right Trigger = 5
//UD Dpad = 7
//LR Dpad = 6

int main() {
	while (1) {
		rcin.readRCstate();
		rcin.printRCstate(1);
	}
	return 0;
};