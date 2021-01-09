//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"

controller::controller() {
	ctlcomms.zeros(8,1,"PWM Control Signals"); //The standards must be TAERA1A2A3A4
};

void controller::loop(double t,MATLAB state,MATLAB statedot,int* rxcomms) {
	//At a minimum you need to just feed through the rxcomms into the ctlcomms
	for (int idx=0;idx<6;idx++){
		ctlcomms.set(idx+1,1,rxcomms[idx]);
	}
}