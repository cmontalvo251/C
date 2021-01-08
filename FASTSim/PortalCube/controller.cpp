//This is a portal cube template. You must adhere to these standards if you write your
//own


#include "controller.h"

controller::controller() {
	ctlcomms.zeros(6,1,"PWM Control Signals"); //The standards must be TAERA1A2
	ctlcomms.plus_eq(1500); //these are in units of PWM signals so the default is 1500
};

void controller::loop(double time,MATLAB state,MATLAB statedot,int* rxcomms) {
	//At a minimum you need to just feed through the rxcomms into the ctlcomms
	for (int idx=0;idx<6;idx++){
		ctlcomms.set(idx+1,1,rxcomms[idx]);
	}
}