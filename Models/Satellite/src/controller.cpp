//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"

controller::controller() {
	ctlcomms.zeros(NUMSIGNALS,1,"PWM Control Signals"); //The standards must be TAERA1A2A3A4
	pqr.zeros(3,1,"PQR");
	mxyz.zeros(3,1,"MXYZ");
	desired_moments.zeros(3,1,"desired moment");
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
	ctlcomms.mult_eq(0); //Zero out the currents
	
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
	  //printf("RUNNING DETUMBLING \n");
	  //Extract PQR in (deg/s)
	  pqr.vecset(1,3,sensor_state,10);
	  //Convert to (rad/s)
	  pqr.mult_eq(PI/180.0);
	  //pqr.disp();
	  //Extract Magnetomter Readings
	  mxyz.vecset(1,3,sensor_state,13);
	  //mxyz.disp();
	  //mu_ideal = k*(omega cross B)
	  desired_moments.cross(pqr,mxyz);
	  desired_moments.mult_eq(KMAG);
	  //Convert to currents
	  desired_moments.mult_eq(1.0/(NUMTURNS*AREA));
	  //send to ctlcomms (but might not be 3 actuators)
	  for (int i = 0;i<NUMSIGNALS;i++) {
	    ctlcomms.set(i+1,1,desired_moments.get(i+1,1));
	  }
	  //Saturation on current
	  double sum = ctlcomms.abssum();
	  if (sum > MAXCURRENT) {
	    ctlcomms.mult_eq(MAXCURRENT/sum);
	    // for (int i = 0;i<3;i++) {
	    //   double val = ctlcomms.get(i+1,1);
	    //   ctlcomms.set(i+1,val/sum*maxcurrent);
	    // }
	  }
	} 
	//ctlcomms.disp();
	//printf("SUM = %lf \n",ctlcomms.abssum());
}
