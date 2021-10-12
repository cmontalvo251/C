/* Sensor Block for FASTSim
*/

#include "sensors.h"

//Constructor
sensors::sensors() {
	//The constructor must create these 3x1 vectors
	ptp.zeros(3,1,"Roll Pitch Yaw");	
	ptpdot.zeros(3,1,"Euler Angle Derivatives");
	latlonalt.zeros(3,1,"Latitude Longitude Altitude");
	pqr.zeros(3,1,"PQR");
	q0123.zeros(4,1,"Quaternions");
	xyz.zeros(3,1,"XYZ");
	uvw.zeros(3,1,"XYZ");
	xyzdot.zeros(3,1,"xyzdot");
	uvwdot.zeros(3,1,"uvwdot");
	pqrdot.zeros(3,1,"pqrdot");
	errstate.zeros(12,1,"Full State From Sensors");
	errstatedot.zeros(12,1,"Full Statedot From Sensors");
}

//This only gets called if you have Navio plugged in
void sensors::initSensors(int IMUTYPE) {
  orientation.init(IMUTYPE); //0 for MPU and 1 for LSM
}

///Overloaded sensor routine that reads 
///sensors on board Navio
void sensors::readSensors(double time,double dt) {

  ///////////////////?FIRST POLL ALL SENSORS////////////////////////
  
  ///Read the IMU (ptp,pqr)
  double s = 0.0; //0 for no filtering and 1.0 for overfiltering
  orientation.loop(dt,s);
  //Later we can do the following
  //satellites.poll(time,0);
  //satellites.latitude/longitude/altitude - convertGPS2XY() and computeSpeed(double)
  //barotemp.poll(time);
  //analog.get_results();
  //barotemp.pressure
  //barotemp.temperature
  //analog.results is an array of length analog.channel_count

  /////////////////////////////////////////////////////////////////

  ///////////////////??THEN POPULATE STATE VECTOR////////////////////

  //Initialize everything to -99
  errstate.mult_eq(0);
  errstate.plus_eq(-99);
  errstatedot.mult_eq(0);
  errstatedot.plus_eq(-99);

  ///XYZ
  //errstate

  //PTP
  errstate.set(4,1,orientation.roll);
  errstate.set(5,1,orientation.pitch);
  errstate.set(6,1,orientation.yaw);

  //UWV

  //PQR
  errstate.set(10,1,orientation.roll_rate);
  errstate.set(11,1,orientation.pitch_rate);
  errstate.set(12,1,orientation.yaw_rate);

  //XYZDOT
  //???

  //PTPDOT
  errstatedot.set(4,1,orientation.roll_rate);
  errstatedot.set(5,1,orientation.pitch_rate);
  errstatedot.set(6,1,orientation.yaw_rate);

  //UVWDOT
  //???

  //PQRDOT
  //????
  
}

void sensors::readSensors(MATLAB state,MATLAB statedot) {
	//This is an overloaded function. In here we don't need to read the states from
	//onboard sensors. We just need to copy over the state vector for now.
	//eventually we will add sensor errors but for now we just need to convert the 
	//quaternions to ptp
	//vecset(int in_start,int in_end,MATLAB A,int A_start)
  	//printf("QUATS NO ERRORS \n");
  	//q0123.disp();
	q0123.vecset(1,4,state,4);
	ine2bod321.L321(q0123,1);
	ine2bod321.getptp(ptp);
	//printf("PTP NO ERRORS \n");
	//ptp.disp();

	//Oh and PTP needs to be in degrees
	ptp.mult_eq(180.0/PI);

	//Extract xyz,uvw,pqr
	xyz.vecset(1,3,state,1);
	uvw.vecset(1,3,state,8);
	pqr.vecset(1,3,state,11);

	//PQR also needs to be in degrees because of IMU.cpp in HIL
	pqr.mult_eq(180.0/PI);

	//For the state derivatives the easiest thing in my opinion would be to 
	//compute ptpdot from the H matrix.
	ptpdot.mult(ine2bod321.H,pqr);
	xyzdot.vecset(1,3,statedot,1);
	uvwdot.vecset(1,3,statedot,8);
	pqrdot.vecset(1,3,statedot,11);

	//printf("PTP BEFORE ERRORS \n");
	//ptp.disp();

	//if the flag is set in Simulation_Flags.txt
	if (ADD_ERRORS == 1) {
		//So what sensors are on board the vehicle?
		double original,bias,noise,polluted;
		for (int idx = 1;idx<=3;idx++) {
			//The IMU which measures phi, theta and psi. So we need a bias term
			//and a noise term for those angles
			original = ptp.get(idx,1);
			bias = bias_Angles.get(idx,1);
			noise = randnum(-noise_Angle,noise_Angle);
			polluted = original + bias + noise;
			ptp.set(idx,1,polluted);
			//The IMU also measure pqr - angular velocity - so we need another bias 
			//value and noise term
			original = pqr.get(idx,1);
			bias = bias_Gyros.get(idx,1);
			noise = randnum(-noise_Gyro,noise_Gyro);
			polluted = original + bias + noise;
			pqr.set(idx,1,polluted);
		}
	}

	//printf("PTP AFTER ERRORS \n");
	//ptp.disp();

	//Copy everything over
	errstate.vecset(1,3,xyz,1);
	errstate.vecset(4,6,ptp,1);
	errstate.vecset(7,9,uvw,1);
	errstate.vecset(10,12,pqr,1);
	errstatedot.vecset(1,3,xyzdot,1);
	errstatedot.vecset(4,6,ptpdot,1);
	errstatedot.vecset(7,9,uvwdot,1);
	errstatedot.vecset(10,12,pqrdot,1);
	
	//state.disp();
	//statedot.disp();
	//errstate.disp();
	//errstatedot.disp();
	//PAUSE();

}

void sensors::initSensorErr(MATLAB sensordata) {
	//By calling this function you automatically are turning
	//Errors on in the dynamic model
	ADD_ERRORS = 1;

	//Bias value of the angular measurements
	bias_Angle = sensordata.get(1,1);
	std_bias_Angle = sensordata.get(2,1);
	noise_Angle = sensordata.get(3,1);
	//PQR
	bias_Gyro = sensordata.get(4,1);
	std_bias_Gyro = sensordata.get(5,1);
	noise_Gyro = sensordata.get(6,1);

	//The bias is going to be different for the 3 axes
	bias_Angles.zeros(3,1,"bias Angles");
	bias_Gyros.zeros(3,1,"bias Gyros");
	double bias;
	for (int idx = 1;idx<=3;idx++) {
		//First set the bias of the angles
		bias = bias_Angle + randnum(-std_bias_Angle,std_bias_Angle);
		bias_Angles.set(idx,1,bias);
		//Then the bias of the gyro
		bias = bias_Gyro + randnum(-std_bias_Gyro,std_bias_Gyro);
		bias_Gyros.set(idx,1,bias);
	}
}
