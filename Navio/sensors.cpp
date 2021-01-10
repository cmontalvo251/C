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
	errstatedot.zeros(12,1,"Full State From Sensors");
}

void sensors::readSensors() {
	///Read the IMU (ptp,pqr)

	//Read the GPS (lat/lon/alt)

	//Read the Pitot Probes (uvw)
}

void sensors::readSensors(MATLAB state,MATLAB statedot) {
	//This is an overloaded function. In here we don't need to read the states from
	//onboard sensors. We just need to copy over the state vector for now.
	//eventually we will add sensor errors but for now we just need to convert the 
	//quaternions to ptp
	//vecset(int in_start,int in_end,MATLAB A,int A_start)
	q0123.vecset(1,4,state,4);
	ine2bod321.L321(q0123,1);
	ine2bod321.getptp(ptp);

	//Then we just copy everything over
	xyz.vecset(1,3,state,1);
	uvw.vecset(1,3,state,8);
	pqr.vecset(1,3,state,11);
	errstate.vecset(1,3,xyz,1);
	errstate.vecset(4,6,ptp,1);
	errstate.vecset(7,9,uvw,1);
	errstate.vecset(10,12,pqr,1);

	//For the state derivatives the easiest thing in my opinion would be to 
	//compute ptpdot from the H matrix.
	ptpdot.mult(ine2bod321.H,pqr);
	xyzdot.vecset(1,3,statedot,1);
	uvwdot.vecset(1,3,statedot,8);
	pqrdot.vecset(1,3,statedot,11);
	errstatedot.vecset(1,3,xyzdot,1);
	errstatedot.vecset(4,6,ptpdot,1);
	errstatedot.vecset(7,9,uvwdot,1);
	errstatedot.vecset(10,12,pqrdot,1);
}