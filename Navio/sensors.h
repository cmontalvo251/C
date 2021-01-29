#ifndef SENSORS_H
#define SENSORS_H

#include "MATLAB.h"
#include "Rotation3.h"

class sensors {
private:
	int ADD_ERRORS = 0;
public:
	Rotation3 ine2bod321;
	MATLAB ptp,ptpdot,latlonalt,pqr,q0123,xyz,uvw,errstate,errstatedot,pqrdot;
	MATLAB xyzdot,uvwdot;
	MATLAB bias_Gyros,bias_Angles;
	double bias_Angle,bias_Gyro,std_bias_Gyro,std_bias_Angle;
	double noise_Angle,noise_Gyro;
	//The reason why these sensors are overloaded is because the state vector does not exist
	//if we're running in AUTO mode
	void readSensors(); //calls onboard sensors
	void readSensors(MATLAB state,MATLAB statedot); //overloaded function for onboard sensors
	void initSensorErr(MATLAB sensordata);
	//constructor
	sensors();
};


#endif