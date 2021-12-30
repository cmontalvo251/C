#ifndef SENSORS_H
#define SENSORS_H

#include <MATLAB/MATLAB.h>
#include <Rotation/Rotation3.h>
#include <Timer/timer.h>

#include <Baro/BaroTemp.h>
#include <GPS/GPS.h>
#include <IMU/IMU.h>
#include <ADC/ADC.h>

class sensors {
private:
  int ADD_ERRORS = 0;
  BaroTemp barotemp;
  GPS satellites;
  IMU orientation;
  ADC analog;
  double gps_heading=-99;
  double IMUbias = 0;
  double compassFilterConstant = 0.2;
public:
  Rotation3 ine2bod321;
  MATLAB ptp,ptpdot,latlonalt,pqr,q0123,xyz,uvw,errstate,errstatedot,pqrdot;
  MATLAB xyzdot,uvwdot,mxyz;
  MATLAB bias_Gyros,bias_Angles,bias_Mags;
  double bias_Angle,bias_Gyro,std_bias_Gyro,std_bias_Angle;
  double bias_Mag,std_bias_Mag;
  double noise_Angle,noise_Gyro,noise_Mag;
  double compass;
  //The reason why these sensors are overloaded is because the state vector does not exist
  //if we're running in AUTO mode
  void readSensors(double,double); //calls onboard sensors
  void readSensors(MATLAB state,MATLAB statedot,MATLAB BVECB,double time); //overloaded function for onboard sensors
  void initSensorErr(MATLAB sensordata);
  void initSensors(int);
  void computeCompassHeading(double,double);
  double getHeading();
  double getLatitude();
  double getLongitude();
  double getAltitude();
  double getPressure();
  double getTemp();
  int adc_channels();
  double getAnalog(int);
  //constructor
  sensors();
};


#endif
