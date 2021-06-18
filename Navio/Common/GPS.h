#ifndef GPS_H
#define GPS_H

#include <Common/Ublox.h>
#include <mathp.h>
#include <MATLAB/MATLAB.h>

#define GPSPERIOD 0.5
#define NGPS 25

class GPS {
 public:
  std::vector<double> pos_data,nav_data;
  //X AND Y are Hardcoded to be zero initially and the origin point
  //is roughly set to Mobile
  double latitude,longitude,altitude,X=0,Y=0,Z=0,xprev,yprev,zprev,X_origin=30.69,Y_origin=-88.17;
  Ublox sensor;
  int ok;
  int end_pt = NGPS;
  int start_pt = 1;
  MATLAB dist_vec;
  MATLAB time_vec;
  double speed,dist;
  unsigned long lastTime = 0;
  GPS(); //constructor
  void poll(float,int);
  int status();
  void computeSpeed(double);
  void ConvertGPS2XY();
  void ConvertXYZ2LLH();
  void setXYZ(double,double,double);
};

#endif
