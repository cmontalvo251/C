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
  double latitude,longitude,altitude,X,Y,xprev,yprev,X_origin,Y_origin;
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
};

#endif
