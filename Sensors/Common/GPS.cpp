#include "GPS.h"
#include <math.h>

GPS::GPS() {
  if(sensor.testConnection()){
    printf("Ublox test OK\n");
    if(!sensor.configureSolutionRate(1000)){
      printf("Setting new rate: FAILED\n");
    }
  } else {
    printf("GPS Test failed \n");
  }
  dist_vec.zeros(NGPS,1,"dist_vec");
  time_vec.zeros(NGPS,1,"time_vec");
}

void GPS::poll(float currentTime,int FILEOPEN) {
  lastTime = currentTime;
  sensor.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data);
  if (pos_data.size() > 4) {
    latitude = pos_data[2]/10000000.0; //lon - Maxwell says it may be lon lat
    longitude = pos_data[1]/10000000.0; //lat - It really is lon lat
    altitude = pos_data[3]/1000.0; ///height above ellipsoid 1984?
    //If the measurement is good and the file is open we need to compute speed as well.
    if (FILEOPEN) {
      computeSpeed(currentTime);
    } else {
      //otherwise set the origin
      X_origin = latitude;
      Y_origin = longitude;
    }
  } else {
    latitude = -99;
    longitude = -99;
    altitude = -99;
  }
  //printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \n",latitude,longitude,X_origin,Y_origin,X,Y,xprev,yprev,dist,speed);
  //dist_vec.disp();
  //time_vec.disp();
    
}	  

int GPS::status() {
  #ifdef PRINTSEVERYWHERE
  printf("Checking GPS Health \n");
  #endif
  //if (timeSinceStart > 10) {
  sensor.decodeSingleMessage(Ublox::NAV_STATUS, nav_data);
  int size = nav_data.size();
  #ifdef PRINTSEVERYWHERE
  printf("Size of Nav_Data = %d \n",size);
  #endif
  ok = 1;
  if (size > 0) {
    ok = (int(nav_data[0]) == 0x00);
  }
  return ok;
}

void GPS::ConvertGPS2XY(){
  ///CONVERT LAT LON TO METERS
  if (longitude == -99) {
    Y = yprev;
  } else {
    Y = (longitude - Y_origin)*GPSVAL*cos(X_origin*DEG2RAD);  
  }
  if (latitude == -99) {
    X = xprev;
  } else {
    X = (latitude - X_origin)*GPSVAL;
  }
}

void GPS::computeSpeed(double current_time) {
  //First convert the current measurement to XY
  ConvertGPS2XY();
  //Then proceed with the speed measurement
  double dx = X - xprev;
  double dy = Y - yprev;
  dist = sqrt((pow(dx,2)) + (pow(dy,2)));
  int em1;

  //Get previous value
  em1 = end_pt - 1;

  if (em1 == 0) {
    em1 = NGPS;
  }
  
  double previous_distance = dist_vec.get(em1,1);
  double new_distance = dist + previous_distance;
  dist_vec.set(end_pt,1,new_distance);

  time_vec.set(end_pt,1,current_time/1000000.0);

  double del_dist = dist_vec.get(end_pt,1) - dist_vec.get(start_pt,1);
  double del_time = time_vec.get(end_pt,1) - time_vec.get(start_pt,1);

  //printf("%lf %lf %lf %lf %lf %lf \n",dist_vec.get(end_pt,1),dist_vec.get(start_pt,1),time_vec.get(end_pt,1),time_vec.get(start_pt,1),del_dist,del_time);
  
  speed = del_dist/del_time;
  
  end_pt += 1;
  start_pt += 1;
  
  if (end_pt > NGPS) {
    end_pt = 1;
  }
  if (start_pt > NGPS){
    start_pt = 1;
  }

  //UPDATE PREVIOUS VALUES
  xprev = X;
  yprev = Y;
}

