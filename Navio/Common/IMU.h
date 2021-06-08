#ifndef IMU_H
#define IMU_H

#include <Common/MPU9250.h>
#include <Navio2/AHRS.h>
#include <mathp.h>

class IMU {
 public:
  InertialSensor *mpu;
  AHRS ahrs;
  IMU();
  float ax=0, ay=0, az=0;
  float gx=0, gy=0, gz=0;
  float mx=0, my=0, mz=0;
  float roll = 0 , pitch = 0 , yaw = 0;
  float gx_filtered=0, gy_filtered=0, gz_filtered=0;
  float offset[3];
  double roll_rate,pitch_rate,yaw_rate;
  void imuSetup();
  void loop(float,float);
};

#endif
