#ifndef BAROTEMP_H
#define BAROTEMP_H

#include <Common/MS5611.h>
#include <Common/Util.h>
#include <unistd.h>
#include <stdio.h>

#define SLEEP_TIME 0.01 //seconds
#define LOOP_TIME 1.0 //seconds

class BaroTemp {
 private:
  MS5611 barometer;
  double updatetime=-99;
  int PHASE=0;
 public:
  BaroTemp(); //constructor
  void poll(double);
  double temperature=-99;
  double pressure=-99;
};

#endif
