#ifndef BAROTEMP_H
#define BAROTEMP_H

#include <pthread.h>
#include <Common/MS5611.h>

//This can be threaded or not

class BarometerTemperature {
 public:
  float Altitude,Temperature,Pressure;
  float P_avg, T_avg;
  unsigned long lastTime = 0;
  int THREAD = 0;
  int NREADINGS = 1;
  MS5611 sensor;
  pthread_t sensor_thread;
  void ComputeAveragePT();
  void wait();
  void refresh();
  void read();
  void process();
  void refreshT();
  void refreshP();
  void readT();
  void readP();
};

void *acquireMS5611Data(void*);
void setupMS5611(BarometerTemperature*);

#endif
