///BaroTemp Class
#include "BaroTemp.h"

BaroTemp::BaroTemp() {
  #ifndef DESKTOP
  barometer.initialize();
  #else
  printf("Running Fictitious Barometer on Desktop \n");
  #endif
}

void BaroTemp::poll(double currentTime) {
  if (PHASE == 0) {
    if ((currentTime - updatetime) > LOOP_TIME) {
      barometer.refreshPressure();
      PHASE = 1;
      updatetime = currentTime;
    }
  }
  if (PHASE == 1) {
    if ((currentTime - updatetime) > SLEEP_TIME) {
      barometer.readPressure();
      barometer.refreshTemperature();
      PHASE = 2;
      updatetime = currentTime;
    }
  }
  if (PHASE == 2) {
    if ((currentTime - updatetime) > SLEEP_TIME) {
      barometer.readTemperature();
      barometer.calculatePressureAndTemperature();
      temperature = barometer.getTemperature();
      pressure = barometer.getPressure();
      updatetime = currentTime;
      PHASE = 0;
    }
  }
}
