#include "timer.h"
#include <math.h>

int main() {
  
  TIMER track_time;
  double timeElapsed;
  while (1) {
    timeElapsed = track_time.getTimeSinceStart();
    printf("Seconds Elapsed = %lf \n",timeElapsed);
  }
}
