// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner
#ifndef TIMER_H
#define TIMER_H

#include <time.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


void cross_sleep(double);
void PAUSE();
void error(char *);

class TIMER {
 private:
  double start_sec_;
  time_t rawtime_;
  clock_t t_;
  void getCurrentTime();
  double getSeconds();
  struct tm* ptm_;
  struct timespec ts;
 public:
  double getTimeElapsed(); //give time elapsed from subsequent function calls in seconds
  double getTimeSinceStart(); //give time elapsed from subsequent function calls in seconds

  //Constructor
  TIMER();

};

#endif
