#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "MATLAB.h";

class Dynamics {
 private:
  //Private functions and vars
  MATLAB State,k;
 public:
  //Public Functions and vars
  int NUMSTATES;
  //void setState(MATLAB istate);
  void Derivatives(MATLAB State,MATLAB k);
  void init();
  //Constructor
  Dynamics();
};


#endif
