#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "MATLAB.h";
#include "environment.h";

class Dynamics {
 private:
  //Private functions and vars
  MATLAB State,k,I;
  environment env;
  double m;
 public:
  //Public Functions and vars
  int NUMSTATES;
  //void setState(MATLAB istate);
  void Derivatives(MATLAB State,MATLAB k);
  void init();
  void setMassProps(MATLAB massdata);
  void setEnvironment(int G);
  //Constructor
  Dynamics();
};


#endif
