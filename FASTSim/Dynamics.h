#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "MATLAB.h";
#include "environment.h";
#include "Rotation3.h"

#if defined (PORTALCUBE) || (AIRCRAFT) || (QUADCOPTER) || (X8)
#define SIXDOF //this turns on 6DOF dynamics. The other defines are for different dynamic models
#endif

class Dynamics {
 private:
  //Private functions and vars
  MATLAB State,k,I,pqr,cgdotI,cgdotB,ptpdot,FTOTALI,FTOTALB,MTOTALI,MTOTALB;
  MATLAB pqrdot,Iinv,q0123,I_pqr,uvwdot,pqrskew_I_pqr,Kuvw_pqr;
  environment env;
  double m;
  Rotation3 ine2bod321;
 public:
  //Public Functions and vars
  MATLAB cg,ptp;
  int NUMSTATES;
  void setState(MATLAB);
  void Derivatives(MATLAB State,MATLAB k);
  void init();
  void setMassProps(MATLAB massdata);
  void setEnvironment(int G);
  //Constructor
  Dynamics();
};


#endif
