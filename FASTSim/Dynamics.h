#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "MATLAB.h";
#include "environment.h";
#include "Rotation3.h"
//No matter what we need a way to receive commands from the PIC
//The rcin class can handle anything we throw at it. 
//Joystick commands or even a receiver on the raspberry pi or an Arduino
//If running in SIMONLY the rcin class just acts as a place holder for variables
#include "RCInput.h" 

//These two includes are craft dependent.
//There is an open source portal cube example. The aircraft and 
//quad, etc are all in private repos
#include "aerodynamics.h" 
#include "controller.h"

class Dynamics {
 private:
  //Private functions and vars
  MATLAB State,k,I,pqr,cgdotI,cgdotB,ptpdot,FTOTALI,FTOTALB,MTOTALI,MTOTALB;
  MATLAB pqrdot,Iinv,q0123,I_pqr,uvwdot,pqrskew_I_pqr,Kuvw_pqr;
  double m;
  Rotation3 ine2bod321;
  environment env;
  aerodynamics aero;
  RCInput rcin;
  controller ctl;
 public:
  //Public Functions and vars
  MATLAB cg,ptp;
  int NUMSTATES;
  void setState(MATLAB);
  void Derivatives(double time,MATLAB State,MATLAB k);
  void initExternalModels(int G,int A);
  void setMassProps(MATLAB massdata);
  void loop(double time,MATLAB State,MATLAB Statedot);
  void controlloop(double time,MATLAB State,MATLAB Statedot);
  //Constructor
  Dynamics();
};


#endif
