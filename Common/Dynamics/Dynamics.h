#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <MATLAB/MATLAB.h>
#include <Rotation/Rotation3.h>
//No matter what we need a way to receive commands from the PIC
//The rcin class can handle anything we throw at it. 
//Joystick commands or even a receiver on the raspberry pi or an Arduino
//If running in SIMONLY the rcin class just acts as a place holder for variables
#include <RCIO/RCInput.h>

//The RCOutput class is for sending PWM signals.
//It's possible we may only need this when in AUTO mode
//but for now we will include it all the time
#include <RCIO/RCOutput.h> 

//These two includes are craft dependent.
//There is an open source portal cube example among others
//in the Models folder
#include "forces.h" 
#include "controller.h"

/*This sensor block does alot of different things. If we're running
SIMONLY or SIL these will call ficitious sensor blocks.
Even in HIL they will call fictitious sensor blocks. 
Only in AUTO will we call the onboard sensors
*/
#include <Sensors/sensors.h>

#ifdef RPI
#ifndef SIMONLY
#define TELEMETRY
//Import telemetry class
#include <Serial/Telemetry.h>
#endif
#endif

class environment {
 private:
  int GRAVITY_FLAG;
  double mass;
 public:
   MATLAB FGRAVI,FGNDI,MGNDI;
   void init(int G);
   void gravitymodel();
   void groundcontactmodel(MATLAB,MATLAB);
   void setMass(double);
   environment(); //constructor   
};

class Dynamics {
 private:
  //Private functions and var
  /////TELEMETRY IF RPI AND NOT SIMONLY
  #ifdef TELEMETRY
  Telemetry serial;
  float number_array[MAXFLOATS]; //MAXFLOATS is set to 10 in Telemetry.h as of 7/16/2021
  int number_Telemetry_vars = 7;
  #endif
  MATLAB State,k,I,pqr,cgdotI,cgdotB,ptpdot,FTOTALI,FTOTALB,FGNDB,MGNDB,MTOTALI,MTOTALB;
  MATLAB pqrdot,Iinv,q0123,I_pqr,uvwdot,pqrskew_I_pqr,Kuvw_pqr,state,statedot;
  MATLAB actuatorState,actuatorStatedot,actuatorErrorPercentage;
  MATLAB actuatorTimeConstants;
  double m,tlastRCread=-99,tlastCTL=-99,tRC,tCTL;
  Rotation3 ine2bod321;
  environment env;
  double ACTUATOR_ERROR_PERCENT;
 public:
  //Public Functions and vars
  MATLAB cg,ptp;
  MATLAB actuatorICs,actuatorError;
  RCInput rcin;
  RCOutput rcout;
  controller ctl;
  forces extforces;
  sensors err;
  int NUMSTATES,NUMLOGS,CONTROLLER_FLAG_INITIAL,NUMACTUATORS=0,NUMVARS;
  void setState(MATLAB state,MATLAB statedot);
  void Derivatives(double time,MATLAB State,MATLAB k);
  void initExtModels(int G);
  void initExtForces(int F);
  void setMassProps(MATLAB massdata);
  void initErrModel(MATLAB sensordata);
  void loop(double time,double dt);
  void printRC(int all);
  void setRates(double,double,double);
  void initActuators(MATLAB);
  void initActuators(int);
  void initController(int);
  void initStateVector();
  void rcio_init();
  //Constructor
  Dynamics();
};


#endif
