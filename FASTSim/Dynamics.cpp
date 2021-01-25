#include "Dynamics.h"

//Constructor
Dynamics::Dynamics() {
  NUMSTATES = 13; //Using Quaternions
  //Always create these just because I don't want to think about
  //when we actually need them and it's really easy to create them
  cg.zeros(3,1,"Center of Mass");
  ptp.zeros(3,1,"Roll Pitch Yaw");
  FTOTALI.zeros(3,1,"Total Forces Inertial Frame");

  //Six DoF Model is always running
  q0123.zeros(4,1,"Quaternions");
  cgdotI.zeros(3,1,"Velocity Inertial");
  cgdotB.zeros(3,1,"Velocity Body Frame");
  ptpdot.zeros(3,1,"Euler Derivatives");
  pqr.zeros(3,1,"Angular Velocity Body Frame");
  pqrdot.zeros(3,1,"Derivative of Angular Velocity");
  uvwdot.zeros(3,1,"Derivaitves of Velocity Body Frame");
  //Initialize Forces and Moments
  FTOTALB.zeros(3,1,"Total Forces Body Frame");
  MTOTALI.zeros(3,1,"Total Moments Inertial Frame");
  MTOTALB.zeros(3,1,"Total Moments Body Frame");
  I_pqr.zeros(3,1,"I times pqr");
  pqrskew_I_pqr.zeros(3,1,"pqr cross I times pqr");
  Kuvw_pqr.zeros(3,1,"uvw cross pqr");
  state.zeros(13,1,"Vehicle state vector");
  statedot.zeros(13,1,"Vehicle statedot vector");

  //Run the initialize routine for RCInput so we can compute the number of logvars;
  rcin.initialize();

  //Number of states to log
  //13 states + time + 8 channels or so on rcin
  NUMLOGS = NUMSTATES + 1 + rcin.num_of_axis;

}

void Dynamics::setState(MATLAB state_in,MATLAB statedot_in) {
  //First 3 states are inertial position
  cg.set(1,1,state.get(1,1));
  cg.set(2,1,state.get(2,1));
  cg.set(3,1,state.get(3,1));
  //next 4 states are the quaternions
  q0123.set(1,1,state.get(4,1));
  q0123.set(2,1,state.get(5,1));
  q0123.set(3,1,state.get(6,1));
  q0123.set(4,1,state.get(7,1));
  //We need to convert the quaternions to ptp
  ptp.quat2euler(q0123);
  //next 3 states are the xbody velocities
  cgdotB.set(1,1,state.get(8,1));
  cgdotB.set(2,1,state.get(9,1));
  cgdotB.set(3,1,state.get(10,1));
  //The next 3 states are the pqr angular velocities
  pqr.set(1,1,state.get(11,1));
  pqr.set(2,1,state.get(12,1));
  pqr.set(3,1,state.get(13,1));
  //Also set the entire state vector
  state.overwrite(state_in);
  statedot.overwrite(statedot_in);
}

void Dynamics::initExtModels(int G,int A,int C) {
  //If the AERO Model is on we initialize the aero model
  MATLAB var;
  var.zeros(1,1,"ext model vars");
  if (A) {
    var.set(1,1,A); //Sending Aerodynamics to this var 
    aero.setup(var);
  }
  //Initialize the Gravity model no matter what. The type of model is handled
  //inside this init routine
  env.init(G);
  //Initialize control system model
  CONTROLLER_FLAG_INITIAL = C; //The ctl.loop routine can change the controller flag
  //in the ctl.loop so we need to save the initial control value in case we need
  //it for SIMONLY.
  if (C) {
    var.set(1,1,C);
    ctl.setup(var);
  }
}

void Dynamics::setMassProps(MATLAB massdata) {
  m = massdata.get(1,1);
  I.zeros(3,3,"Inertia");
  I.set(1,1,massdata.get(2,1));
  I.set(2,2,massdata.get(3,1));
  I.set(3,3,massdata.get(4,1));
  Iinv.overwrite(I,"Iinv");
  Iinv.inverse();
}

void Dynamics::loop(double t) {
  
  ////////////////////Poll External Inputs////////////////////////////
  rcin.readRCstate();    
  ////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////
  /*If you're running in SIMONLY. The Input_Files/Simulation_Flags.txt
  controls the CONTROLLER_FLAG. Since whomever is writing the controller.cpp
  has jurisdiction on what to do the best way is to simply set Aux1 to HIGH.
  In this way when you call the control loop you can turn the control
  system on or off if Aux1 is HIGH. It's also possible you're running
  in SIL mode and the joystick did not initialize
  */
  #if defined (SIMONLY) || (SIL)
  if ((CONTROLLER_FLAG_INITIAL == 1) && (rcin.joy_fd == -1)) {
    rcin.rxcomm[4] = STICK_MAX;
  }
  #endif
  //////////////////////////////////////////////////////////////////////

  ///////////////////////Call the Sensor Block////////////////////////
  #ifdef AUTO
  err.readSensors(); //this calls onboard sensors on the Navio
  #else
  err.readSensors(state,statedot); //this simulates sensors
  #endif
  ///////////////////////////////////////////////////////////////////
  
  //////////////////////////////////////////////////////////////////

  ////////////////////Call the Control loop////////////////////////
  ctl.loop(t,err.errstate,err.errstatedot,rcin.rxcomm);
  /////////////////////////////////////////////////////////////////

  /////////////////////Saturation Block/////////////////////////////////
  /*Need to call a saturation block to make sure we don't send a 
  command that's too big. This needs to happen here in the dynamics
  routine because the controller.cpp is written by someone else.
  */
  saturation_block();
  /////////////////////////////////////////////////////////////////
}

void Dynamics::saturation_block() {
  for (int idx=0;idx<ctl.NUMSIGNALS;idx++) {
    double val = ctl.ctlcomms.get(idx+1,1);
    if (val > STICK_MAX) {
      ctl.ctlcomms.set(idx+1,1,STICK_MAX);
    }
    if (val < STICK_MIN) {
      ctl.ctlcomms.set(idx+1,1,STICK_MIN);
    }
  }
}

void Dynamics::printRC(int all) {
  rcin.printRCstate(all);  
}

void Dynamics::Derivatives(double t,MATLAB State,MATLAB k) {
  //The Derivatives are vehicle specific

  //Dynamics boils down to F=ma and M=Ia so we need a force a moment model

  ////////////////FORCE AND MOMENT MODEL///////////////////////

  //Aerodynamic Model
  aero.ForceMoment(t,State,k,ctl.ctlcomms);

  //Gravity Model
  env.gravitymodel();

  ///////////And then finally an acceleration model////////////
  //Extract individual States from State Vector
  cgdotB.vecset(1,3,State,8);
  q0123.vecset(1,4,State,4);
  pqr.vecset(1,3,State,11);
  double q0 = q0123.get(1,1);
  double q1 = q0123.get(2,1);
  double q2 = q0123.get(3,1);
  double q3 = q0123.get(4,1);
  double p = pqr.get(1,1);
  double q = pqr.get(2,1);
  double r = pqr.get(3,1);
  //This is used to rotate things from body to inertial and vice versa
  ine2bod321.L321(q0123, 1);

  ///Linear Kinematics (xyzdot = TIB uvw)
  ine2bod321.rotateBody2Inertial(cgdotI,cgdotB);
  k.vecset(1,3,cgdotI,1);

  ///Check for ground plane
  double z = State.get(3,1);
  double zdot = k.get(3,1);
  if ((z > 0) && (zdot > 0)) {
    k.set(3,1,0);
  }

  ///Rotational Kinematics (Quaternion Derivatives)
  k.set(4,1,(-p*q1-q*q2-r*q3)/2.0);
  k.set(5,1,(p*q0+r*q2-q*q3)/2.0);
  k.set(6,1,(q*q0-r*q1+p*q3)/2.0);
  k.set(7,1,(r*q0+q*q1-p*q2)/2.0);

  //Add Up Forces and Moments
  FTOTALI.overwrite(env.FGRAVI); //add gravity 

  //Rotate Forces to body frame
  ine2bod321.rotateInertial2Body(FTOTALB,FTOTALI);

  //Add Aero Forces and Moments
  FTOTALB.plus_eq(aero.FAEROB);

  //Translational Dynamics
  Kuvw_pqr.cross(pqr,cgdotB);
  FTOTALB.mult_eq(1.0/m); 
  uvwdot.minus(FTOTALB,Kuvw_pqr); 
  k.vecset(8,10,uvwdot,1);

  //Moments vector
  MTOTALB.overwrite(aero.MAEROB);

  ///Rotational Dynamics
  //pqrskew = [0 -r q;r 0 -p;-q p 0];  
  //pqrdot = Iinv*(LMN-pqrskew*I*pqr);
  //pqr.disp();
  I_pqr.mult(I,pqr);
  //I.disp();
  //I_pqr.disp();
  pqrskew_I_pqr.cross(pqr,I_pqr);
  //pqrskew_I_pqr.disp();
  MTOTALB.minus_eq(pqrskew_I_pqr);
  //LMN.disp();
  //Iinv.disp();
  pqrdot.mult(Iinv,MTOTALB);
  
  //Save pqrdot
  k.vecset(11,13,pqrdot,1);

  ////////////////////////////////////////////////////////////////
}
