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
}

void Dynamics::setState(MATLAB state) {
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
}

void Dynamics::initExternalModels(int G,int A) {
  //If the AERO Model is on we initialize the aero model
  if (A) {
    MATLAB var;
    var.zeros(1,1,"aero model vars");
    var.set(1,1,A); //Sending Aerodynamics to this var 
    aero.setup(var);
  }
  //Initialize the Gravity model no matter what. The type of model is handled
  //inside this init routine
  env.init(G);
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

void Dynamics::loop(double t,MATLAB State,MATLAB Statedot) {
  ////////////////////Poll External Inputs////////////////////////////
  rcin.readRCstate();    
  //////////////////////////////////////////////////////////////////

  ////////////////////Call the Control loop////////////////////////
  //This only happens once every timestep
  controlloop(t,State,Statedot);
  /////////////////////////////////////////////////////////////////
}

void Dynamics::controlloop(double time,MATLAB State,MATLAB Statedot) {
  ctl.loop(time,State,Statedot,rcin.axis);
}      

void Dynamics::Derivatives(double time,MATLAB State,MATLAB k) {
  //The Derivatives are vehicle specific

  //Dynamics boils down to F=ma and M=Ia so we need a force a moment model

  ////////////////FORCE AND MOMENT MODEL///////////////////////

  //Aerodynamic Model
  aero.ForceMoment(time,State,k,ctl.ctlcomms);

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
