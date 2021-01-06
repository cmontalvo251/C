#include "Dynamics.h"

//Constructor
Dynamics::Dynamics() {
  
}

void Dynamics::init() {
  #ifdef POINTMASS
  NUMSTATES = 6;
  #endif
  //Always create these just because I don't want to think about
  //when we actually need them and it's really easy to create them
  cg.zeros(3,1,"Center of Mass");
  ptp.zeros(3,1,"Roll Pitch Yaw");
}

void Dynamics::setState(MATLAB state) {
  #ifdef POINTMASS
  cg.set(1,1,state.get(1,1));
  cg.set(2,1,state.get(2,1));
  cg.set(3,1,state.get(3,1));
  //ptp is just zeros because it's a point mass
  #endif
}

void Dynamics::setEnvironment(int G) {
  env.init(G);
}

void Dynamics::setMassProps(MATLAB massdata) {
  m = massdata.get(1,1);
  I.zeros(3,3,"Inertia");
  I.set(1,1,massdata.get(2,1));
  I.set(2,2,massdata.get(3,1));
  I.set(3,3,massdata.get(4,1));
}
      

void Dynamics::Derivatives(MATLAB State,MATLAB k) {
  //The Derivatives are vehicle specific

  //Dynamics boils down to F=ma and M=Ia so we need a force a moment model

  ////////////////FORCE AND MOMENT MODEL///////////////////////

  //Gravity Model
  env.gravitymodel();

  ///////////And then finally an acceleration model////////////

  ///This is a point mass with no inertia
  #ifdef POINTMASS
  //Kinematics
  for (int i = 1;i<=3;i++){
    k.set(i,1,State.get(i+3,1));
  }
  //Dynamics
  for (int i = 4;i<=6;i++) {
    k.set(i,1,env.FGRAV.get(i-3,1)/m);
  }
  #endif

  ////////////////////////////////////////////////////////////////
}
