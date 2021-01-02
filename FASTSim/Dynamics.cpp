#include "Dynamics.h"

//Constructor
Dynamics::Dynamics() {
  
}

void Dynamics::init() {
  #ifdef POINTMASS
  NUMSTATES = 6;
  #endif
  
  //If Simulating you need these variables
  //To keep track of states and derivatives
  //#ifdef SIMONLY
  //State.zeros(NUMSTATES,1,"State Vector");
  //k.zeros(NUMSTATES,1,"Derivative of State Vector");
  //#endif
}

//void Dynamics::setState(MATLAB istate) {
//  State.overwrite(istate);
//}

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
