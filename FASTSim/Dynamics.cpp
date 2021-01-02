#include "Dynamics.h"

//Constructor
Dynamics::Dynamics() {
  
}

void Dynamics::init() {
  #ifdef POINTSPACE
  NUMSTATES = 6;
  #endif
  
  //If Simulating on the desktop you need these variables
  //To keep track of states and derivatives
  #ifdef DESKTOP
  //State.zeros(NUMSTATES,1,"State Vector");
  //k.zeros(NUMSTATES,1,"Derivative of State Vector");
  #endif
}

//void Dynamics::setState(MATLAB istate) {
//  State.overwrite(istate);
//}

void Dynamics::Derivatives(MATLAB State,MATLAB k) {
  //The Derivatives are vehicle specific

  //Dynamics boils down to F=ma and M=Ia so we need a force a moment model

  ///////////And then finally an acceleration model////////////

  ///This is a point mass in space so no external forces
  #ifdef POINTSPACE
  for (int i = 1;i<=3;i++){
    k.set(i,1,State.get(i+3,1));
  }
  #endif

  ////////////////////////////////////////////////////////////////
}
