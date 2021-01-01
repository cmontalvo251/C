#include "RK4.h"

//Constructor
RK4::RK4() {

};

void RK4::init(int NUMSTATES,double dt) {
  //Initialize State Vector
  TIMESTEP = dt;
  State.zeros(NUMSTATES,1,"State");
  StateDel.zeros(NUMSTATES,1,"StateDel");
  k.zeros(NUMSTATES,1,"k");
  k1.zeros(NUMSTATES,1,"k1");
  k2.zeros(NUMSTATES,1,"k2");
  k3.zeros(NUMSTATES,1,"k3");
  k4.zeros(NUMSTATES,1,"k4");
  phi.zeros(NUMSTATES,1,"phi");
}


void RK4::set_ICs(MATLAB x0) {
  State.overwrite(x0);
  StateDel.overwrite(x0);
}

void RK4::integrate(int i) {
  //First call
  switch (i) {
  case (1):
    //Derivatives(State,k1);
    k1.overwrite(k);
    StateDel.overwrite(State); //StateDel = State
    StateDel.plus_mult_eq(k1,TIMESTEP/2); //StateDel = StateDel + k1*TIMESTEP/2
    break;
  case (2):
    //Derivatives(StateDel,k2);
    k2.overwrite(k);
    StateDel.overwrite(State); 
    StateDel.plus_mult_eq(k2,TIMESTEP/2);
    break;
  case (3):
    //Third Call
    //Derivatives(StateDel,k3);
    k3.overwrite(k);
    StateDel.overwrite(State);
    StateDel.plus_mult_eq(k3,TIMESTEP);
    break;
  case (4):
    //Fourth Call
    k4.overwrite(k);
    //Derivatives(StateDel,k4);
    //Put it all together
    // phi = k1/6 + k2/3 + k3/3 + k4/6
    phi.mult(k1,1.0/6.0); // phi = k1/6
    phi.plus_mult_eq(k2,1.0/3.0); // phi = phi + k2/3
    phi.plus_mult_eq(k3,1.0/3.0); // phi = phi + k3/3
    phi.plus_mult_eq(k4,1.0/6.0); // phi = phi + k2/3
    //And then State = State + phi*timestep
    State.plus_mult_eq(phi,TIMESTEP);
  }
}
