#include "environment.h"

//constructor
environment::environment() {
}

void environment::init(int G){
  GRAVITY_FLAG = G;
  FGRAVI.zeros(3,1,"FORCE OF GRAVITY INERTIAL");
}

void environment::gravitymodel() {
  FGRAVI.mult_eq(0); //zero out gravity

  if (GRAVITY_FLAG == 1) {
    //Flat Earth model
    FGRAVI.set(3,1,GEARTH);
  }
}
