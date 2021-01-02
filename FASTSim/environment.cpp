#include "environment.h"

//constructor
environment::environment() {
}

void environment::init(int G){
  GRAVITY = G;
  FGRAV.zeros(3,1,"FORCE OF GRAVITY");
}

void environment::gravitymodel() {
  FGRAV.mult_eq(0); //zero out gravity

  if (GRAVITY == 1) {
    //Flat Earth model
    FGRAV.set(3,1,GEARTH);
  }
}
