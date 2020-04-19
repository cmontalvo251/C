#include "Rotation3.h"

using namespace std;

int main () {

  MATLAB euler_ang;
  euler_ang.zeros(3,1,"euler");
  double phi = 0;
  double theta = PI/4;
  double psi = PI/4;
  euler_ang.set(1,1,phi);
  euler_ang.set(2,1,theta);
  euler_ang.set(3,1,psi);
  
  Rotation3 IB;

  IB.L321(euler_ang,0);

  IB.dispQuat();

  IB.dispEuler();

  IB.disp();

  MATLAB vecI;

  vecI.zeros(3,1,"vecI");

  MATLAB vecB;

  vecB.zeros(3,1,"vecB");

  vecB.set(1,1,1);

  vecB.disp();

  IB.rotate(vecI,vecB);

  vecI.disp();

  IB.rotateInverse(vecB,vecI);

  vecB.disp();
}
