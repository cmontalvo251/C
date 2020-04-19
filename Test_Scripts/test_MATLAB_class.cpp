#include "MATLAB.h"
#include "mathp.h"

int main() {

  MATLAB x,Ivec,I,Iinv;

  x.ones(5,5,"x");
  x.disp();

  Ivec.zeros(3,1,"Ivec");
  Ivec.set(1,1,1);
  Ivec.set(2,1,2);
  Ivec.set(3,1,5);
  Ivec.disp();

  I.diag(Ivec,"I");
  I.disp();
  
  Iinv.inv(I,"Iinv");
  Iinv.disp();

  MATLAB xlin;

  xlin.linspace(1,5,5,"xlin");
  xlin.disp();

  MATLAB sum;
  sum.plus(I,Iinv,"sum");
  sum.disp();

  sum.plus_eq(I);
  sum.disp();

  MATLAB sumscalar;
  sumscalar.plus(I,5,"sumscalar");
  sumscalar.disp();

  sumscalar.plus_eq(10);
  sumscalar.set(3,1,0);
  sumscalar.disp();
  sumscalar.transpose();
  sumscalar.disp();
  
  xlin.transpose();
  printf("sum of xlin = %lf \n",xlin.sum());

  MATLAB Identity;
  Identity.eye(3,"Identity");
  Identity.disp();

  MATLAB mimic;
  mimic.copy(Identity,"mimic");
  mimic.disp();

  mimic.overwrite(sumscalar);
  mimic.disp();

  MATLAB multiply;
  multiply.mult(sumscalar,mimic,"multiply");
  multiply.disp();

  MATLAB scalarmult;
  scalarmult.mult(multiply,5,"scalarmult");
  scalarmult.disp();

  scalarmult.mult_eq(0.1);
  scalarmult.disp();

  double xlimnorm = scalarmult.norm();
  xlin.disp();
  printf("var = %lf \n",xlimnorm);

  MATLAB a;
  a.init(3,1,"a");
  a.set(1,1,1);
  a.set(2,1,2);
  a.set(3,1,3);
  a.disp();
  MATLAB b;
  b.init(3,1,"b");
  b.set(1,1,-3);
  b.set(2,1,2);
  b.set(3,1,5);
  b.disp();
  MATLAB c;
  c.cross(a,b,"c");
  c.disp();

  c.cross_eq(a);
  c.disp();
  


  return 0;
}

// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner
