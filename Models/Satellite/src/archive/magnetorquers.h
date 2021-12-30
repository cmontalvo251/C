#ifndef MAGNETORQUERS_H
#define MAGNETORQUERS_H

#include <MATLAB/MATLAB.h>

class Magnetorquers {
  friend class Satellite;
 private:
  int MAGNETORQUER_CONTROLLER_TYPE_,NUMMAGTORQUERS;
  double Moment_Limit,K,UpdateRate_;
  double xcurrent, ycurrent, zcurrent, areamagnetorque, magnnumturn, maxcurrent;
  MATLAB XMTVEC, YMTVEC, ZMTVEC, MTVEC, XMMTVEC, YMMTVEC, ZMMTVEC, MMTVEC;
  MATLAB BVEC_Tesla,BVEC_Tesla_OLD,pqr,quat,ptp;
  MATLAB CURRENT_VEC,LMN,BVEC_,inertia;
  MATLAB KAPPA,KAPPA_inv,xycurrent,YZdesired;
  MATLAB currents,desired_magmoments;

  //functions
  double getCurrent(int);
  void compute_moments();
  void UpdateMagneticField(MATLAB);
  
 public:
  Magnetorquers(char*); //constructor
};

#endif
