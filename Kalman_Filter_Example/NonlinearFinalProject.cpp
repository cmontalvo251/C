#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <cmath>
#include <iostream>
#include <mathp.h>
#include "MATLAB.h"
using namespace std;

//CONSTANTS AND ASSIGNMENTS
#define NUMSTATES 21
#define CREATEARRAY(IN) (double*)malloc(IN*sizeof(double));

MATLAB State,k1,k2,k3,k4,phi,StateDel;
MATLAB I,Imodel,Iinv,Imodelinv,M,Mtrans,MQ,MQMtrans,Hsensor,Htrans;
MATLAB negHsensor,Identity,Pkmin,xkmin,R,ybar,Hsensorkk;
MATLAB HsensorHtrans,xkminHsensorkk,HsensorkkIdentity;
MATLAB K,kinv,kk,KinvHtrans,xkplus,Pkplus,Quat,Quatm,PQR,PQRmodel;
MATLAB negPQR,negPQRm,PQRmatrix,PQRmodelmatrix,Ftilde,Ftrans,P;
MATLAB quatdot,quatmdot,H,Hm ,PQRcrossH,PQRmcrossH;
MATLAB PQRdot,PQRmdot,Pdot,Pdotvec,FtransP;

//GLOBALS
int WRITEON;
double SensorTime,SensorPeriod,q,h,m;
double Ixx,Iyy,Izz,IxxModel,IyyModel,IzzModel,v;

double TFINAL,TINITIAL,TIMEON,TIMESTEP;
time_t rawtime;
struct tm * ptm;

void PAUSE()
{
  printf("%s","Function Paused\n");
  printf("Type in any number(1-inf) and press enter to continue\n");
  int  a;
  scanf("%i",&a);
}

void setup(double p0,double q0,double r0,double pm0,double qm0,double rm0,double Pq00,double Pq10,double Pq20,double Pq30,double Pp0,double Pq0,double Pr0)
{
  WRITEON = 1;
  TINITIAL = 0;
  TFINAL = 50;
  TIMESTEP = 0.001;
  TIMEON = 50;
  SensorTime = -1; //-1 So that a measurement will occur at TIME=0
  SensorPeriod= 1; //1 Second between measurements
  State.zeros(NUMSTATES,1,"State");
  StateDel.zeros(NUMSTATES,1,"StateDel");
  k1.zeros(NUMSTATES,1,"k1");
  k2.zeros(NUMSTATES,1,"k2");
  k3.zeros(NUMSTATES,1,"k3");
  k4.zeros(NUMSTATES,1,"k4");
  phi.zeros(NUMSTATES,1,"phi");

  //Setting Initial Conditions
  //Note: State 1-4 and 8-11 are for the quaternions
  State.set(5,1,p0); 
  State.set(6,1,q0);
  State.set(7,1,r0);
  State.set(12,1,pm0);
  State.set(13,1,qm0);
  State.set(14,1,rm0);
  State.set(15,1,Pq00);
  State.set(16,1,Pq10); 
  State.set(17,1,Pq20);
  State.set(18,1,Pq30);
  State.set(19,1,Pp0);
  State.set(20,1,Pq0);
  State.set(21,1,Pr0);
  //Coefficients//
  q = 10;
  h = 1;
  m = 1;

  Ixx = 5;
  Iyy = 4; 
  Izz = 2;

  IxxModel = 4.8;
  IyyModel = 4.2;
  IzzModel = 1.6;

  //Matrix Initialization

  I.zeros(3,3,"I");
  I.set(1,1,Ixx);
  I.set(2,2,Iyy);
  I.set(3,3,Izz);

  Imodel.zeros(3,3,"Imodel");
  Imodel.set(1,1,IxxModel);
  Imodel.set(2,2,IyyModel);
  Imodel.set(3,3,IzzModel);

  Iinv.zeros(3,3,"Iinv");
  Iinv.matrix_inverse(I,3);

  Imodelinv.zeros(3,3,"Imodelinv");
  Imodelinv.matrix_inverse(Imodel,3);

  M.zeros(7,7,"M");
  M.set(1,1,m);
  M.set(2,2,m);
  M.set(3,3,m);
  M.set(4,4,m);
  M.set(5,5,m);
  M.set(6,6,m);
  M.set(7,7,m);

  Mtrans.zeros(7,7,"Mtrans");
  Mtrans.overwrite(M);
  Mtrans.transpose();

  MQ.zeros(7,7,"MQ");
  MQ.mult(M,q);
  MQMtrans.zeros(7,7,"MQMtrans");
  MQMtrans.mult(MQ,Mtrans);

  Hsensor.zeros(7,7,"Hsensor");
  Hsensor.set(1,1,h);
  Hsensor.set(2,2,h);
  Hsensor.set(3,3,h);
  Hsensor.set(4,4,h);
  Hsensor.set(5,5,h);
  Hsensor.set(6,6,h);
  Hsensor.set(7,7,h);

  negHsensor.zeros(7,7,"Hsensor");
  negHsensor.set(1,1,-h);
  negHsensor.set(2,2,-h);
  negHsensor.set(3,3,-h);
  negHsensor.set(4,4,-h);
  negHsensor.set(5,5,-h);
  negHsensor.set(6,6,-h);
  negHsensor.set(7,7,-h);  

  Htrans.zeros(7,7,"Htrans");
  Htrans.overwrite(Hsensor);
  Htrans.transpose();

  HsensorHtrans.zeros(7,7,"HsensorHtrans");

  Identity.zeros(7,7,"Identity");
  Identity.set(1,1,1);
  Identity.set(2,2,1);
  Identity.set(3,3,1);
  Identity.set(4,4,1);
  Identity.set(5,5,1);
  Identity.set(6,6,1);
  Identity.set(7,7,1);

  Pkmin.zeros(7,7,"Pkmin");
  xkmin.zeros(7,7,"xkmin");

  R.zeros(7,7,"R");
  ybar.zeros(7,7,"ybar");

  K.zeros(7,7,"k");
  kinv.zeros(7,7,"kinv");
  KinvHtrans.zeros(7,7,"KinvHtrans");
  kk.zeros(7,7,"kk");

  Hsensorkk.zeros(7,7,"Hsensorkk");
  xkminHsensorkk.zeros(7,7,"xkminHsensorkk");
  HsensorkkIdentity.zeros(7,7,"HsensorkkIdentity");

  xkplus.zeros(7,7,"xkplus");
  Pkplus.zeros(7,7,"Pkplus");

  Quat.zeros(4,1,"Quat");
  Quatm.zeros(4,1,"Quatm");
  
  PQR.zeros(3,1,"PQR");
  PQRmodel.zeros(3,1,"PQRmodel");

  negPQR.zeros(3,1,"negPQR");
  negPQRm.zeros(3,1,"negPQRm");

  PQRmatrix.zeros(4,4,"PQRmatrix");
  PQRmodelmatrix.zeros(4,4,"PQRmodelmatrix");

  Ftilde.zeros(7,7,"Ftilde");
  Ftrans.zeros(7,7,"Ftrans");

  P.zeros(7,7,"P");

  quatdot.zeros(4,1,"quatdot");
  quatmdot.zeros(4,1,"quatmdot");

  H.zeros(3,1,"H");
  Hm.zeros(3,1,"Hm");

  PQRcrossH.zeros(3,1,"PQRcrossH");
  PQRmcrossH.zeros(3,1,"PQRmcrossH");

  PQRdot.zeros(3,1,"PQRdot");
  PQRmdot.zeros(3,1,"PQRmdot");

  Pdot.zeros(7,7,"Pdot");
  Pdotvec.zeros(7,1,"Pdotvec");
  FtransP.zeros(7,7,"FtransP");
}

void SensorUpdate(MATLAB State,double TIME)
{ //Unpack Model State Vector 
  double q0kmin = State.get(8,1);
  double q1kmin = State.get(9,1);
  double q2kmin = State.get(10,1);
  double q3kmin = State.get(11,1);
  double pkmin = State.get(12,1);
  double qkmin = State.get(13,1);
  double rkmin = State.get(14,1);
  //Unpack Covariance Vector
  double Pq0kmin = State.get(15,1);
  double Pq1kmin = State.get(16,1);
  double Pq2kmin = State.get(17,1);
  double Pq3kmin = State.get(18,1);
  double Ppkmin = State.get(19,1);
  double Pqkmin = State.get(20,1);
  double Prkmin = State.get(21,1);

  //Setup Old Covariance Matrix
  Pkmin.set(1,1,Pq0kmin);
  Pkmin.set(2,2,Pq1kmin);
  Pkmin.set(3,3,Pq2kmin);
  Pkmin.set(4,4,Pq3kmin);
  Pkmin.set(5,5,Ppkmin);
  Pkmin.set(6,6,Pqkmin);
  Pkmin.set(7,7,Prkmin);

  //Setup Old Model State Matrix
  xkmin.set(1,1,q0kmin);
  xkmin.set(2,2,q1kmin);
  xkmin.set(3,3,q2kmin);
  xkmin.set(4,4,q3kmin);
  xkmin.set(5,5,pkmin);
  xkmin.set(6,6,qkmin);
  xkmin.set(7,7,rkmin);

  //Sensor Signal Disturbance 
  v = .1*sin(100*TIME);

  //Signal Disturbance Matrix
  R.set(1,1,v);
  R.set(2,2,v);
  R.set(3,3,v);
  R.set(4,4,v);
  R.set(5,5,v);
  R.set(6,6,v);
  R.set(7,7,v);

  ybar.mult(Hsensor,xkmin);
  ybar.plus_eq(R);

  //Kalman Gain Matrix
  HsensorHtrans.mult(Hsensor,Htrans);
  K.mult(HsensorHtrans,Pkmin);
  K.plus_eq(R);
  kinv.matrix_inverse(K,7);
  KinvHtrans.mult(Htrans,kinv);
  kk.mult(KinvHtrans,Pkmin);


  //State Update Equation
  Hsensorkk.mult(negHsensor,kk);
  xkminHsensorkk.mult(Hsensorkk,xkmin);
  xkplus.mult(kk,R);
  xkplus.plus_eq(xkminHsensorkk);
  xkplus.plus_eq(xkmin);

  //New Covariance
  HsensorkkIdentity.plus(Hsensorkk,Identity);
  Pkplus.mult(Pkmin,HsensorkkIdentity);

  //Unpack Covariance Matrix to Push into State Vector
  double Pq0kplus = Pkplus.get(1,1);
  double Pq1kplus = Pkplus.get(2,2);
  double Pq2kplus = Pkplus.get(3,3);
  double Pq3kplus = Pkplus.get(4,4);
  double Ppkplus = Pkplus.get(5,5);
  double Pqkplus = Pkplus.get(6,6);
  double Prkplus = Pkplus.get(7,7);

  //Unpack Model State Matrix to Push into State Vector
  double q0kplus = xkplus.get(1,1);
  double q1kplus = xkplus.get(2,2);
  double q2kplus = xkplus.get(3,3);
  double q3kplus = xkplus.get(4,4);
  double pkplus = xkplus.get(5,5);
  double qkplus = xkplus.get(6,6);
  double rkplus = xkplus.get(7,7);


  //Update Model State
  State.set(8,1,q0kplus);
  State.set(9,1,q1kplus);
  State.set(10,1,q2kplus);
  State.set(11,1,q3kplus);
  State.set(12,1,pkplus);
  State.set(13,1,qkplus);
  State.set(14,1,rkplus);
  State.set(15,1,Pq0kplus);
  State.set(16,1,Pq1kplus);
  State.set(17,1,Pq2kplus);
  State.set(18,1,Pq3kplus);
  State.set(19,1,Ppkplus);
  State.set(20,1,Pqkplus);
  State.set(21,1,Prkplus);
}
void Derivatives(MATLAB State,MATLAB StateDot,double TIME)
{ 
  //Quaternion Vector Setup
  Quat.vecset(1,4,State,1);
  Quatm.vecset(1,4,State,8);

  //PQR Vector Setup
  PQR.vecset(1,3,State,5);
  PQRmodel.vecset(1,3,State,12);

  negPQR.mult(PQR,-1);
  negPQRm.mult(PQRmodel,-1);

  //PQR Matrix Setup
  double p = State.get(5,1);
  double q = State.get(6,1);
  double r = State.get(7,1);

  double pm = State.get(12,1);
  double qm = State.get(13,1);
  double rm = State.get(14,1);

  PQRmatrix.set(1,2,-p);
  PQRmatrix.set(1,3,-q);
  PQRmatrix.set(1,4,-r);
  PQRmatrix.set(2,1,p);
  PQRmatrix.set(2,3,r);
  PQRmatrix.set(2,4,-q);
  PQRmatrix.set(3,1,q);
  PQRmatrix.set(3,2,-r);
  PQRmatrix.set(3,4,p);
  PQRmatrix.set(4,1,r);
  PQRmatrix.set(4,2,q);

  PQRmodelmatrix.set(4,3,-pm);
  PQRmodelmatrix.set(1,2,-pm);
  PQRmodelmatrix.set(1,3,-qm);
  PQRmodelmatrix.set(1,4,-rm);
  PQRmodelmatrix.set(2,1,pm);
  PQRmodelmatrix.set(2,3,rm);
  PQRmodelmatrix.set(2,4,-qm);
  PQRmodelmatrix.set(3,1,qm);
  PQRmodelmatrix.set(3,2,-rm);
  PQRmodelmatrix.set(3,4,pm);
  PQRmodelmatrix.set(4,1,rm);
  PQRmodelmatrix.set(4,2,qm);
  PQRmodelmatrix.set(4,3,-pm);

  //Ftilde Setup
  double ixxm = Imodelinv.get(1,1);
  double iyym = Imodelinv.get(2,2);
  double izzm = Imodelinv.get(3,3);

  Ftilde.set(4,3,-pm);
  Ftilde.set(1,2,-pm);
  Ftilde.set(1,3,-qm);
  Ftilde.set(1,4,-rm);
  Ftilde.set(2,1,pm);
  Ftilde.set(2,3,rm);
  Ftilde.set(2,4,-qm);
  Ftilde.set(3,1,qm);
  Ftilde.set(3,2,-rm);
  Ftilde.set(3,4,pm);
  Ftilde.set(4,1,rm);
  Ftilde.set(4,2,qm);
  Ftilde.set(4,3,-pm);
  Ftilde.set(5,5,ixxm);
  Ftilde.set(6,6,iyym);
  Ftilde.set(7,7,izzm);

  Ftrans.overwrite(Ftilde);
  Ftrans.transpose();

  //Pvector Setup
  P.vecset(1,7,State,15);

  //Quaternion Rate of Change
  Quat.disp(); //Display Quaternion Matrix
  PQRmatrix.disp(); //Display PQR Matrix
  quatdot.mult(PQRmatrix,Quat);
  quatdot.mult_eq(.5);

  quatmdot.mult(PQRmodelmatrix,Quatm);
  quatmdot.mult_eq(.5);

  //Angular Momentum
  H.mult(I,PQR);
  Hm.mult(Imodel,PQRmodel);

  //Rate of Change of Angular Velocity
  PQRcrossH.cross(negPQR,H);
  PQRmcrossH.cross(negPQRm,Hm);

  PQRdot.mult(Iinv,PQRcrossH);
  PQRmdot.mult(Imodelinv,PQRmcrossH);

  //Covariance
  Pdot.mult(Ftilde,P);
  FtransP.mult(P,Ftrans);
  Pdot.plus_eq(FtransP);
  Pdot.plus_eq(MQMtrans);

  double Pq0dot = Pdot.get(1,1);
  double Pq1dot = Pdot.get(2,2);
  double Pq2dot = Pdot.get(3,3);
  double Pq3dot = Pdot.get(4,4);
  double Ppdot = Pdot.get(5,5);
  double Pqdot = Pdot.get(6,6);
  double Prdot = Pdot.get(7,7);

  Pdotvec.set(1,1,Pq0dot);
  Pdotvec.set(2,1,Pq1dot);
  Pdotvec.set(3,1,Pq2dot);
  Pdotvec.set(4,1,Pq3dot);
  Pdotvec.set(5,1,Ppdot);
  Pdotvec.set(6,1,Pqdot);
  Pdotvec.set(7,1,Prdot);

  //Statedot Vector Setup
  StateDot.vecset(1,4,quatdot,1); 
  StateDot.vecset(5,7,PQRdot,1);
  StateDot.vecset(8,11,quatmdot,1); 
  StateDot.vecset(12,14,PQRmdot,1);
  StateDot.vecset(15,21,Pdotvec,1);
}

int main(int argc,char* argv[])
{
  //If you want to grab some input arguments you can use
  //atof()
 
  //Run the setup routine and send it the initial conditions
 
    setup(atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),atof(argv[7]),atof(argv[8]),atof(argv[9]),atof(argv[10]),atof(argv[11]),atof(argv[12]),atof(argv[13]));
 

  //Create OUTPUT FILE so you can read it later in MATLAB
  FILE* outputfile;
  if (WRITEON)
    {
      outputfile = fopen("FinalProject.out","wb");
    }

  //Kick off a clock for kicks
  clock_t t;
  t = clock();

  for (double TIME = TINITIAL;TIME<=TFINAL;TIME+=TIMESTEP)
    {
      //Print State to File
      if (WRITEON)
{
 fprintf(outputfile,"%lf ",TIME);
 for (int i = 1;i<=NUMSTATES;i++) {
   fprintf(outputfile,"%lf ",State.get(i,1)); //Again use the get() routine
 }
 fprintf(outputfile,"\n");
}
      //Notify User of Progress
      printf("Simulation %lf Percent Complete \n",TIME/TFINAL*100);
     
      //Compute Control once per timestep
      //Control(State,TIME);

      //First call
      Derivatives(State,k1,TIME);
      StateDel.overwrite(State); //StateDel = State
      StateDel.plus_mult_eq(k1,TIMESTEP/2); //StateDel = StateDel + k1*TIMESTEP/2

      //Second Call
      Derivatives(StateDel,k2,TIME);
      StateDel.overwrite(State);
      StateDel.plus_mult_eq(k2,TIMESTEP/2);

      //Third Call
      Derivatives(StateDel,k3,TIME);
      StateDel.overwrite(State);
      StateDel.plus_mult_eq(k3,TIMESTEP);

      //Fourth Call
      Derivatives(StateDel,k4,TIME);

      //Put it all together
      // phi = k1/6 + k2/3 + k3/3 + k4/6
      phi.mult(k1,1.0/6.0); // phi = k1/6
      phi.plus_mult_eq(k2,1.0/3.0); // phi = phi + k2/3
      phi.plus_mult_eq(k3,1.0/3.0); // phi = phi + k3/3
      phi.plus_mult_eq(k4,1.0/6.0); // phi = phi + k2/3

      //And then State = State + phi*timestep
      State.plus_mult_eq(phi,TIMESTEP);
      //Update State and Covariance if it is time for measurement
      if ((TIME - SensorTime) >= SensorPeriod)
      { 
        SensorTime = TIME;
        SensorUpdate(State,TIME);
        printf("SensorTime = %lf \n", SensorTime);
      }
    }

  //Close the output file
  if (WRITEON)
    {
      fclose(outputfile);
    }
 
  //Print End Time
  t = clock() - t;
  printf("C++ Elapsed Time = %f \n",(float)t/CLOCKS_PER_SEC);
}