#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
using namespace std;

//CONSTANTS AND ASSIGNMENTS
#define NUMSTATES 2
#define CREATEARRAY(IN) (double*)malloc(IN*sizeof(double));

//GLOBALS
int WRITEON;
double ucontrol,kp,kd,xcommand,xdotcommand,m,c,k;
double TFINAL,TINITIAL,TIMEON,TIMESTEP;
time_t rawtime;
struct tm * ptm;
double *State,*k1,*k2,*k3,*k4,*phi,*StateDel;

void PAUSE()
{
  printf("%s","Function Paused\n");
  printf("Type in any number(1-inf) and press enter to continue\n");
  int  a;
  scanf("%i",&a);
}

void Initialize_Inputs()
{
  WRITEON = 1;
  TINITIAL = 0;
  TFINAL = 100;
  TIMESTEP = 0.001;
  State = CREATEARRAY(NUMSTATES);
  StateDel = CREATEARRAY(NUMSTATES);
  k1 = CREATEARRAY(NUMSTATES);
  k2 = CREATEARRAY(NUMSTATES);
  k3 = CREATEARRAY(NUMSTATES);
  k4 = CREATEARRAY(NUMSTATES);
  phi = CREATEARRAY(NUMSTATES);
  State[0] = -1;
  State[1] = 0;
  m = 1;
  c = 0;
  k = 3;
  TIMEON = 500000;
  xcommand = 1;
  xdotcommand = 0;
  kp = 30;
  kd = 10;
}

////////ROUTINES FOR MATRICES//////////////
void disp(double *in,char* name) {
  printf("%s = ",name);
  for (int i = 0;i<NUMSTATES;i++) {
    printf("%lf ",in[i]);
  }
  printf("\n");
}

void mult(double *out,double *in,double val) {
  //out = in*val
  for (int i=0;i<NUMSTATES;i++){
    out[i] = in[i]*val;
  }
}

void multeq(double *inout,double val) {
  //inout = inout*val
  for (int i=0;i<NUMSTATES;i++){
    inout[i] *= val;
  }
}

void plus(double* out,double *in,double *add,double val) {
  //out = in + add*val
  for (int i = 0;i<NUMSTATES;i++) {
    out[i] = in[i] + add[i]*val;
  }
}

void pluseq(double* inout,double *add,double val) {
  //inout = inout + add*val
  for (int i = 0;i<NUMSTATES;i++) {
    inout[i] += add[i]*val;
  }
}

////////////CONTROL AND DERIVATIVES ROUTINE////////////////
void Control(double *State,double TIME)
{
   ucontrol = 0;
   if (TIME > TIMEON)
   {
    ucontrol = -kp*(State[0]-xcommand) - kd*(State[1]-xdotcommand);
   }
}

void Derivatives(double* State,double* StateDot)
{
  //extract states
  double x = State[0];
  double xdot = State[1];
  StateDot[0] = xdot;
  StateDot[1] = -k/m*x - c/m*xdot + ucontrol/m;
}

int main()
{
  //printf("Spring Mass Damper Program \n");
  //int x;
  //x = 2;
  //printf("x = %d \n",x);
  //Initialize Read_Inputs
  Initialize_Inputs();
  //Initialize State Derivatives and Control
  Control(State,0);
  //Create OUTPUT FILE
  FILE* outputfile;
  if (WRITEON)
    {
      outputfile = fopen("SimulationResultsC++.out","wb");
    }
  //Loop until TIME > TFINAL

  clock_t t;
  t = clock();

  for (double TIME = TINITIAL;TIME<=TFINAL;TIME+=TIMESTEP)
    {
      //Compute Control
      Control(State,TIME);
      //Print State to File
      if (WRITEON)
	{
	  fprintf(outputfile,"%lf ",TIME);
	  for (int i = 0;i<NUMSTATES;i++) {
	    fprintf(outputfile,"%lf ",State[i]);
	  }
	  fprintf(outputfile,"\n");
	}
      printf("Simulation %lf Percent Complete \n",TIME/TFINAL*100);
      //RK4 Calls
      Derivatives(State,k1);
      plus(StateDel,State,k1,TIMESTEP/2);
      Derivatives(StateDel,k2);
      plus(StateDel,State,k2,TIMESTEP/2);
      Derivatives(StateDel,k3);
      plus(StateDel,State,k3,TIMESTEP);
      Derivatives(StateDel,k4);
      plus(phi,k1,k2,2.0);
      pluseq(phi,k3,2.0);
      pluseq(phi,k4,1.0);
      multeq(phi,1.0/6.0);
      pluseq(State,phi,TIMESTEP);
    }
  if (WRITEON)
    {
      fclose(outputfile);
    }

  //Print End Time
  t = clock() - t;
  printf("C++ Elapsed Time = %f \n",(float)t/CLOCKS_PER_SEC);
}
