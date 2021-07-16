//Ok let's heavily comment this shit. First we need a bunch of built in libraries
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
using namespace std;

//If we were just doing basic multiplication and substraction we could just use
//those but if we want to do fancy matrix multiplication and inverse routines we
//need something better. I've done this already. It's called MATLAB++ (MATLAB for C++)
//the quotes are because it's in this folder. Make sure to include the cpp file
//as well when you code your stuff
#include "../MATLAB.h"
//^^All of the routines for matrix multiplication are in there.
//The problem is you need to compile MATLAB.cpp into a module using the -c objective

//So first
//g++ ../MATLAB.cpp -c -w
//this will create a .o or module file

//Then you need
//g++ MATLAB.o rk4_using_MATLAB++.cpp

//This will create an a.out file

//Alright now when we make our global variables and we want them to be a matrix
//we use the MATLAB class. So rather than everything being a double we use MATLAB
//like this
MATLAB State,k1,k2,k3,k4,phi,StateDel,A,B;
//Everything that is an integer can be initialized the same way as before
int WRITEON;
double ucontrol,kp,kd,xcommand,xdotcommand,m,k,c;
double TFINAL,TINITIAL,TIMEON,TIMESTEP;
time_t rawtime;
struct tm * ptm;

//This will tell use how many states we have
#define NUMSTATES 2

///We can still use this function for debugging so I'll leave it here
void PAUSE()
{
  printf("%s","Function Paused\n");
  printf("Type in any number(1-inf) and press enter to continue\n");
  int  a;
  scanf("%i",&a);
}

///Let's create a setup routine like they do on arduino.
void setup(double x0,double xdot0)
{
  //All scalars can be initialized the same way
  WRITEON = 1;
  TINITIAL = 0;
  TFINAL = 100;
  TIMESTEP = 0.001;
  TIMEON = 1;
  xcommand = 1;
  xdotcommand = 0;
  kp = 30;
  kd = 10;
  m = 3;
  k = 10;
  c = 2;
  ucontrol = 0;

  //Matrices on the other hand have some special functions to
  //get initialized. The zeros() function takes the number of rows, columns and
  //the name of the vector
  State.zeros(NUMSTATES,1,"State");
  StateDel.zeros(NUMSTATES,1,"StateDel");
  k1.zeros(NUMSTATES,1,"k1");
  k2.zeros(NUMSTATES,1,"k2");
  k3.zeros(NUMSTATES,1,"k3");
  k4.zeros(NUMSTATES,1,"k4");
  phi.zeros(NUMSTATES,1,"phi");
  A.zeros(NUMSTATES,NUMSTATES,"A");
  B.zeros(NUMSTATES,1,"B");

  //In order to set the initial conditions you need to use the set() function
  State.set(1,1,x0); //this says put x0 in the first row first column
  State.set(2,1,xdot0);

  //THe equations of motion will be
  //statedot = A*state + B*u so
  A.set(1,1,0);
  A.set(1,2,1);
  A.set(2,1,-k/m);
  A.set(2,2,-c/m);
  B.set(1,1,0);
  B.set(2,1,1/m);
  
}

////////////CONTROL AND DERIVATIVES ROUTINE////////////////
void Control(MATLAB State,double TIME)
{
  //In order to extract variables from a MATLAB matrix you need to use the get() command
  double x = State.get(1,1);
  double xdot = State.get(2,1);
   ucontrol = 0;
   if (TIME > TIMEON)
   {
     ucontrol = -kp*(x-xcommand) - kd*(xdot-xdotcommand);
   }
}

void Derivatives(MATLAB State,MATLAB StateDot)
{
  //For this derivatives routine we're going to do some fancy matrix
  //math. First let's assume the equations of motion are
  //statedot = A*state + B*u
  //First compute A*state
  StateDot.mult(A,State);
  //Then add B*u
  StateDot.plus_mult_eq(B,ucontrol);
  //And that's it. Again, MATLAB.h has a list of all the different routines.
  //like transpose, and inverse
}

int main(int argc,char* argv[])
{
  //If you want to grab some input arguments you can use
  //atof()
  
  //Run the setup routine and send it the initial conditions
  if (argc == 3) {
    setup(atof(argv[1]),atof(argv[2]));
  } else {
    printf("Improper number of input arguments \n");
    exit(1);
  }
    

  //Create OUTPUT FILE so you can read it later in MATLAB
  FILE* outputfile;
  if (WRITEON)
    {
      outputfile = fopen("SimulationResultsC++.out","wb");
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
      Control(State,TIME);
      
      //RK4 Calls --- /Again now we need to use matrix math to add things together

      //First call
      Derivatives(State,k1);
      StateDel.overwrite(State); //StateDel = State
      StateDel.plus_mult_eq(k1,TIMESTEP/2); //StateDel = StateDel + k1*TIMESTEP/2

      //Second Call
      Derivatives(StateDel,k2);
      StateDel.overwrite(State); 
      StateDel.plus_mult_eq(k2,TIMESTEP/2);

      //Third Call
      Derivatives(StateDel,k3);
      StateDel.overwrite(State);
      StateDel.plus_mult_eq(k3,TIMESTEP);

      //Fourth Call
      Derivatives(StateDel,k4);

      //Put it all together
      // phi = k1/6 + k2/3 + k3/3 + k4/6
      phi.mult(k1,1.0/6.0); // phi = k1/6
      phi.plus_mult_eq(k2,1.0/3.0); // phi = phi + k2/3
      phi.plus_mult_eq(k3,1.0/3.0); // phi = phi + k3/3
      phi.plus_mult_eq(k4,1.0/6.0); // phi = phi + k2/3

      //And then State = State + phi*timestep
      State.plus_mult_eq(phi,TIMESTEP);
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
