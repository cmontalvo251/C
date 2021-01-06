//////Facility for Aerial Systems and Technology (FAST) Real Time Simulator/////////
/////Initial edited by Carlos Montalvo

//See the following Issue on Github - https://github.com/cmontalvo251/C/issues/3

///Revisions created - 12/10/2020 - Added Loop timer
//1/1/2021 - Added Datalogger, RK4 routine and point mass model in space using Dynamics class
//1/2/2021 - Added point mass model on flat earth, added environment class. Fixed some compilation flags.

//Revisions Needed 
//force and moment model (sixdof model as well)
//openGL if requested 
//Joystick if manual mode
//Sensor block
//Once everything is imported it's time to kick off the main loop which depends on the algorithm.
//My vote is we do it in this order
//Send state vector to openGL if rendering and desktop
//Send state vector via serial if HIL
//Send state vector to sensor routine if desktop
//Render OpenGL if turned on
//Call the sensor block which polls fictitious sensors on desktop
//Read Joystick or Hardcoded guidance commands (skip if HIL and desktop)
//Pass Commands and Measurements to autopilot specific to drone being simulated (skip if HIL and desktop)
//If desktop pass control signals to RK4 routine
//Integrate RK4 1 step If desktop

#include "main.h"

//define time parameters 
double t = 0;
double PRINT = 0;
double LOG = 0;
double startTime,current_time;
double tfinal,INTEGRATIONRATE,PRINTRATE;
double LOGRATE;

///REALTIME VARS
#ifdef REALTIME
TIMER timer;
#endif

//Simulator
#ifdef RK4_H
RK4 integrator;
#endif

//Vehicle Specifics (Every vehicle will have its own dynamic model but everything
//will be in a Dynamics class)
Dynamics vehicle;

///DATALOGGER IS ALWAYS RUNNING
Datalogger logger;
MATLAB logvars;

//////Main/////////////
int main(int argc,char** argv) {

  //Initialize timer if code running realtime
  #ifdef REALTIME
  startTime = timer.getTimeSinceStart();
  current_time = timer.getTimeSinceStart() - startTime;
  #endif

  //Initialize Vehicle
  vehicle.init();
  #ifdef DEBUG
  printf("Vehicle Initialized\n");
  #endif

  //Initialize Datalogger
  logger.findfile("logs/");
  logger.open();
  logvars.zeros(vehicle.NUMSTATES+1,1,"Vars to Log"); //All states + time
  //Import Simulation Flags
  MATLAB simdata;
  int ok = logger.ImportFile("Input_Files/Simulation_Flags.txt",&simdata,"simdata",vehicle.NUMSTATES);
  if (!ok) { exit(1); } else {simdata.disp();}
  tfinal = simdata.get(1,1);
  INTEGRATIONRATE = simdata.get(2,1);
  PRINTRATE = simdata.get(3,1);
  #ifdef REALTIME
  PRINTRATE = 1.0;
  #endif
  LOGRATE = simdata.get(4,1);
  int GRAVITY = simdata.get(5,1);

  /////////////////Initialize RK4 if simulating Dynamics///////////////////
  #if defined (SIMONLY) || (SIL) || (HIL)
  //First Initialize integrator and Dynamic Model
  integrator.init(vehicle.NUMSTATES,INTEGRATIONRATE);
  //Import Initial Conditions, mass properties
  MATLAB icdata,massdata;
  ok = logger.ImportFile("Input_Files/Initial_Conditions.txt",&icdata,"icdata",vehicle.NUMSTATES);
  if (!ok) { exit(1); } else {icdata.disp();}
  ok = logger.ImportFile("Input_Files/MassProperties.txt",&massdata,"massdata",4);
  if (!ok) { exit(1); } else {massdata.disp();}
  //Set ICs in integrator 
  integrator.set_ICs(icdata);
  //Send Mass Data to Dynamic Model
  vehicle.setMassProps(massdata);
  //Initialize Environment
  vehicle.setEnvironment(GRAVITY);
  #ifdef DEBUG
  printf("Integrator Initialized\n");
  #endif
  #endif

  ///Initialize the Rendering Environment Must be done in a boost thread
  #ifdef OPENGL_H
  renderInit(argc,argv);
  #endif

  ////Begin Simulation
  runSimulation();
  
} //end main loop desktop computer

#ifdef OPENGL_H
void renderInit(int argc,char** argv) {
  int Farplane = 10000;
  int width = 600;
  int height = 600;
  int defaultcamera = 0;
  glhandle_g.Initialize(argc,argv,Farplane,width,height,defaultcamera);
}
#endif

void runSimulation() {
  printf("Running Simulation \n");
  //Kick off integration loop
  while (t < tfinal) {

    /////////////UPDATE CURRENT TIME//////////////////////////////////
    #if defined (SIL) || (HIL)
    //Wait loop to make sure we run in realtime
    //Don't do in AUTO or SIMONLY because we want to run as fast as possible
    //Get current time 
    while (current_time < t) {
      current_time = timer.getTimeSinceStart()-startTime;   
    }
    #endif

    #ifdef REALTIME
    //Keep simulating until user hits CTRL+C when running in AUTO or HIL mode
    tfinal = t+100; 
    //Get Time right now
    current_time = timer.getTimeSinceStart()-startTime;
    #endif
    ///////////////////////////////////////////////////////////////////

    /////////RK4 INTEGRATION LOOP///////////////
    #if defined (SIL) || (SIMONLY) || (HIL)
    for (int i = 1;i<=4;i++){
      vehicle.Derivatives(integrator.StateDel,integrator.k);
      //integrator.StateDel.disp();
      //integrator.k.disp();
      //PAUSE();
      integrator.integrate(i);
    }
    //Integrate time
    t += INTEGRATIONRATE;
    #endif
    ////////////////////////////////////////////

    /////////////////Print to STDOUT////////////////
    if (PRINT<t) {
      printf("%lf ",t);
      for (int i = 0;i<integrator.NUMSTATES;i++) {
        printf("%lf ",integrator.State.get(i+1,1));
      }
      printf("\n");
      PRINT+=PRINTRATE;
    }

    ////////////////LOG DATA////////////////////////
    if (LOG<t) {
      logvars.set(1,1,t);
      for (int i = 0;i<integrator.NUMSTATES;i++) {
        logvars.set(2+i,1,integrator.State.get(i+1,1));
      }
      logger.println(logvars);
    }
    ////////////////////////////////////////////////

  } //End while loop on main loop
  printf("Simulation Complete\n");
}