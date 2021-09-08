//////Facility for Aerial Systems and Technology (FAST) Real Time Simulator/////////
/////Initial edited by Carlos Montalvo

//See the following Issue on Github - https://github.com/cmontalvo251/C/issues/3
#include "main.h"

//define Global time parameters 
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
//Vehicle is initialized on creation in the constructor
//so is the rcin class which is in side the Dynamics class
Dynamics vehicle;

///DATALOGGER IS ALWAYS RUNNING
Datalogger logger;
MATLAB logvars;

///Global Variable for Fileroot? Can I do this safely?
char fileroot[256];

void get_fileroot(int argc,char** argv,char fileroot[]) {
  printf("Number of Input Arguments = %d \n",argc);
  if (argc == 1) {
    //This means there are no input arguments and we must default to PortalCube
    sprintf(fileroot,"%s","PortalCube/");
    printf("%s \n","No Input Arguments given....defaulting to:");
  } else {
    //this means some input arguments were provided
    sprintf(fileroot,"%s",argv[1]);
    printf("%s \n","Input argument given:");
  }
  printf("%s \n",fileroot);
}

//////Main/////////////
int main(int argc,char** argv) {

  //////////////Grab Input Arguments//////////////////////////////////////////
  get_fileroot(argc,argv,fileroot);
  //////////////////////////////////////////////////////////

  //////////////////////////Initialize Datalogger//////////////////////////////
  logger.findfile("logs/");
  logger.open();
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////Import Simulation Flags//////////////////////////////////
  MATLAB simdata;
  char simfile[256]={NULL};
  strcat(simfile,fileroot);
  strcat(simfile,"Input_Files/Simulation_Flags.txt");
  int ok = logger.ImportFile(simfile,&simdata,"simdata",-99);
  if (!ok) { exit(1); } else {simdata.disp();}
  tfinal = simdata.get(1,1);
  ////////RATES////////////////////////////////////////////
  INTEGRATIONRATE = simdata.get(2,1);
  PRINTRATE = simdata.get(3,1); 
  LOGRATE = simdata.get(4,1); 
  double RCRATE = simdata.get(5,1);
  double CTLRATE = simdata.get(6,1);
  vehicle.setRates(RCRATE,CTLRATE);
  //These are extras that we only need if we are integrating the but it doesn't
  //Require any computation time except on startup to read them so just keep
  //them here for all different scenarios
  int GRAVITY_FLAG = simdata.get(7,1);
  int AERO_FLAG = simdata.get(8,1);
  int CTL_FLAG = simdata.get(9,1);
  int ERROR_FLAG = simdata.get(10,1);
  int ACTUATOR_FLAG = simdata.get(11,1);
  //////////////////////////////////////////////////////////////////////////////

  /////////////////////////////MASS DATA/////////////////////////////
  ////////////////////////THIS IS GOING TO IMPORT EVERYTIME//////////
  /////////////////////IN THE EVENT YOU NEED MASS AND INERTIA///////
  ///////////////////FOR YOUR CONTROL SYSTEM////////////////////////
  MATLAB massdata;
  char massfile[256]={NULL};
  strcat(massfile,fileroot);
  strcat(massfile,"Input_Files/MassProperties.txt");
  ok = logger.ImportFile(massfile,&massdata,"massdata",4);
  if (!ok) { exit(1); } else {massdata.disp();}
  //Send Mass Data to Dynamic Model
  vehicle.setMassProps(massdata);
  /////////////////////////////////////////////////////////////

  ////////////////////Initialize Controller///////////////////
  /////This happens regardless of the type of simulation
  vehicle.initController(CTL_FLAG);
  printf("Controller Online \n");
  ///The RC input class is initialized all the time and its settings are in
  //the Makefile and RCInput.cpp

  //////////////////?WHEN INTEGRATING ON THE COMPUTER///////////////////
  /////////////////////A FEW MORE THINGS NEED TO HAPPEN///////////////
  #ifdef RK4_H
  /////////////////////////Import Actuator Parameters////////////////////////////
  if (ACTUATOR_FLAG == 1) {
    MATLAB actuatordata;
    char actuatorfile[256]={NULL};
    strcat(actuatorfile,fileroot);
    strcat(actuatorfile,"Input_Files/Actuators.txt");
    ok = logger.ImportFile(actuatorfile,&actuatordata,"actuatordata",-99);
    if (!ok) {exit(1);} else {actuatordata.disp();}
    vehicle.initActuators(actuatordata);
    printf("Actuators Online \n");
  } else {
    //If actuators are off we still need a pass through
    vehicle.initActuators(vehicle.ctl.NUMSIGNALS);
    printf("NO ACTUATORS!!! \n");
  }
  //First Initialize integrator and Dynamic Model
  integrator.init(vehicle.NUMVARS,INTEGRATIONRATE);
  printf("Integrator Initialized \n");

  //Import Initial Conditions
  MATLAB icdata;
  char initfile[256]={NULL};
  strcat(initfile,fileroot);
  strcat(initfile,"Input_Files/Initial_Conditions.txt");
  ok = logger.ImportFile(initfile,&icdata,"icdata",vehicle.NUMSTATES);
  if (!ok) { exit(1); } else {icdata.disp();}
  //Set ICs in integrator 
  if (vehicle.NUMACTUATORS > 0) {
    //Need to send the integrator the combined state from the states and the 
    //actuators
    MATLAB icdataALL;
    icdataALL.zeros(vehicle.NUMACTUATORS,1,"icdataALL");
    icdataALL.vecset(1,vehicle.NUMSTATES,icdata,1);
    icdataALL.vecset(vehicle.NUMSTATES,vehicle.NUMVARS,vehicle.actuatorICs,1);
    icdataALL.disp();
    integrator.set_ICs(icdataALL);
  } else {
    integrator.set_ICs(icdata);
  }
  printf("Initial Conditions Sent to Integrator \n");
  //Set State of Vehicle since the integrator doesn't assume a 6DOF model or actuators
  vehicle.setState(integrator.State,integrator.k);
  printf("Vehicle State Set \n");
  ////Init Aerodynamics
  vehicle.initAerodynamics(AERO_FLAG);
  printf("Aerodynamics is Windy \n");
  //Init Extra Models
  vehicle.initExtModels(GRAVITY_FLAG);
  printf("Earth Environment is Flat as of Version 0.9 \n");
  //Initialize the Error Model 
  if (ERROR_FLAG) {
    MATLAB sensordata;
    char sensorfile[256]={NULL};
    strcat(sensorfile,fileroot);
    strcat(sensorfile,"Input_Files/Sensor_Errors.txt");
    ok = logger.ImportFile(sensorfile,&sensordata,"sensordata",-99); //-99 for automatic length array
    if (!ok) { exit(1); } else {sensordata.disp();}    
    vehicle.initErrModel(sensordata);
    printf("User has added Sensor Errors \n");
  }
  #endif
  ////////////////////////////////////////////////////////////////////////////

  ///////////COMPUTE NUMBER OF VARIABLES TO LOG/////////////
  //Number of states to log
  vehicle.NUMLOGS = 1; //Always log time
  //If RK4 is on we log....
  #ifdef RK4_H
  //All the states including the actuator states
  vehicle.NUMLOGS+=(vehicle.NUMVARS);
  vehicle.NUMLOGS+=6; //Forces and moments
  //It will also log the actuatorError values
  //But only if this value is non-zero. Otherwise 
  //this value is just a pass through a worthless
  vehicle.NUMLOGS+=vehicle.NUMACTUATORS; 
  #endif 
  //No matter what we log the sensor measurements
  //The error model only assumes a 12 state system
  //The control signals below will give you the commands sent
  vehicle.NUMLOGS+=12;
  //We also log all of the receiver signals
  vehicle.NUMLOGS+=vehicle.rcin.num_of_axis;
  //We also log the control signals
  vehicle.NUMLOGS+=vehicle.ctl.NUMSIGNALS;
  logvars.zeros(vehicle.NUMLOGS,1,"Vars to Log");
  ///////////////////////////////////////////////////////

  //////////////////Start Rendering Environment Must be done in a boost thread/////////////////
  #ifdef OPENGL_H
  boost::thread render(runRenderLoop,argc,argv);
  //Wait for the opengl routine to actually start
  while (glhandle_g.ready == 0) {
    cross_sleep(1);
  }
  #endif
  /////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////Begin MainLoop///////////////////////////////////////////////
  #ifdef OPENGL_H 
  //When you have opengl running you need to kick this off as a thread
  boost::thread run(runMainLoop); 
  //Now there is a problem here. When you kick off the rendering environment and the Mainloop
  //above the code actually has 3 threads now. The MainLoop, the rendering environment and this main.cpp
  //In otherwords you need to create an infinite loop here
  while(1){cross_sleep(1);}; 
  #else
  //If you aren't rendering you just need to kick off the mainloop without a thread
  runMainLoop();
  #endif
  /////////////////////////////////////////////////////////////////////////////////////////////
  
} //end main loop desktop computer

///////////////////////////RENDERERING ENVIRONMENT///////////////////////////////////
///////////////////////////MUST BE CALLED IN A BOOST THREAD//////////////////////////
#ifdef OPENGL_H
void runRenderLoop(int argc,char** argv) {
  int Farplane = 10000;
  int width = 600;
  int height = 600;
  int defaultcamera = 3; //This is where you set the default camera
  //#camera 0-objects-1 = follow cameras
  //#objects-objects*2 - origin cameras
  //so a 0 will follow the first object
  //if there are 3 objects (cube, sky, ground) then defcam = 3 would
  //be an origin camera following the cube
  glhandle_g.loop(argc,argv,fileroot,Farplane,width,height,defaultcamera);
}
#endif
////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////MAIN LOOP/////////////////////////////////////////////
void runMainLoop() {
  #ifdef DEBUG
  printf("Running MainLoop \n");
  #endif

  //////////////////Initialize timer if code running realtime////////////////////
  #ifdef REALTIME
  startTime = timer.getTimeSinceStart();
  current_time = timer.getTimeSinceStart() - startTime;
  PRINTRATE = 1.0;
  #endif
  ///////////////////////////////////////////////////////////////////////////////

  //Kick off main while loop
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

    //////////////////GET CURRENT TIME////////////////////////////////////
    #ifdef REALTIME
    //Keep simulating until user hits CTRL+C when running in AUTO or HIL mode
    tfinal = t+100; 
    //Get Time right now
    current_time = timer.getTimeSinceStart()-startTime;
    #endif
    ///////////////////////////////////////////////////////////////////

    //////////////////////Run the Main Vehicle Loop////////////////////
    //This polls the RC inputs and the controller. This will always run
    //when we're on the RPI but only in SIMONLY and SIL on the desktop
    #if defined (SIMONLY) || (SIL) || (RPI)
    vehicle.loop(t);
    #endif
    ///////////////////////////////////////////////////////////////////
    
    /////////RK4 INTEGRATION LOOP///////////////
    #ifdef RK4_H
    for (int i = 1;i<=4;i++){
      vehicle.Derivatives(t,integrator.StateDel,integrator.k);
      //integrator.StateDel.disp();
      //integrator.k.disp();
      //PAUSE();
      integrator.integrate(i);
    }
    //Integrate time
    t += INTEGRATIONRATE;
    //Set State of Vehicle rather than the integrator
    //The integrator is dumb and does not know what the states are
    //It just takes z(i+1) = z(i) + k*dt
    //where k = zdot(i)
    //This routine below takes the integrator states and puts them into
    //more standard 6DOF nomenclature
    vehicle.setState(integrator.State,integrator.k); 
    #endif
    ////////////////////////////////////////////

    //////////////IF RENDERING//////////////////
    ////Send state vector to OpenGL
    #ifdef OPENGL_H
    glhandle_g.state.UpdateRender(t,vehicle.cg,vehicle.ptp,1);
    #endif

    /////////////////Print to STDOUT////////////////
    if (PRINT<t) {
      printf("%lf ",t);
      #ifdef RK4_H
      for (int i = 0;i<integrator.NUMVARS;i++) {
        printf("%lf ",integrator.State.get(i+1,1));
      }
      #endif
      //vehicle.printRC(0); //the zero means just the sticks
      printf("\n");
      PRINT+=PRINTRATE;
      //PAUSE();
    }
    /////////////////////////////////////////////////

    ////////////////LOG DATA////////////////////////
    if (LOG<t) {
      logvars.set(1,1,t);
      int ctr = 2;
      //Error States (Sensor Measurements) - Always on and always 12 states
      for (int i = 0;i<12;i++) {
        logvars.set(ctr,1,vehicle.err.errstate.get(i+1,1));
        ctr++;
      }       
      //Receiver Commands - always on
      for (int i = 0;i<vehicle.rcin.num_of_axis;i++) {
        logvars.set(ctr,1,vehicle.rcin.rxcomm[i]);
        ctr++;
      }
      //Control Commands - always on
      for (int i = 0;i<vehicle.ctl.NUMSIGNALS;i++) {
        logvars.set(ctr,1,vehicle.ctl.ctlcomms.get(i+1,1));
        ctr++;
      }
      #ifdef RK4_H
      //Integrator States which includes 6DOF states and
      //actuators if there are any
      for (int i = 0;i<integrator.NUMVARS;i++) {
        logvars.set(ctr,1,integrator.State.get(i+1,1));
        ctr++;
      }
      //Forces and Moments
      for (int i = 0;i<3;i++) {
        logvars.set(ctr,1,vehicle.aero.FAEROB.get(i+1,1));
        ctr++;
      }
      for (int i = 0;i<3;i++) {
        logvars.set(ctr,1,vehicle.aero.MAEROB.get(i+1,1));
        ctr++;
      }
      ///Actuator Error Values but only if the user added actuators though
      if (vehicle.NUMACTUATORS > 0) {
        for (int i = 0;i<vehicle.NUMACTUATORS;i++) {
          logvars.set(ctr,1,vehicle.actuatorError.get(i+1,1));
          ctr++;
        }
      }
      #endif
      logger.println(logvars);
      LOG+=LOGRATE;
    }
    ////////////////////////////////////////////////

  } //End while loop on main loop
  printf("Simulation Complete\n");
}
