//////Facility for Aerial Systems and Technology (FAST) Real Time Simulator/////////
/////Initial edited by Carlos Montalvo

//See the following Issue on Github - https://github.com/cmontalvo251/C/issues/3
#include "main.h"

//define Global time parameters 
double t = 0;
double PRINT = 0;
double LOG = 0;
double startTime,current_time,prev_time;
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

  ////////////////////??CHECK FOR SUDO IF RUNNING IN AUTO MODE///////////////
  #ifdef AUTO
  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
    exit(1);
  }
  #endif
  ///////////////////////////////////////////////////////////////////////////

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
  double TELEMRATE = simdata.get(7,1);
  vehicle.setRates(RCRATE,CTLRATE,TELEMRATE);
  //These are extras that we only need if we are integrating the but it doesn't
  //Require any computation time except on startup to read them so just keep
  //them here for all different scenarios
  int GRAVITY_FLAG = simdata.get(8,1);
  int AERO_FLAG = simdata.get(9,1);
  int CTL_FLAG = simdata.get(10,1);
  int ERROR_FLAG = simdata.get(11,1);
  int ACTUATOR_FLAG = simdata.get(12,1);
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

  ////////////////////Initialize Receiver,Controller,PWM outputs///////////////////
  /////This happens regardless of the type of simulation
  vehicle.initController(CTL_FLAG);
  printf("Controller Online \n");
  ///Initialize radio controlled input and output
  //These settings are defined in the Makefile using -D commands
  vehicle.rcio_init();
  //?If running in AUTO mode you need to turn on the sensors
  #ifdef AUTO
  vehicle.err.initSensors(0); //0 for MPU and 1 for LSM
  #endif

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
    icdataALL.zeros(vehicle.NUMVARS,1,"icdataALL");
    icdataALL.vecset(1,vehicle.NUMSTATES,icdata,1);
    icdataALL.vecset(vehicle.NUMSTATES+1,vehicle.NUMVARS,vehicle.actuatorICs,1);
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
  //AND gps heading and compass heading
  vehicle.NUMLOGS+=2;
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
  //We also log all of the receiver signals - also output num of axis
  vehicle.NUMLOGS+=(vehicle.rcin.num_of_axis+1);
  //We also log the control signals
  //Also output the numsignals variable
  vehicle.NUMLOGS+=(vehicle.ctl.NUMSIGNALS+1);
  logger.setLogVars(vehicle.NUMLOGS);
  ///////////////////////////////////////////////////////

  //////////////////Start Rendering Environment Must be done in a boost thread/////////////////
  #ifdef OPENGL_H
  printf("Kicking off OpenGL \n");
  boost::thread render(runRenderLoop,argc,argv);
  //Wait for the opengl routine to actually start
  while (glhandle_g.ready == 0) {
    cross_sleep(1);
  }
  #endif
  /////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////Begin MainLoop///////////////////////////////////////////////
  #ifdef OPENGL_H 
  printf("Kicking off Main Loop as a thread \n");
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
  printf("Running in Real Time \n");
  startTime = timer.getTimeSinceStart();
  current_time = timer.getTimeSinceStart() - startTime;
  PRINTRATE = 0.1;
  #endif
  ///////////////////////////////////////////////////////////////////////////////

  //Kick off main while loop
  while (t < tfinal) {

    /////////////WAIT LOOP//////////////////////////////////
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
    //printf("Getting Current time \n");
    //Keep simulating until user hits CTRL+C when running in AUTO or HIL mode
    tfinal = t+100; 
    //Get Time right now
    current_time = timer.getTimeSinceStart()-startTime;
    #endif
    ///////////////////////////////////////////////////////////////////

    ///////////////////GET TIME ELAPSED################################
    #ifdef AUTO
    INTEGRATIONRATE = current_time - prev_time;
    prev_time = current_time;
    #endif

    //////////////////////Run the Main Vehicle Loop////////////////////
    //This polls the RC inputs and the controller. This will always run
    //when we're on the RPI but only in SIMONLY and SIL on the desktop
    #if defined (SIMONLY) || (SIL) || (RPI)
    //printf("Dynamics Loop \n");
    vehicle.loop(t,INTEGRATIONRATE);
    #endif
    ///////////////////////////////////////////////////////////////////

    ////////////////LOG DATA////////////////////////
    if (LOG<=t) {
      if (logger.IsHeader == 0) {
        logger.logheader[0] = "Time (sec)";
      }
      logger.logvars.set(1,1,t);
      int ctr = 2;
      //GPS always on and 4 states (L,L,H,heading)
      //vehicle.satellites.latitude - I think LL are redundant
      //vehicle.satellites.longitude - because you can recover those from X,Y
      //vehicle.satellites.altitude - same with this one it's just -Z
      //vehicle.satellites.heading - so we just want heading and 
      if (logger.IsHeader == 0) {
        logger.logheader[1] = "GPS Heading (deg)";
        logger.logheader[2] = "Fused GPS+IMU Heading (deg)";
      }
      logger.logvars.set(ctr,1,vehicle.err.getHeading());
      ctr++;
      //Sensors has compass as well
      //vehicle.sensors.compass - the fused compass measurement
      logger.logvars.set(ctr,1,vehicle.err.compass);
      ctr++;
      //Error States (Sensor Measurements) - Always on and always 12 states
      if (logger.IsHeader == 0) {
        logger.logheader[3] = "X Position (m)";
        logger.logheader[4] = "Y Position (m)";
        logger.logheader[5] = "Z Position (m)";
        logger.logheader[6] = "Roll (deg)";
        logger.logheader[7] = "Pitch (deg)";
        logger.logheader[8] = "Yaw (deg)";
        logger.logheader[9] = "U (m/s)";
        logger.logheader[10] = "V (m/s)";
        logger.logheader[11] = "W (m/s)";
        logger.logheader[12] = "P (deg/s)";
        logger.logheader[13] = "Q (deg/s)";
        logger.logheader[14] = "R (deg/s)";
      }
      for (int i = 0;i<12;i++) {
        logger.logvars.set(ctr,1,vehicle.err.errstate.get(i+1,1));
        ctr++;
      }       
      //Receiver Commands - always on
      if (logger.IsHeader == 0) {
        logger.logheader[15] = "Number of RC Signals";
        logger.logheader[16] = "Throttle RX (us)";
        logger.logheader[17] = "Aileron RX (us)";
        logger.logheader[18] = "Elevator RX (us)";
        logger.logheader[19] = "Rudder RX (us)";
      }
      logger.logvars.set(ctr,1,vehicle.rcin.num_of_axis);
      ctr++;
      char*rc;
      for (int i = 0;i<vehicle.rcin.num_of_axis;i++) {
        if (logger.IsHeader == 0) {
          if (i+1>4) {
            rc = new char[12];
            sprintf(rc,"%s%d","Aux ",i+1-4);
            logger.logheader[ctr-1] = rc;
          }
        }
        logger.logvars.set(ctr,1,vehicle.rcin.rxcomm[i]);
        ctr++;
      }
      //PWM Commands - always on
      if (logger.IsHeader == 0) {
        logger.logheader[ctr-1] = "Number of PWM Signals";
      }
      logger.logvars.set(ctr,1,vehicle.rcout.NUMSIGNALS);
      ctr++;
      for (int i = 0;i<vehicle.rcout.NUMSIGNALS;i++) {
        if (logger.IsHeader == 0) {
          rc = new char[12];
          sprintf(rc,"%s%d","PWM ",i+1);
          logger.logheader[ctr-1] = rc;
        }
        logger.logvars.set(ctr,1,vehicle.rcout.pwmcomms[i]);
        ctr++;
      }
      #ifdef RK4_H
      //Integrator States which includes 6DOF states (13) and
      //actuators if there are any
      for (int i = 0;i<integrator.NUMVARS;i++) {
        logger.logvars.set(ctr,1,integrator.State.get(i+1,1));
        ctr++;
      }
      ///Actuator Error Values but only if the user added actuators though
      //When NUMACTUATORS == 0 actuatorError is just a pass through and thus 
      //pointless to log
      if (vehicle.NUMACTUATORS > 0) {
        for (int i = 0;i<vehicle.NUMACTUATORS;i++) {
          logger.logvars.set(ctr,1,vehicle.actuatorError.get(i+1,1));
          ctr++;
        }
      }
      //Forces and Moments
      for (int i = 0;i<3;i++) {
        logger.logvars.set(ctr,1,vehicle.aero.FAEROB.get(i+1,1));
        ctr++;
      }
      for (int i = 0;i<3;i++) {
        logger.logvars.set(ctr,1,vehicle.aero.MAEROB.get(i+1,1));
        ctr++;
      }
      #endif
      if (logger.IsHeader == 0) {
        logger.printheaders();
      }
      logger.println();
      LOG+=LOGRATE;
    }
    ////////////////////////////////////////////////
    
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
    //more standard 6DOF nomenclature. This also include the actuator states
    vehicle.setState(integrator.State,integrator.k); 
    #else
    t = current_time;
    #endif
    ////////////////////////////////////////////

    //////////////IF RENDERING//////////////////    
    #ifdef OPENGL_H
    ////Send state vector to OpenGL and Grab Keyboard State
    glhandle_g.state.UpdateRender(t,vehicle.cg,vehicle.ptp,1,vehicle.rcin.keyboard);
    #endif

    /////////////////Print to STDOUT////////////////
    //vehicle.printRC(-5);
    //printf("\n");
    if (PRINT<t) {
      printf("%lf ",t);
      #ifdef RK4_H
      //for (int i = 0;i<integrator.NUMVARS;i++) {
        //printf("%lf ",integrator.State.get(i+1,1));
      //}
      #endif
      vehicle.printRC(-5); //the zero means just the sticks
      printf(" ::: ");
      vehicle.rcout.print(); //This prints the motor signals
      printf(" ::: ");
      for (int idx = 0;idx<3;idx++){
        printf("%lf ",vehicle.err.errstate.get(4+idx,1));
      }
      printf("\n");
      PRINT+=(1*PRINTRATE);
      //PAUSE();
    }
    /////////////////////////////////////////////////

  } //End while loop on main loop
  printf("Simulation Complete\n");
}
