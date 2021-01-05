#include "scheduler.h";

//Constructor
scheduler::scheduler() {

};

void scheduler::init() {  
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
  ok = logger.ImportFile("Input_Files/Simulation_Flags.txt",&simdata,"simdata",vehicle.NUMSTATES);
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
}

void::scheduler::run() {
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