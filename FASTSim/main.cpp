//////Facility for Aerial Systems and Technology (FAST) Real Time Simulator/////////
/////Initial edited by Carlos Montalvo

//See the following Issue on Github - https://github.com/cmontalvo251/C/issues/3

/*///Revisions created - 12/10/2020 - Added Loop timer

1/1/2021 - Added Datalogger, RK4 routine and point mass model in space using 
Dynamics class

1/2/2021 - Added point mass model on flat earth, added environment class. 
Fixed some compilation flags.

1/5/2021 - Added opengl but system does not move. Still need to add boost 
threads to get this to work properly

1/6/2021 - opengl model is finally working and ready to go!!!

1/7/2021 - Added 6DOF dynamic model. Added RCInput class but it is not 
implemented. My suggestion is to make a bare bones aerodynamic model first 
and test open loop

1/8/2021 - Alright I added placeholders for aerodynamics and controller models with 
hooks in place. We basically need to test everything I just wrote. My vote is to get
the fictitious sensors to work. After that my vote is to work on the aircraft autopilot.
Then to work on the quad autopilot

1/9/2021 - Test aerodynamics and autopilot. Autopilot is not completely done but I'm making
progress

1/10/2021 - Added full autopilot for portal cube, I have not coded a stabilize mode but I
I think I know how to do it. If you hold the left joystick you can turn the autopilot on
and off just like a trainer switch. I also created a mapping function to map the joystick
keys to standard receiver inputs

1/25/2021 - Changed the default camera to follow the first object in the series which if
you set it up correctly is the main object. I also added a zaxis offset to the camera in
render.txt file so now you can have the camera slightly above the ground. I also added a
ground plane check. When you're below the ground and with a negative zdot you stop. It's
not completely tested but at least it somewhat works for now. I would say the sim is ready
to add the quadcopter obj and dynamic model. 

1/26/2021 - Added some comments to make sure I know what to do next. I made a CAD for a 
quadcopter but it sucks. I downloaded a *.blend from the internet and tried to make an
OBJ but it just looks terrible. I did learn alot though. What you need to do is add a 
texture to the UV tab. Then select all the vertices in edit of your object and hit U 
for unwrap. Once you do that you can export everything to an OBJ. If you did it right
you will see v for vertices vn for normals vt for texture vertices and then f for face
in the format f v/vt/vn. The quadcopter I did create was so freaking big that the 
integrator would just kind of keep integrating in the background before you could
see what was happening. So I created a wait loop in the main routine here to cross_sleep
until the opengl routine was operational. My recommendation is to get a better CAD model
by just building a CAD model yourself or getting a grad student to do it.

1/27/2021 - Still kind of upset at how last night went I created a simple quadcopter.obj
and threw it into FASTPilot. That's a private repo but the fact is you can make your own
obj file provided you use the same format as the cube that's in this repo.

1/28/2021 - Sensors errors have been added. It doesn't necessarily work and there's a ton
of holes in the system but hey you can add sensor errors and sensor noise and you can plot
the output of those sensors. Anyway the next thing to add in my opinion is the control
cycle. That seems like an easy thing to add

1/29/2021 - Add polling rates for the Receiver and the control loop. I think that's every
thing we need for polling rates. I would say the next thing to work on is actuator dynamics.
For actuator dynamics I'm not entirely sure how I want to do this. So forces in this routine
are all in the aerodynamics model. So I think we need to have an extra error value (in percent)
in the simulation flags portion and then send it to the aero model. Let's try that. Oh ok wait 
this is for actuator errors. Lol. Let's just do that anyway right now. Ok not entirely sure if it's
exactly how I want it but actuator errors are in the sim now. It's in the aerodynamics model which I'm
not too crazy about. I'd rather it be in maybe the dynamics class or something but I'm not sure what
to do here. At least the hooks are in there properly. So the last thing I need to do is add the actuator
dynamics. Can you tell I've been delaying the crap out of it?

2/4/2021 - Compiled SIMONLY on RPI. Only thing you need to do is get rid of OPENGL in the Makefile and the
make_links script. 

My plan for actuator dynamics is this - In some input file you need to tell the dynamics routine the number
of actuators and the time constant of each one. Then it will run through a for loop and integrate those dynamics
The first order filter equation is as follows
actuator_var_dot = time_constant*(actuator_command - actuator_var)
So the question then is what is the actuator command???? Is this from the controller??
On line 184 of Dynamics.cpp I send ctl.ctlcomms to the aero model. So what I need to do is 
send the cltcomms to a first order filter and then send state.actuator_var to the aero model
So I had the aerodynamics.h routine set the number of actuators rather then set it in the input files
so all you need to do is create a public variable in aerodynamics.h and the have the dynamics routine 
use that to create dynamics. Seems simple enough

2/9/2021 - When I compiled SIMONLY on the RPI it turns out that I didn't have the -DRPI directive in place. 
So I had to fix a few things. It is working now which is nice.

2/26/2021 - Added RCTECH USB contorller in the lab to FASTsim. To get this to work you need to run RCInput by iself
and make sure the mapping is set up correctly. Then change the stick min and max values in RCINput.h and then add a -D
to the makefile for this controller

Also tried to import a UAM cad modle but it has wayyyy too many vertices and no texture for mapping

4/1/2021 - Been busy. Started writing a README to get up and running because it's been a while. I also started
doing this revision "Add separate input file folders and have argv grab the root directory of your input files 
including the objs and stuff". So I finished the README and I got the input files to work properly which is nice.
Collin also sent me an X8 model. The textures are super messed up but hey it's a start

4/30/2021 - Long time no see. I am here fixing the plotting routine because for some reason the commands
from the controller are printing correctly but the actual values are not. Currently investigating the issue here. 
Ok false alarm. Turns out those values were actually from the RX which are always empty anyway in SIMONLY mode.

Ok so it's time to continue working on Actuator dynamics. So I looked at what I did and I don't like it.
In my opinion the user should only be in charge of the following things. 

actuator values -> Forces produced by those values

So what we need to do is the following. 

1.) Integrate the state vector to produce State.
2.) Send STate to the sensor routine and receive the polluted state
3.) Send pollutted state to controller built by the user to produce ctlcomms
4.) Send ctlcomms to actuator dynamics to produce actuators values
5.) Pollute actuator values with ACTUATOR ERRORS by some percentage
6.) Send actuator values to aerodynamics routine written by user to produce Forces and moments

Ok let's get started. 

1.) Integrate state vector to produce state. That's done in the integrator.state
line of code so we're done there.
2.) Send State to the sensor routine to receive polluted state. 

*/

/* //Revisions Needed 

////Things to do before you move to FASTPilot

2.) Actuator dynamics 

/// Things you can do on desktop

0.) Link code to FASTPilot
4.) Quadcopter dynamics
5.) Aircraft dynamics
6.) X8 dynamics
7.) Quadcopter autopilot
8.) aircraft autopilot
9.) X8 autopilot

//// Things you have to do with an RPI

3.) Run SIL on RPI - This mode should probably throw an error in my opinion since you
can't run opengl. 
4.) Run HIL on Desktop and RPI - You need to write two instances of the same software.
The RPI/HIL version will accept a polluted state vector from the computer and run that
through the control algorithm and then pass back the control commands to the integrator.
So serialListens and SerialResponds etc will need to be written to send data back and
forth
5.) AUTO - In this mode the sensors will be called in the loop and the commands will
be compute using the autopilot. These commands will then be sent to the actuators.
6.) RPI IMU
7.) RPI Rcin needs to be overhauled
8.) RPI Rcout needs to be overhauled
9.) RPI GPS
10.) RPI Barometer - This one bugs me the most

///Potential Issues in the future

1.) We have an issue with which variables we log to the SD card.
Right now the number of variables is set in the Dynamics.cpp setup routine
but it seems more realistic to have the programmer decide which variables they want
logged to the SD card. As such I wonder if in the init routine for the logger
we compute which states we want logged. I think once we start porting this over
to the Raspberry Pi and running in AUTO mode we will have to start doing a large amount
of overhaul to get this to work.

*/

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
  logvars.zeros(vehicle.NUMLOGS,1,"Vars to Log"); //All states + time
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
  //Require any computation time except on startup to read them 
  int GRAVITY_FLAG = simdata.get(7,1);
  int AERO_FLAG = simdata.get(8,1);
  int CTL_FLAG = simdata.get(9,1);
  int ERROR_FLAG = simdata.get(10,1);
  int ACTUATOR_ERROR_PERCENTAGE = simdata.get(11,1);
  //////////////////////////////////////////////////////////////////////////////

  /////////////////Initialize RK4 if simulating Dynamics///////////////////
  #ifdef RK4_H
  //First Initialize integrator and Dynamic Model
  integrator.init(vehicle.NUMSTATES,INTEGRATIONRATE);
  //Import Initial Conditions, mass properties
  MATLAB icdata;
  char initfile[256]={NULL};
  strcat(initfile,fileroot);
  strcat(initfile,"Input_Files/Initial_Conditions.txt");
  ok = logger.ImportFile(initfile,&icdata,"icdata",vehicle.NUMSTATES);
  if (!ok) { exit(1); } else {icdata.disp();}
  //Set ICs in integrator 
  integrator.set_ICs(icdata);
  //Set State of Vehicle for use elsewhere
  vehicle.setState(integrator.State,integrator.k);
  #ifdef DEBUG
  printf("Integrator Initialized\n");
  #endif
  #endif
  ////////////////////////////////////////////////////////////////////////////

  /////////////////////////////MASS DATA, ENV MODEL, SENSOR MODEL, AERO MODEL, CTL MODEL///////////////////
  MATLAB massdata;
  char massfile[256]={NULL};
  strcat(massfile,fileroot);
  strcat(massfile,"Input_Files/MassProperties.txt");
  ok = logger.ImportFile(massfile,&massdata,"massdata",4);
  if (!ok) { exit(1); } else {massdata.disp();}
  //Send Mass Data to Dynamic Model
  vehicle.setMassProps(massdata);
  //Initialize Environment, Aero and Control System
  vehicle.initAerodynamics(AERO_FLAG,ACTUATOR_ERROR_PERCENTAGE);
  vehicle.initExtModels(GRAVITY_FLAG,CTL_FLAG);
  #ifdef RK4_H
  //Initialize the Error Model but only if we're running the integrator
  if (ERROR_FLAG) {
    MATLAB sensordata;
    char sensorfile[256]={NULL};
    strcat(sensorfile,fileroot);
    strcat(sensorfile,"Input_Files/Sensor_Errors.txt");
    ok = logger.ImportFile(sensorfile,&sensordata,"sensordata",-99); //-99 for automatic length array
    if (!ok) { exit(1); } else {sensordata.disp();}    
    vehicle.initErrModel(sensordata);
  }
  #endif
  #ifdef DEBUG
  printf("Extra Models Initialized \n");
  #endif
  
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
    //Set State of Vehicle for use elsewhere
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
      for (int i = 0;i<integrator.NUMSTATES;i++) {
        printf("%lf ",integrator.State.get(i+1,1));
      }
      //vehicle.printRC(0); //the zero means just the sticks
      printf("\n");
      PRINT+=PRINTRATE;
    }
    /////////////////////////////////////////////////

    ////////////////LOG DATA////////////////////////
    if (LOG<t) {
      logvars.set(1,1,t);
      int ctr = 2;
      #ifdef RK4_H
      //Integrator States
      for (int i = 0;i<integrator.NUMSTATES;i++) {
        logvars.set(ctr,1,integrator.State.get(i+1,1));
        ctr++;
      }
      #endif
      //Error States
      for (int i = 0;i<integrator.NUMSTATES-1;i++) {
        logvars.set(ctr,1,vehicle.err.errstate.get(i+1,1));
        ctr++;
      }      
      //Receiver Commands
      for (int i = 0;i<vehicle.rcin.num_of_axis;i++) {
        logvars.set(ctr,1,vehicle.rcin.rxcomm[i]);
        ctr++;
      }
      //Control Commands
      for (int i = 0;i<vehicle.ctl.NUMSIGNALS;i++) {
        logvars.set(ctr,1,vehicle.ctl.ctlcomms.get(i+1,1));
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
      logger.println(logvars);
      LOG+=LOGRATE;
    }
    ////////////////////////////////////////////////

  } //End while loop on main loop
  printf("Simulation Complete\n");
}
