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
you set it up correctly is the main object

*/

/* //Revisions Needed 

A couple bugs I'd like to fix before moving on to the Aircraft and Quad dynamics
2.) right now the camera is in the ground so you can't see the ground. The easiest thing
to do in my opinion is to put the camera at 0,0,10 or something. The other option would
be to move all objects down but that sounds like a bad hack. Maybe in the Render.txt
you can add an offset to shift everything?
3.) In the dynamics routine I would like to add a stop if you hit the ground. Either full
stop and you have to restart or make the zdot derivative zero. Don't worry about the sky
for now. That's too complicated. Once you've made these changes I think you can move 
to the airplane and quad.

//SIMONLY DESKTOP - Working
//SIL DESKTOP - Working

//SIMONLY RPI - Needs testing
//SIL RPI - no need to test but what happens if you compile in this mode?
//HIL DESKTOP and RPI - Need to add Serial debugging
//AUTO DESKTOP - What happens if you compile in this mode?
//AUTO RPI - Need to poll all the sensors

//Call the sensor block which polls fictitious sensors on desktop
//Send state vector via serial if HIL
//Read control vector via serial if HIL

Aero and Autopilot models needs
Aircraft
Quad
X8

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

//////Main/////////////
int main(int argc,char** argv) {
  //////////////////////////Initialize Datalogger//////////////////////////////
  logger.findfile("logs/");
  logger.open();
  logvars.zeros(vehicle.NUMLOGS,1,"Vars to Log"); //All states + time
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////Import Simulation Flags//////////////////////////////////
  MATLAB simdata;
  int ok = logger.ImportFile("Input_Files/Simulation_Flags.txt",&simdata,"simdata",-99);
  if (!ok) { exit(1); } else {simdata.disp();}
  tfinal = simdata.get(1,1);
  INTEGRATIONRATE = simdata.get(2,1);
  PRINTRATE = simdata.get(3,1); 
  LOGRATE = simdata.get(4,1); //We need to do this no matter what because the LOGRATE is required all the time
  int GRAVITY_FLAG = simdata.get(5,1);
  int AERO_FLAG = simdata.get(6,1);
  int CTL_FLAG = simdata.get(7,1);
  //////////////////////////////////////////////////////////////////////////////

  //////////////////Initialize timer if code running realtime////////////////////
  #ifdef REALTIME
  startTime = timer.getTimeSinceStart();
  current_time = timer.getTimeSinceStart() - startTime;
  PRINTRATE = 1.0;
  #endif
  ///////////////////////////////////////////////////////////////////////////////

  /////////////////Initialize RK4 if simulating Dynamics///////////////////
  #ifdef RK4_H
  //First Initialize integrator and Dynamic Model
  integrator.init(vehicle.NUMSTATES,INTEGRATIONRATE);
  //Import Initial Conditions, mass properties
  MATLAB icdata;
  ok = logger.ImportFile("Input_Files/Initial_Conditions.txt",&icdata,"icdata",vehicle.NUMSTATES);
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

  /////////////////////////////MASS DATA, ENV MODEL, AERO MODEL, CTL MODEL///////////////////
  MATLAB massdata;
  ok = logger.ImportFile("Input_Files/MassProperties.txt",&massdata,"massdata",4);
  if (!ok) { exit(1); } else {massdata.disp();}
  //Send Mass Data to Dynamic Model
  vehicle.setMassProps(massdata);
  //Initialize Environment, Aero and Control System
  vehicle.initExtModels(GRAVITY_FLAG,AERO_FLAG,CTL_FLAG);
  #ifdef DEBUG
  printf("Extra Models Initialized \n");
  #endif
  
  //////////////////Start Rendering Environment Must be done in a boost thread/////////////////
  #ifdef OPENGL_H
  boost::thread render(runRenderLoop,argc,argv);
  cross_sleep(1); //Give the render a bit of time to start
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
  glhandle_g.loop(argc,argv,Farplane,width,height,defaultcamera);
}
#endif
////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////MAIN LOOP/////////////////////////////////////////////
void runMainLoop() {
  #ifdef DEBUG
  printf("Running MainLoop \n");
  #endif

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
    //This polls the RC inputs and the controller but only if we're HIL and DESKTOP
    //are not defined. Instead we need to send the state vector to the external hardware
    #if not defined (HIL) && (DESKTOP)
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
      vehicle.printRC(0); //the zero means just the sticks
      printf("\n");
      PRINT+=PRINTRATE;
    }
    /////////////////////////////////////////////////

    ////////////////LOG DATA////////////////////////
    if (LOG<t) {
      logvars.set(1,1,t);
      int ctr = 2;
      for (int i = 0;i<integrator.NUMSTATES;i++) {
        logvars.set(ctr,1,integrator.State.get(i+1,1));
        ctr++;
      }
      for (int i = 0;i<vehicle.rcin.num_of_axis;i++) {
        logvars.set(ctr,1,vehicle.rcin.rxcomm[i]);
        ctr++;
      }
      logger.println(logvars);
      LOG+=LOGRATE;
    }
    ////////////////////////////////////////////////

  } //End while loop on main loop
  printf("Simulation Complete\n");
}
