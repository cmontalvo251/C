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

*/

/* //Revisions Needed 

////Things to do before you move to FASTPilot

2.) Actuator dynamics
3.) Sensor Noise

/// Things you can do on desktop

0.) Link code to FASTPilot
2.) CAD Model of X8
4.) Quadcopter dynamics
5.) Aircraft dynamics
6.) X8 dynamics
7.) Quadcopter autopilot
8.) aircraft autopilot
9.) X8 autopilot

//// Things you have to do with an RPI

2.) Run SIMONLY on RPI
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
  glhandle_g.loop(argc,argv,Farplane,width,height,defaultcamera);
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
