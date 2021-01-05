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

///If running SIL or HIL and on a desktop (anything with a rendering environment)
//we need to turn on OPENGL this way the PIC actually has something to fly.
#if defined (SIL) || (HIL)
#if defined (DESKTOP)
#define RENDER
#endif
#endif

//If we are rendering use the preferred rendering environment
#ifdef RENDER
#include "opengl.h"
OPENGL render; //this will create our render variable
#endif

///Create a scheduler for everything we want to run
#include "scheduler.h";
scheduler schedule;

//////Main Loop/////////////
int main() {
  
  //Initialize the scheduler
  schedule.init();

  ////Begin Simulation
  schedule.run();	
  
} //end main loop desktop computer
