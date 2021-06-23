#ifndef MAIN_H
#define MAIN_H

#include <Datalogger/Datalogger.h> //Always need a data logger
#include "Dynamics.h"; //Always need vehicle dynamics
//Always need timer for the pause function and the error function. 
//May not need to create a TIMER class but still need the built-in functions
#include <Timer/timer.h>
#include <iostream>
#include <stdlib.h>

///If running SIL or HIL and on a desktop (anything with a rendering environment)
//we need to turn on OPENGL this way the PIC actually has something to fly.
#if defined (SIL) || (HIL)
#if defined (DESKTOP)
//If we are rendering use the preferred rendering environment
#include <OpenGL/opengl.h>
#endif
#endif

#if defined (SIL) || (SIMONLY) || (HIL)
//If you're simulating the vehicle you have to have an RK4 Engine
//that integrates the equations of motion
#include <RK4/RK4.h>
#endif

//If we are running SIL/HIL we are simulating the flight control board
//which means we need to run in realtime.
//Obviously AUTO must run in realtime as well because that will help with all our timing
#if defined (SIL) || (HIL) || (AUTO)
#define REALTIME
#endif

///Functions
void runMainLoop();
void runRenderLoop(int argc,char** argv);

#endif