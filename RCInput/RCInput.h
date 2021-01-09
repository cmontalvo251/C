#ifndef RCIN_H
#define RCIN_H

#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <err.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <iostream>

//////////Here are the iterations

////SIMONLY - no rx at all
////SIL,HIL,AUTO - Need Receiver or Joystick
////    RPI - Receiver
////    Arduino - Receiver Ard
////    Desktop - Joystick 

///Running SIL/HIL/AUTO on RPI - Use Receiver
#if defined (SIL) || (HIL) || (AUTO)
#ifdef RPI
#define RECEIVER
#endif
#endif

//?Running in Realtime on Desktop
//If HIL - RPI handles comms
#if defined (SIL) && (DESKTOP)
#define JOYSTICK
#endif

#ifdef RECEIVER
//Using a receiver on the Raspberry Pi
#include <Common/Util.h>
#endif

#ifdef JOYSTICK
#include <linux/joystick.h>
//Using Microsoft X-Box 360 pad 
//Throttle = 1 (inv)
//Rudder = 0 
//Aileron =  3
//Elevator = 4
//Left Trigger = 2
//Right Trigger = 5
//UD Dpad = 7
//LR Dpad = 6
#endif

//Leaving these defines here just in case. Some are for RPi and some are for joystick
#define RCIN_SYSFS_PATH "/sys/kernel/rcio/rcin"
#define JOY_DEV "/dev/input/js0"
#define NAME_LENGTH 80

///SERVO VALUES
#define STICK_MAX 2016. /*uS*/
#define STICK_MIN 992. /*us*/
#define STICK_MID 1504. /*us*/
#define IDLE 1200. /*uS*/


class RCInput {
public:
    void initialize();
    void readRCstate();
    void printRCstate(int);
    void setStickMin();
    //Constructor
    RCInput();
    //Destructor
    ~RCInput();
    
    int joy_fd,*axis=NULL,*axis_id=NULL,num_of_axis=0,num_of_buttons=0,x;
    char *button = NULL,name_of_joystick[NAME_LENGTH];
    #ifdef JOYSTICK
    struct js_event js;
    #endif
private:
    int open_axis(int ch);
    int read_axis(int ch);
};

#endif
