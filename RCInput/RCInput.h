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

//If on the raspberry pi and running HIL or AUTO mode. We need the receiver
#if defined (HIL) || (AUTO)
#ifdef RPI
#define RECEIVER
#endif
#endif

#ifdef RECEIVER
//Using a receiver on the Raspberry Pi
#include <Common/Util.h>
#define RCIN_SYSFS_PATH "/sys/kernel/rcio/rcin"
#else
#include <linux/joystick.h>
#define JOY_DEV "/dev/input/js0"
#define NAME_LENGTH 80
#endif

class RCInput {
public:
    void initialize();
    void readRCstate();
    void printRCstate(int);
    //Constructor
    RCInput();
    //Destructor
    ~RCInput();
    
    int joy_fd,*axis=NULL,*axis_id=NULL,num_of_axis=0,num_of_buttons=0,x;
    char *button = NULL,name_of_joystick[NAME_LENGTH];
    #ifndef RECEIVER
    struct js_event js;
    #endif
private:
    int open_axis(int ch);
    int read_axis(int ch);
};

#endif
