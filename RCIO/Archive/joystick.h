#ifndef JOYSTICK_H
#define JOYSTICK_H

/////FOR JOYSTICKS/////
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <iostream>
#include <linux/joystick.h>
#define JOY_DEV "/dev/input/js0"
#define NAME_LENGTH 80

using namespace std;

class joystick {
	private:
		int joy_fd,*axis=NULL,num_of_axis=0,num_of_buttons=0,x;
		char *button = NULL,name_of_joystick[NAME_LENGTH];
		struct js_event js;
	public:
		//constructor
		joystick();
		int init();
		void readJoystickState();
		void printJoystickState();
};

#endif