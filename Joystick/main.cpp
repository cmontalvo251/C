#include <iostream>
#include <math.h>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
/////FOR JOYSTICKS/////
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#define JOY_DEV "/dev/input/js0"
#define NAME_LENGTH 80
//////////////////////
#define PI 3.14159265358979
#include "timer.h"

///Running lsusb produces

// Bus 003 Device 019: ID 046d:c216 Logitech, Inc. Dual Action Gamepad
// Looking online I found that joysticks default to /dev/input/js0


using namespace std;


int main (int argc,char ** argv) {
  
  int joy_fd, *axis=NULL,num_of_axis=0,num_of_buttons=0,x;
  char *button = NULL,name_of_joystick[NAME_LENGTH];
  TIMER time;
  int AUTOPILOT = 0,PRESS = 0;
  double DEU,DAU,DELTHRUSTU,DRU,nowtime,itime=0;
  struct js_event js;

  cout << "Let's try and open the joystick \n";
  if( ( joy_fd = open(JOY_DEV,O_RDONLY)) == -1 ) {
    cout << "Couldn't open joysticks \n";
    return 0;
  } else {
    cout << "Success!!! \n";
  }
  
  cout << "Now let's get information from the joystick \n";
  
  ioctl(joy_fd,JSIOCGAXES,&num_of_axis);
  
  cout << "Number of axes = " << num_of_axis << endl;
  
  ioctl(joy_fd,JSIOCGBUTTONS,&num_of_buttons);

  cout << "Number of buttons = " << num_of_buttons << endl;

  ioctl(joy_fd,JSIOCGNAME(NAME_LENGTH),&name_of_joystick);

  cout << "Name of Joystick = " << name_of_joystick << endl;

  cout << "Allocating Memory for Joysticks. Cross your fingers" << endl;
  
  axis = (int *) calloc(num_of_axis,sizeof(int));
  button = (char *) calloc(num_of_buttons,sizeof(int));

  cout << "Success!!!! \n";

  cout << "Setting non-blocking mode \n";
  
  fcntl(joy_fd,F_SETFL,O_NONBLOCK);

  //Create an infinite loop
  while (1) {
    nowtime = time.getTimeSinceStart();
    // cout << "Current time = " <<  << endl;
    // cout << "Reading Joystick state \n";
    read(joy_fd,&js,sizeof(struct js_event));
    // cout << "What is the joystick state? \n";
    switch (js.type & ~JS_EVENT_INIT) 
      {
      case JS_EVENT_AXIS:
	axis[js.number] = js.value;
	break;
      case JS_EVENT_BUTTON:
	button[js.number] = js.value;
	break;
      }

    //Turn autopilot on or off
    if (button[1] != NULL && PRESS == 0) {
      PRESS = 1;
      if (AUTOPILOT) {
	cout << "AUTOPILOT OFF \n";
	AUTOPILOT = 0;
      } else {
	cout << "AUTOPILOT ON \n";
	AUTOPILOT = 1;
      }
    }
    if (button[1] == NULL && PRESS == 1){
      PRESS = 0;
    }

    //Now link axes to control surfaces 
    //Elevator is axis 3
    //Aileron is axis 2
    //Throttle is axis 1
    //Rudder is axis 0 
    if (nowtime > itime) {
      itime+=0.1;
      DEU = 30*axis[3]/(double)32000;
      cout << "Elevator Angle (deg) = " << DEU << endl;
      DAU = 30*axis[2]/(double)32000;
      cout << "Aileron Angle (deg) = " << DAU << endl;
      DELTHRUSTU = -(axis[1]/(double)32000);
      cout << "Thrust (+-1) = " << DELTHRUSTU << endl;
      DRU = 30*axis[0]/(double)32000;
      cout << "Rudder Angle (deg) = " << DRU << endl;
    }

    // cout << "Axis State... \n";

    // for (x = 0;x<num_of_axis;x++){
    //   cout << "Axis Number = " << x << " State = " << axis[x]/(double)32000 << endl;
    // }

    // cout << "Button State... \n";
    
    // for (x = 0;x<num_of_buttons;x++){
    //   cout << "Button Number = " << x << " State = " << button[x] << endl;
    // }

    //cross_sleep(0.1);
  }

}
