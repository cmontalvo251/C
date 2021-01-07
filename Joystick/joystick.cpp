#include "joystick.h"

//constructor
joystick::joystick() {
	
}

///Running lsusb produces
// Bus 003 Device 019: ID 046d:c216 Logitech, Inc. Dual Action Gamepad
// Looking online I found that joysticks default to /dev/input/js0

int joystick::init() {
	printf("Initializing joystick \n");
  	if( ( joy_fd = open(JOY_DEV,O_RDONLY)) == -1 ) {
    	printf("Couldn't open joysticks \n");
    	return 0;
  	}
	printf("Getting information from the joystick \n");
	ioctl(joy_fd,JSIOCGAXES,&num_of_axis);
	printf("Number of axes = %d \n",num_of_axis);
	ioctl(joy_fd,JSIOCGBUTTONS,&num_of_buttons);
	printf("Number of buttons = %d \n",num_of_buttons);
	ioctl(joy_fd,JSIOCGNAME(NAME_LENGTH),&name_of_joystick);
	printf("Name of Joystick = %s \n",name_of_joystick);
	printf("Allocating Memory for Joysticks. Cross your fingers \n");
	axis = (int *) calloc(num_of_axis,sizeof(int));
	button = (char *) calloc(num_of_buttons,sizeof(int));
	printf("Success!!!! \n");
	printf("Setting non-blocking mode \n");
	fcntl(joy_fd,F_SETFL,O_NONBLOCK);
	return 1;
}

void joystick::readJoystickState() {
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
}

void joystick::printJoystickState() {
	printf("Axis State = ");
   	for (int x = 0;x<num_of_axis;x++){
    	printf("%d ",x,axis[x]);
    }
 	printf(" Button State = ");
    for (int x = 0;x<num_of_buttons;x++){
    	printf("%d ",button[x]);
    }
}
