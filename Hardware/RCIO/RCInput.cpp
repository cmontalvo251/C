#include "RCInput.h"

//constructor class
RCInput::RCInput() {
}

void RCInput::initialize() {
  //Receiver on Raspberry Pi
  //Or running as fast as possible
  num_of_axis = 8; //default num_of_axis to 8 no matter what.
  //It might change if you have a joystick but at least it's initialized to 8

  ///Joystick on Desktop
  #ifdef JOYSTICK
  printf("Initializing joystick \n");
  if((joy_fd = open(JOY_DEV,O_RDONLY)) == -1 ) {
    printf("Couldn't open joysticks \n");
    printf("Defaulting to Stick min \n");
    printf("joy_fd = %d \n",joy_fd);
  } else {
    printf("Getting information from the joystick \n");
    ioctl(joy_fd,JSIOCGAXES,&num_of_axis);
    ioctl(joy_fd,JSIOCGBUTTONS,&num_of_buttons);
    printf("Number of buttons = %d \n",num_of_buttons);
    ioctl(joy_fd,JSIOCGNAME(NAME_LENGTH),&name_of_joystick);
    printf("Name of Joystick = %s \n",name_of_joystick);
    printf("Allocating Buttons for Joysticks. Cross your fingers \n");
    button = (char *) calloc(num_of_buttons,sizeof(int));
    printf("Success!!!! \n");
    printf("Setting non-blocking mode \n");
    fcntl(joy_fd,F_SETFL,O_NONBLOCK);
  }
  #endif

  printf("Allocating Axes \n");
  printf("Number of axes = %d \n",num_of_axis);
  rxcomm = (int *) calloc(num_of_axis,sizeof(int));
  joycomm = (int *) calloc(num_of_axis,sizeof(int));
  axis_id = (int *) calloc(num_of_axis,sizeof(int));
  printf("Done \n");

  //Extra stuff on RPi using a Receiver
  #ifdef RECEIVER
  for (size_t i = 0; i < num_of_axis; i++) {
    axis_id[i] = open_axis(i);
    if (axis_id[i] < 0) {
      printf("Error opening an axis \n");
      perror("open");
    }
  }
  #endif
}

void RCInput::setStick(int val) {
  //printf("Setting stick to neutral \n");
  for (int idx = 0;idx<num_of_axis;idx++) {
    rxcomm[idx] = val; //STICK_MIN #define
  }
}

void RCInput::LostCommCheck() {
  int lostcomms = 0;
  for (int idx = 0;idx<4;idx++) {
    if (rxcomm[idx] == 0) {
      lostcomms = 1;
    }
  }
  if (lostcomms == 1) {
    setStickNeutral();
  }
}

void RCInput::setStickNeutral() {
  setStick(STICK_MIN); //First set all sticks to min
  //but then set the Aileron, Elevator and Rudder to mid
  rxcomm[1] = STICK_MID;
  rxcomm[2] = STICK_MID;
  rxcomm[3] = STICK_MID;
}

int RCInput::bit2PWM(int val) {
  ///the values from the joystick are from -32678 to 32768 which is a 16 bit number
  //printf("val = %d \n");
  return (STICK_MAX-STICK_MID)*val/BIT_RANGE + STICK_MID;
}

void RCInput::readRCstate()
{
  #ifdef KEYBOARD
  for (int idx = 0;idx<num_of_axis;idx++) {
    //printf("%lf ",keyboard[idx]);
    if (idx<4) {
      rxcomm[idx] = keyboard[idx]*(STICK_MAX-STICK_MIN)/2.0 + STICK_MID;
    } else {
      rxcomm[idx] = STICK_MIN;
    }
  }
  //printf("\n");
  //printRCstate(-4);
  #endif

  #ifdef RECEIVER
  //printf("Reading from Receiver \n");
  for (int idx = 0;idx<num_of_axis;idx++) {
    rxcomm[idx] = read_axis(idx);
  }
  #endif

  #ifdef JOYSTICK
  if (joy_fd != -1) {
    // cout << "Current time = " <<  << endl;
    //cout << "Reading Joystick state \n";
    read(joy_fd,&js,sizeof(struct js_event));
    // cout << "What is the joystick state? \n";
    switch (js.type & ~JS_EVENT_INIT) {
    case JS_EVENT_AXIS:
      //if (js.number == 5) {
      //  printf("js.value and number = %d %d \n",js.number,js.value);
      //}
      joycomm[js.number] = bit2PWM(js.value);
      break;
    case JS_EVENT_BUTTON:
      button[js.number] = js.value;
      break;
    }
    mapjoy2rx();
  } else {
    setStickNeutral();
  }
  #endif

  #ifdef SIMONLY
  setStickNeutral();
  #endif

  LostCommCheck();

  //printf("TTTT ");
  //printRCstate(-4);
  //printf("FFFF ");
}

int RCInput::invert(int val) {
  int out;
  //A value that is STICK_MIN will produce STICK_MAX and viceversa
  if (val != 0) {
    int delta = val - STICK_MID;
    int inverse = -delta;
    out = STICK_MID + inverse;
  } else {
    //A value of zero means loss of signal
    out = 0;
  }
  return out;
}

void RCInput::mapjoy2rx() {
  //First we're just going to copy everything over to make sure everything copies over
  for (int i = 0;i<num_of_axis;i++) {
    rxcomm[i] = joycomm[i];
  }

  //The code below will then run depending on what controller you've selected in the makefile

  //First extract the relavent commands from the receiver.
  //double throttle = rxcomms[0];
  //double aileron = rxcomms[1];
  //double elevator = rxcomms[2];
  //double rudder = rxcomms[3];
  //double autopilot = rxcomms[4];

  #ifdef XBOX
  //So the problem is that my Xbox controller is not mapped properly. Here
  //is the mapping
  //Using Microsoft X-Box 360 pad 
  //Throttle = 1 (inv)
  //Aileron =  3
  //Elevator = 4
  //Rudder = 0 
  //Left Trigger = 2
  //Right Trigger = 5
  //UD Dpad = 7
  //LR Dpad = 6
  rxcomm[0] = invert(joycomm[1]);
  rxcomm[1] = joycomm[3];
  rxcomm[2] = invert(joycomm[4]);
  rxcomm[3] = joycomm[0];
  rxcomm[4] = joycomm[2];
  rxcomm[5] = joycomm[5];
  rxcomm[6] = joycomm[7];
  rxcomm[7] = joycomm[6];
  //printf("PRINTING::");
  //printRCstate(0);
  //printf("joycomm[4] = %d \n",joycomm[4]);
  //printf("rxcomm[2] = %d \n",rxcomm[2]);
  #endif

  #ifdef RCTECH
  //Using RCTECH Controller
  //Throttle = 2 (inv)
  //Rudder = 5
  //Aileron =  0
  //Elevator = 1
  //arm switch = 3
  //aux 0 = 4
  rxcomm[0] = invert(joycomm[2]);
  rxcomm[1] = joycomm[0];
  rxcomm[2] = joycomm[1];
  rxcomm[3] = joycomm[5];
  rxcomm[4] = joycomm[3];
  rxcomm[5] = joycomm[4];
  //rxcomm[6] = joycomm[6];
  //rxcomm[7] = joycomm[7];
  #endif
}

void RCInput::printRCstate(int all) {
  //printf("Axis State = ");
  int val = num_of_axis;
  if (all < 0) {
    val = -all;
  }
  for (x = 0;x<val;x++){
    printf("%d ",rxcomm[x]);
  }
  #ifdef JOYSTICK
  if (all == 1) {
    //printf(" Button State = ");
    for (x = 0;x<num_of_buttons;x++){
      printf("%d ",button[x]);
    }
  }
  #endif
  //printf("\n");
}

#ifdef RECEIVER
int RCInput::open_axis(int channel)
{
  char *channel_path;
  if (asprintf(&channel_path, "%s/ch%d", RCIN_SYSFS_PATH, channel) == -1) {
    err(1, "channel: %d\n", channel);
  }
  int fd = ::open(channel_path, O_RDONLY);
  free(channel_path);
  return fd;
}

//This is where we read the axis
int RCInput::read_axis(int ch)
{
  if (ch > num_of_axis)
    {	
      fprintf(stderr,"Channel number too large\n");
      return -1;
    }
  char buffer[10];
  //so this is some fancy bit shit bullcrap that only computer scientist know how this works
  if (::pread(axis_id[ch], buffer, ARRAY_SIZE(buffer), 0) < 0) {
    perror("pread");
  }
  //this function atoi converts a char to a integer
  return atoi(buffer);
}
#endif

//empty destructor class
RCInput::~RCInput(){
}
