#include "RCInput.h"

//constructor class
RCInput::RCInput() {
  //Run the initialize routine
  initialize();
}


//Here's the initialize routine
void RCInput::initialize() {

  //Receiver on Raspberry Pi
  #ifdef RECEIVER
  num_of_axis = 8;
  #endif

  ///Joystick on Desktop
  #ifdef JOYSTICK
  printf("Initializing joystick \n");
  if((joy_fd = open(JOY_DEV,O_RDONLY)) == -1 ) {
    printf("Couldn't open joysticks \n");
    exit(1);
  }
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
  #endif

  //Running as fast as possible. Just need a dummy variable here
  #ifdef SIMONLY
  num_of_axis = 6;
  #endif

  printf("Allocating Axes \n");
  printf("Number of axes = %d \n",num_of_axis);
  axis = (int *) calloc(num_of_axis,sizeof(int));
  axis_id = (int *) calloc(num_of_axis,sizeof(int));
  printf("Done \n");

  //Extra stuff on RPi
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

void RCInput::readRCstate()
{
  #ifdef RECEIVER
  for (int idx = 0;idx<num_of_axis;idx++) {
    axis[idx] = read_axis(idx);
  }
  #endif

  #ifdef JOYSTICK
  // cout << "Current time = " <<  << endl;
  // cout << "Reading Joystick state \n";
  read(joy_fd,&js,sizeof(struct js_event));
  // cout << "What is the joystick state? \n";
  switch (js.type & ~JS_EVENT_INIT) {
  case JS_EVENT_AXIS:
    axis[js.number] = js.value;
    break;
  case JS_EVENT_BUTTON:
    button[js.number] = js.value;
    break;
  }
  #endif

  #ifdef SIMONLY
  for (int idx = 0;idx<num_of_axis;idx++) {
    axis[idx] = 1500; //Middle of PWM signal
  }
  #endif
}

void RCInput::printRCstate(int all) {
  printf("Axis State = ");
  for (x = 0;x<num_of_axis;x++){
    printf("%d ",axis[x]);
  }
  #ifdef JOYSTICK
  if (all == 1) {
    printf(" Button State = ");
    for (x = 0;x<num_of_buttons;x++){
      printf("%d ",button[x]);
    }
  }
  #endif
  printf("\n");
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
