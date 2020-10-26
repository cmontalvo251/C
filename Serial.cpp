#include "Serial.h"

HANDLE my;

//Call this for defaults
void InitSerialPort(void)
{
  // VERY IMPORTANT: Edit this line of code to designate which COM port the ADCS board is using!!
  int BaudRate = 115200;
  #ifdef __WIN32__
    char *port = "\\\\.\\COM12";
  #endif
  #if defined __linux__ || __APPLE__
    char *port = "/dev/ttyUSB0";
  #endif
  my=SerialInit(port,BaudRate);
}


//Call this for higher level control
HANDLE SerialInit(char *ComPortName, int BaudRate) 
{
  HANDLE hComm;

  #ifdef RPI
  if(wiringPiSetup() == -1) {
      fprintf(stdout, "Unable to start wiringPi: %s\n", strerror (errno));
      return 1;
    }
  hComm = serialOpen(ComPortName,BaudRate);
  if (hComm < 0) {
      fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
      return 1;
    }
  return hComm;
  #endif
  
  //Setup for Windows
  #ifdef __WIN32__  
  hComm = CreateFile(ComPortName,
 	GENERIC_READ | GENERIC_WRITE,
        0, // exclusive access
        NULL, // no security
        OPEN_EXISTING,
        0, // no overlapped I/O
        NULL); // null template 
    //printf("x0 hComm=0x%08x GetLastError=%d\r\n",hComm,GetLastError());
  bPortReady = SetupComm(hComm, 128, 128); // set buffer sizes
  //printf("x1 PortReady=%d GetLastError=%d\r\n",bPortReady,GetLastError());
  bPortReady = GetCommState(hComm, &dcb);
  // printf("x2 PortReady=%d\r\n",bPortReady);
  dcb.BaudRate = BaudRate;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  //dcb.Parity = EVENPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fAbortOnError = TRUE;
 
  // set XON/XOFF
  dcb.fOutX = FALSE;                    // XON/XOFF off for transmit
  dcb.fInX    = FALSE;                    // XON/XOFF off for receive
  // set RTSCTS
  dcb.fOutxCtsFlow = FALSE;                    // turn on CTS flow control
  dcb.fRtsControl = RTS_CONTROL_DISABLE;    // 
  // set DSRDTR
  dcb.fOutxDsrFlow = FALSE;                    // turn on DSR flow control
  //dcb.fDtrControl = DTR_CONTROL_ENABLE;    // 
  dcb.fDtrControl = DTR_CONTROL_DISABLE;    // 
  //dcb.fDtrControl = DTR_CONTROL_HANDSHAKE;    // 
  bPortReady = SetCommState(hComm, &dcb);
  //printf("x3 PortReady=%d\r\n",bPortReady);
  // Communication timeouts are optional
  bPortReady = GetCommTimeouts (hComm, &CommTimeouts);
  //printf("x4 PortReady=%d\r\n",bPortReady);
  CommTimeouts.ReadIntervalTimeout = 5;
  CommTimeouts.ReadTotalTimeoutConstant = 5;
  CommTimeouts.ReadTotalTimeoutMultiplier = 1;
  CommTimeouts.WriteTotalTimeoutConstant = 5;
  CommTimeouts.WriteTotalTimeoutMultiplier = 1;

  bPortReady = SetCommTimeouts (hComm, &CommTimeouts);
  //printf("x5 PortReady=%d\r\n",bPortReady);
  #endif

  //On linux you need to open the tty port
  #if defined __linux__ || __APPLE__
  printf("Opening Com Port on Linux \n");
  hComm = open(ComPortName,  O_RDWR | O_NOCTTY);
  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  // Read in existing settings, and handle any error
  if(tcgetattr(hComm, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be whatever the baudRate variable is
  cfsetispeed(&tty, BaudRate);
  cfsetospeed(&tty, BaudRate);

  // Save tty settings, also checking for error
  if (tcsetattr(hComm, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  #endif

  return hComm;
}
 
 
char SerialGetc(HANDLE *hComm)
{
  char rxchar;

  #ifdef RPI
  if (serialDataAvail(*hComm)) {
      rxchar = serialGetchar(*hComm);
      //fflush(stdout);
    }
  return rxchar;
  #else
  
  #ifdef __WIN32__
    bool bReadRC;
    static DWORD iBytesRead;
    do {
      bReadRC = ReadFile(*hComm, &rxchar, 1, &iBytesRead, NULL);
    } while (iBytesRead==0);
    return rxchar;
  #endif
  #if defined __linux__ || __APPLE__
    // Allocate memory for read buffer, set size according to your needs
    memset(&rxchar, '\0', sizeof(rxchar));
    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    int num_bytes = read(*hComm, &rxchar, sizeof(rxchar));
    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes <= 0) {
      //printf("Error reading: %s", strerror(errno));
      rxchar = '\0';
    }
    // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
    // print it to the screen like this!)
    //printf("Read %i bytes. Received message: %s", num_bytes, read_buf);
    //printf("Read %i bytes, rxchar = %c, ASCII = %d ",num_bytes,rxchar,int(rxchar));
    return rxchar;
  #endif

  #endif

}
 
void SerialPutc(HANDLE *hComm, char txchar)
{
  #ifdef RPI
  serialPutchar(*hComm,txchar);
  fflush(stdout);
  return;
  #endif
  #ifdef __WIN32__
  BOOL bWriteRC;
  static DWORD iBytesWritten;
  bWriteRC = WriteFile(*hComm, &txchar, 1, &iBytesWritten,NULL);
  return;
  #endif
  #if defined __linux__ || __APPLE__
  // Write to serial port
  write(*hComm,&txchar,sizeof(txchar));
  return;
  #endif
}

void SerialPutString(HANDLE *hComm, char *string)
{
  char outchar;
  outchar = *string++;
  while (outchar!=NULL){
    SerialPutc(hComm,outchar);
    //printf("%c",outchar);
    outchar = *string++;
  }
}

void SerialSendArray(HANDLE *hComm,float number_array[],int num) {
  SerialSendArray(hComm,number_array,num,1);
}

void SerialSendArray(HANDLE *hComm,float number_array[],int num,int echo) {
  union inparser inputvar;
  char outline[20];
  for (int i = 0;i<num;i++) {
    inputvar.floatversion = number_array[i];
    int int_var = inputvar.inversion;
    if (echo) {
      printf("Sending = %lf %d \n",number_array[i],int_var);
    }
    sprintf(outline,"H:%08x ",int_var);
    if (echo) {
      printf("Hex = %s \n",outline);
    }
    SerialPutString(hComm,outline);
    //Send a slash r after every number
    SerialPutc(hComm,'\r');
  }
  if (echo) {
    printf("Numbers Sent \n");
  }
}

///////////This is really annoying but when this Serial library was first written, the desktop
////side would send 3 hex numbers at a time and then a \r at the end. The board would then
///respond with 1 hex number at a time with \r at the end. Because of that the SerialPutArray is
///for the desktop to send an array where 3 numbers are followed by \r
///the SerialSendArray is literally the exact same code but it sends 1 number at a time with \r
///at the end. In my opinion, it would be better to send 1 hex number and then \r back and forth
///that way there's no confusion on which routine to use. Problem is that MultiSAT++/HIL is using
//the 3 hex \r format and the RPI Groundstation is using 1 hex \r format. In an effort to not break
//other people's code I have kept SerialPutArray and SerialSendArray. If we can ever get the MultiSAT
//and HIL members in the room together and have a coding party I suggest we change everything to 1 hex \r
//format but for now we will leave this here. CMontalvo 10/13/2020 (This was a Tuesday. Not a Friday)

void SerialPutArray(HANDLE *hComm,float number_array[],int num) {
  SerialPutArray(hComm,number_array,num,1);
}

void SerialPutArray(HANDLE *hComm,float number_array[],int num,int echo) {
  union inparser inputvar;
  char outline[20];
  int slashr = 0;
  for (int i = 0;i<num;i++) {
    inputvar.floatversion = number_array[i];
    int int_var = inputvar.inversion;
    if (echo) {
      printf("Sending = %lf %d \n",number_array[i],int_var);
    }
    sprintf(outline,"H:%08x ",int_var);
    if (echo) {
      printf("Hex = %s \n",outline);
    }
    SerialPutString(hComm,outline);
    slashr++;
    //Send a slash r after every 3rd set of numbers
    if (slashr == 3) {
      SerialPutc(hComm,'\r');
      slashr=0;
    }
  }
  if (echo) {
    printf("Numbers Sent \n");
  }
}

//This function will just read everything from the Serial monitor and print it to screen
void SerialGetAll(HANDLE *hComm) {
  char inchar = '\0';
  printf("Waiting for characters \n");
  int i = 0;
  do {
    do {
      inchar = SerialGetc(hComm);
      //printf("i = %d inchar = %c chartoint = %d \n",i,inchar,int(inchar));
    } while (inchar == '\0');
    printf("Receiving: i = %d char = %c chartoint = %d \n",i,inchar,int(inchar));
    i++;
  } while ((i<MAXLINE));
  printf("Response received \n");
}

void SerialGetArray(HANDLE *hComm,float number_array[],int num) {
  SerialGetArray(hComm,number_array,num,1);
}

void SerialGetArray(HANDLE *hComm,float number_array[],int num,int echo) {
  union inparser inputvar;
  for (int d = 0;d<num;d++) {
    int i = 0;
    char inLine[MAXLINE];
    char inchar = '\0';
    if (echo) {
      printf("Waiting for characters \n");
    }
    do {
      do {
        inchar = SerialGetc(hComm);
      } while (inchar == '\0');
      if (echo) {
      printf("Receiving: i = %d char = %c chartoint = %d \n",i,inchar,int(inchar));
      }
      inLine[i++] = inchar;
    } while ((inchar != '\r') && (i<MAXLINE));
    if (echo) {
      printf("Response received \n");
    }

    // Format from Arduino:
    // H:nnnnnnnn 

    // Now Convert from ASCII to HEXSTRING to FLOAT
    if (echo) {
      printf("Converting to Float \n");
    }
    inputvar.inversion = 0;
    for(i=2;i<10;i++){
      if (echo) {
        printf("Hex Digit: i = %d char = %c \n",i,inLine[i]);
      }
      inputvar.inversion <<= 4;
      inputvar.inversion |= (inLine[i] <= '9' ? inLine[i] - '0' : toupper(inLine[i]) - 'A' + 10);
    }
    if (echo) {
      printf("Integer Received = %d \n",inputvar.inversion);
      printf(" \n");
    }
    number_array[d] = inputvar.floatversion;
  }
}

void SerialPutHello(HANDLE *hComm,int echo) {
  if (echo) {
    printf("Sending w slash r \n");
  }
  SerialPutc(hComm,'w');
  SerialPutc(hComm,'\r');
  if (echo) {
    printf("Sent \n");
  }
}

int SerialGetHello(HANDLE *hComm,int echo) {
  //Consume w\r\n
  if (echo) {
    printf("Reading the Serial Buffer for w slash r slash n \n");
  }
  char inchar;
  int err = 0;
  for (int i = 0;i<3;i++) {
    inchar = SerialGetc(hComm);
    int val = int(inchar);
    err+=val;
    if (echo) {
      printf("%d \n",val);
    }
  }
  return err;
}

int SerialListen(HANDLE *hComm) {
  return SerialListen(hComm,1); //default to having echo on
}

int SerialListen(HANDLE *hComm,int echo) {
  //Listen implies that this is a drone/UAV/robot that is simply
  //listening on the airwaves for anyone sending out w \r
  //Listen w\r
  
  ///////////////THIS WORKS DO NOT TOUCH (RPI ONLY)
  /* char dat;
  if(serialDataAvail(*hComm))
    {
      dat = serialGetchar(*hComm);
      printf("char = %c int(char) = %d \n", dat,int(dat));
    }
    return 0;*/
  ///////////////////////////////////////

  int ok = 0;
  char inchar;
  inchar = SerialGetc(hComm);
  int val = int(inchar);
  if ((echo) && (val > 0) && (val < 255)) {
    printf("SerialListen => char = %c int(char) = %d \n", inchar,val);
  }
  
  if (val == 119) { //That's a w!
    ok += 119;
    //If we received a w we need to read say 10 times and see if we get a \r
    //remember that \r is a 13 in ASCII and \n is 10 in ASCII
    inchar = SerialGetc(hComm);
    val = int(inchar);
    //If we received a 13 or reach max we will break out of this loop
    //There is nothing more we need to do so we will just print val
    //to the screen
    if ((echo) && (val == 13)){
      printf("Slash R Received!!!! \n");
    }
    if (echo) {
      printf("Character Received = %c ASCII Code = %d \n",inchar,val);
    }
    //and then increment ok
    ok+=val;
  }
//either way we shall return ok
  return ok;
}

void SerialDebug(HANDLE *hComm) {
  char inchar;
  inchar = SerialGetc(hComm);
  int val = int(inchar);
  printf("Character Received = %c ASCII Code = %d \n",inchar,val);
}

void SerialRespond(HANDLE *hComm) {
  //overloaded function just calls the echo on version by default
  SerialRespond(hComm,1);
}

//Responding is very much like SerialPutHello except this is
//board side so this implies that a drone/uav/robot is responding
//to a groundstation computer saying hi.
//the response to hello (w\r) is hello, sir (w\r\n)
void SerialRespond(HANDLE *hComm,int echo) {

  /* THIS WORKS DO NOT TOUCH (RPI ONLY)
  char dat;
  dat = 'w';
  printf("Sending char %c \n",dat);
  serialPutchar(my, dat);
  dat = '\r';
  printf("Sending char %c \n",dat);
  serialPutchar(my, dat);
  dat = '\n';
  printf("Sending char %c \n",dat);
  serialPutchar(my, dat);
  */
  
  if (echo) {
    printf("Sending w slash r slash n \n");
  }
  SerialPutc(hComm,'w');
  SerialPutc(hComm,'\r');
  SerialPutc(hComm,'\n');
  if (echo) {
    printf("Sent \n");
  }
}

  
  
