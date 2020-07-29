#include "Serial.h"

HANDLE my;
 
HANDLE SerialInit(char *ComPortName, int BaudRate) 
{
  HANDLE hComm; 
  
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
  #ifdef __linux__
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

  // Set in/out baud rate to be 9600
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
  #ifdef __WIN32__
    bool bReadRC;
    static DWORD iBytesRead;
    do {
      bReadRC = ReadFile(*hComm, &rxchar, 1, &iBytesRead, NULL);
    } while (iBytesRead==0);
  #endif
  #ifdef __linux__
    // Allocate memory for read buffer, set size according to your needs
    memset(&rxchar, '\0', sizeof(rxchar));
    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    int num_bytes = read(*hComm, &rxchar, sizeof(rxchar));
    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
    }
    // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
    // print it to the screen like this!)
    //printf("Read %i bytes. Received message: %s", num_bytes, read_buf);
    #endif
  return rxchar;
}
 
void SerialPutc(HANDLE *hComm, char txchar)
{
  #ifdef __WIN32__
  BOOL bWriteRC;
  static DWORD iBytesWritten;
  bWriteRC = WriteFile(*hComm, &txchar, 1, &iBytesWritten,NULL);
  #endif
  #ifdef __linux__
  // Write to serial port
  //write(*hComm,txchar, sizeof(txchar));
  unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
  write(*hComm,"H",sizeof(msg));
  #endif
  return;
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

void InitSerialPort(void)
{
  // VERY IMPORTANT: Edit this line of code to designate which COM port the ADCS board is using!!
  int baudRate = 115200;
  #ifdef __WIN32__
    char *port = "\\\\.\\COM12";
  #endif
  #ifdef __linux__
    char *port = "/dev/ttyUSB0";
  #endif
  my=SerialInit(port,baudRate);
}
