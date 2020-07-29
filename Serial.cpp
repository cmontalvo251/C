#include "Serial.h"
#include <conio.h>
#include <stdio.h>
#include <windows.h>
 
HANDLE SerialInit(char *ComPortName, int BaudRate) 
{
  HANDLE hComm;
    
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
 
    return hComm;
}
 
 
char SerialGetc(HANDLE *hComm)
{
    char rxchar;
    BOOL    bReadRC;
    static    DWORD    iBytesRead;
    do {
      bReadRC = ReadFile(*hComm, &rxchar, 1, &iBytesRead, NULL);
    } while (iBytesRead==0);
    return rxchar;
}
 
void SerialPutc(HANDLE *hComm, char txchar)
{
  BOOL    bWriteRC;
  static    DWORD    iBytesWritten;
  
  bWriteRC = WriteFile(*hComm, &txchar, 1, &iBytesWritten,NULL);
  
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
  my=SerialInit("\\\\.\\COM12",115200);
}
