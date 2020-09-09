#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>
#include <iostream>

// Flow control flags
#define FC_DTRDSR       0x01
#define FC_RTSCTS       0x02
#define FC_XONXOFF      0x04
 
// variables used with the com port
#ifdef __WIN32__
//Windows needs some extra stuff
#include <conio.h>
#include <windows.h>
DCB dcb;
COMMTIMEOUTS CommTimeouts;
DWORD iBytesWritten;
DWORD iBytesRead;
bool bPortReady;
bool bWriteRC;
bool bReadRC;
#endif
//Linux just needs to open a file to write to Serial
#ifdef __linux__
//So this is kind of a hack but basically I change HANDLE to FILE so that 
//I can use the exact same function declarations for Linux
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#define HANDLE int //this is my hack that I've done so that I can use windows commands on linux
#endif

//Shared my handle for linux/windows
extern HANDLE my;

//This is needed to convert from floats to longs
union inparser {
	long inversion;
	float floatversion;
} ;

#define MAXFLOATS 10
#define MAXLINE 60

//Serial Functions
void InitSerialPort(void); //Use this if you want to use defaults
HANDLE SerialInit(char *ComPortName, int BaudRate); //Use this if you want to specify
char SerialGetc(HANDLE *hComm);
void SerialPutc(HANDLE *hComm, char txchar);
void SerialPutString(HANDLE *hComm, char *string);
void SerialPutArray(HANDLE *hComm,float array[],int num);
void SerialGetArray(HANDLE *hComm,float array[],int num);

#endif
