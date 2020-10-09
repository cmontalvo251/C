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
#ifndef INPARSER_H
#define INPARSER_H
union inparser {
	long inversion;
	float floatversion;
};
#endif

#define MAXFLOATS 10
#define MAXLINE 120

//Serial Functions
void InitSerialPort(void); //Use this if you want to use defaults
HANDLE SerialInit(char *ComPortName, int BaudRate); //Use this if you want to specify
char SerialGetc(HANDLE *hComm);
void SerialPutc(HANDLE *hComm, char txchar);
void SerialPutString(HANDLE *hComm, char *string);

///THE FORMAR FOR SAYING HELLO AND RESPONDING AS WELL AS PUTTING 
//AN ARRAY AND READING AN ARRAY REALLY NEEDS TO BE THE SAME
//THERE IS NO REASON REALLY TO HAVE THEM BE DIFFERENT.
void SerialPutArray(HANDLE *hComm,float array[],int num);
void SerialPutArray(HANDLE *hComm,float array[],int num,int echo);
void SerialGetArray(HANDLE *hComm,float array[],int num);
void SerialGetArray(HANDLE *hComm,float array[],int num,int echo);
void SerialGetAll(HANDLE *hComm);
void SerialPutHello(HANDLE *hComm,int echo);
int SerialGetHello(HANDLE *hComm,int echo);
int IsAnyoneOutThere(HANDLE *hComm,int echo);
int SerialListen(HANDLE *hComm,int echo);
int SerialListen(HANDLE *hComm);
void SerialRespond(HANDLE *hComm,int echo);
void SerialRespond(HANDLE *hComm);
void SerialDebug(HANDLE *hComm);

#endif
