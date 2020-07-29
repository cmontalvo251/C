#ifndef SERIAL_H
#define SERIAL_H

// Flow control flags
#define FC_DTRDSR       0x01
#define FC_RTSCTS       0x02
#define FC_XONXOFF      0x04
 
// variables used with the com <strong class="highlight">port[/b]
BOOL            bPortReady;
DCB                dcb;
COMMTIMEOUTS    CommTimeouts;
BOOL            bWriteRC;
BOOL            bReadRC;
DWORD            iBytesWritten;
DWORD            iBytesRead;

HANDLE SerialInit(char *ComPortName, int BaudRate);
char SerialGetc(HANDLE *hComm);
void SerialPutc(HANDLE *hComm, char txchar);
void SerialPutString(HANDLE *hComm, char *string);
HANDLE my;
void InitSerialPort(void);
#endif
