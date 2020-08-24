#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>

int main()
{
int serial_port;
char dat;
if ((serial_port = serialOpen ("/dev/ttyAMA0",57600)) < 0) /*open serial port*/
{
fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
return 1;
}

if(wiringPiSetup () == -1)
{
fprintf(stdout, "Unable to start wiringPi: %s\n", strerror (errno));
return 1;
}

while(1)
{
if(serialDataAvail(serial_port))
{
dat = serialGetchar(serial_port);
printf("%c", dat);
fflush(stdout);
serialPutchar(serial_port, dat);
}
}
}
