//////////////////////COMMON SHARED FILE///////////////////////////////////
#include <stdlib.h>
#include <iostream>
#include "Serial.h"

////////////////////////DESKTOP COMPUTER///////////////////////////////////
int main() {
	//Initialize Serial Port
	//Make sure Arduino is on this port and 
	//using this baudrate
	my = SerialInit("/dev/ttyACM0",115200); 

	//Send w\r to Arduino
	printf("Sending w slash r \n");
	SerialPutc(&my,'w');
	SerialPutc(&my,'\r');
	printf("Sent \n");

	//Consume w\r\n
	printf("Reading the Serial Buffer for w slash r slash n \n");
	char inchar;
	for (int i = 0;i<3;i++) {
	  inchar = SerialGetc(&my);
	  printf("%d \n",int(inchar));
	}
} //end main loop desktop computer
