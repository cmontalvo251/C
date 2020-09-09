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

	//Create fictitious float
	float number = 5.0;
	float number_array[MAXFLOATS];
	number_array[0] = number;
	int number_of_numbers = 1;

	//Send to Arduino
	SerialPutArray(&my,number_array,number_of_numbers);

	//Now Read from Arduino
	SerialGetArray(&my,number_array,number_of_numbers);

	//Extract Data
	//float rec_number = number_array[0];

	//printf("Number Received = %lf \n",rec_number);

} //end main loop desktop computer
