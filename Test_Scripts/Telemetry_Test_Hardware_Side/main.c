//////////////////////COMMON SHARED FILE///////////////////////////////////
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include "Serial.h"

////////////////////////DESKTOP COMPUTER///////////////////////////////////
int main() {
  //Initialize Serial Port
  //using this baudrate
  printf("Initializing the dev Port \n");
  my = SerialInit("/dev/ttyAMA0",57600);  //This is the UART port on the RPI
  if (my < 0) {
    printf("Error Opening Dev Port \n");
  }
  printf("Dev Port Initialized. If no errors present we are currently listening \n");
  //sleep(5); //--This was an arduino hack. I don't think we need it.

  //After we setup the while loop we need to create an infinite while loop
  //to emulate what an autopilot or robot routine would look like on an RPi

  while (1) {
    //From here we basically need to constantly read the serial port at least once
    //in the while loop and check for w\r from the computer so I'll need to write a
    //int ok = SerialListen(&my,1); //set to 0 to turn echo off, 1 = all echos on, 2 = only echo if you receive anything
    // w\r was received it means we need to responde

    ///DEBUGGING
    char inchar;
    int val;
    if (serialDataAvail(my)) {
      inchar = SerialGetc(&my);
      val = int(inchar);
      printf("Character Received = %c ASCII Code = %d \n",inchar,val);
    }
    
    //if (ok == (119+13)) { //119 is ASCII for w and 13 is ASCII for \r
    //printf("w slash r received!! \n");

      /*
      SerialRespond(&my,1);
      //Once we've responded we must send whomever is talking to us some data

      //Create fictitious floats
      float number1 = -3.6;
      float number2 = 4.5;
      float number_array[MAXFLOATS]; //MAXFLOATS is set to 10 in Serial.h right now
      number_array[0] = number1;
      number_array[1] = number2;
      int number_of_numbers = 2;
      //Send over Serial
      SerialPutArray(&my,number_array,number_of_numbers);
      */
    //}
  
    
    
  } //end robot while loop
} //end main loop robot
