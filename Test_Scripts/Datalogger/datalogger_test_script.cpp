#include <iostream> //these are standard includes
#include <stdlib.h>
using namespace std;

//Include the datalogger class
#include "Datalogger.h"
Datalogger logger; //Creating a variable called Datalogger. just like int or double I can do Datalogger

//Get a Timer
#include "timer.h"
//time and clock are reserved variables so I used watch
TIMER watch;

//THE MAIN FUNCTION THAT this cpp file will run
//main has to return an integer
//int is an integer - 2 bytes 
//double is a double precision floating point number - 8 bytes
//float is a single precision floating point number - 4 bytes
//char is a single character - 2 bytes
//char* is a vector of characters which is basically a string like 'c' is a char
//but 'carlos' is a char*
//char** it's a list of char* so you can do 'carlos','collin'
// argc is the number of input arguments
// argv is the input arguments
int main(int argc,char** argv) {
	printf("Running Datalogger Test Script \n");

	//Looked for the FIle
	printf("Looking for File in %s \n",argv[1]);
	logger.findfile(argv[1]);

	//Then we open it
	logger.open();

	//Let's make a MATLAB variable
	MATLAB outdata;
	outdata.zeros(2,1,"outdata");

	double val = 0.999999;

	//We create a loop to write stuff
	watch.resetStartTime();
	for (int i = 0;i<10;i++){
		printf("Time = %lf \n",watch.getTimeSinceStart());

		//Populate the outdata Matrix
		outdata.set(1,1,watch.getTimeSinceStart());
		outdata.set(2,1,val);
		logger.println(outdata);

		//Change Val
		val = pow(val,2.0);

		cross_sleep(.1);
	}
	///The int at the top means this function will return an integer
	return 0;
}