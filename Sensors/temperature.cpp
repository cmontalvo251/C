#include "temperature.h"

#ifdef DESKTOP
#define NUM_LINES 4
#define NUM_CHARS 14
#else
#define NUM_LINES 1
#define NUM_CHARS 1
#endif

///Constructor
temperature::temperature() {

}

void temperature::get() {
	#ifdef DESKTOP
	system("rm file");
	system("sensors -u > file");
	//system("cat file");
    #else
	system("/opt/vc/bin/vcgencmd measure_temp > file");
	#endif

	//Read file to get contents
	fstream datafile;
	datafile.open("file");
	string input;
	if (datafile.is_open()) {
		for (int i = 0 ; i < NUM_LINES ; i++) {
			getline(datafile,input);
		}
		input.erase(0,NUM_CHARS);
		//printf("%s , %lf \n",input.c_str(),atof(input.c_str()));
		temp = atof(input.c_str());
	} else {
		temp = -99;
	}
}