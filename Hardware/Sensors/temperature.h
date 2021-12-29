#ifndef TEMP_H
#define TEMP_H

#include <iostream>
#include <fstream>
#include <string> //only for g++
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

using namespace std;

class temperature {
private:
public:
	//constructor
	temperature();
	void get();
	double temp = 0.0;
};


#endif