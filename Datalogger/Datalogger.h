#ifndef DATALOGGER_H
#define DATALOGGER_H

#include "MATLAB.h"
#include <iostream>

class Datalogger {
 private:
  FILE* outfile;
  int number = 0;
  char filename[256];
 public:
  void findfile(char* directory);
  void print(MATLAB);
  void println(MATLAB);
  void printchar(char*);
  void close();
  void flush();
  void open();
  int ImportFile(char* filename,MATLAB* data,char* name,int length);
  //Contructor
  Datalogger();
};





#endif


