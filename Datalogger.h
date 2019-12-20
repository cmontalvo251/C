#ifndef DATALOGGER_H
#define DATALOGGER_H

#include "MATLAB.h"

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
  //Contructor
  Datalogger();
};





#endif


