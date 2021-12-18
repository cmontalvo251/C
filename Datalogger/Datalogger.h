#ifndef DATALOGGER_H //HEADER GUARD - It makes sure that you only import this header file once
#define DATALOGGER_H

#include <MATLAB/MATLAB.h>
#include <iostream>

class Datalogger {
 private:
  FILE* outfile;
  int number = 0;
  int length = 0;
  char filename[256];
 public:
  void findfile(char* directory);
  void print(MATLAB);
  void print();
  void println(MATLAB);
  void println();
  void printchar(char*);
  void close();
  void flush();
  void open();
  void printheaders();
  int ImportFile(char* filename,MATLAB* data,char* name,int length);
  //Contructor
  Datalogger();
  int IsHeader = 0;
  int filetype = 0;
  MATLAB logvars;
  char** logheader;
  void setLogVars(int);
};





#endif


