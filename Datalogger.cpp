#include "Datalogger.h"

//Constructor
Datalogger::Datalogger() {
}

void Datalogger::findfile(char* directory) {
  //int number = 0;
  int found = 0;
  //char filename[256];
  FILE* fileout;
  while (!found) {
    sprintf(filename,"%s%d%s",directory,number,".txt");
    printf("%s%s \n","Attempting to check for file: ",filename);
    fileout = fopen(filename,"r");
    if (!fileout) {
      found = 1;
    } else {
      fclose(fileout);
      printf("File exists. Skipping \n");
    }
    number+=1;
  }
  printf("File found for writing = %s \n",filename);
  //return number
}

void Datalogger::open() {
  printf("Attempting to open: %s \n",filename);
  outfile = fopen(filename,"wb");
  //printf("Attempting to open %s \n",filename);
  if (!outfile) {
     printf("File not opened properly = %s \n",filename);
   } else {
     printf("File %s opened successfully \n",filename);
   }
}

//Print function
void Datalogger::print(MATLAB out) {
  out.vecfprintf(outfile);
}

void Datalogger::println(MATLAB out) {
  out.vecfprintfln(outfile);
}

//Print char*
void Datalogger::printchar(char* msg) {
  fprintf(outfile,"%s ",msg);
}

//Close function
void Datalogger::close() {
  printf("Closing File \n");
  fclose(outfile);
  printf("File closed \n");
}

//Flush function
void Datalogger::flush() {
  fflush(outfile);
}
