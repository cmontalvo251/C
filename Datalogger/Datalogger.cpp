#include "Datalogger.h"

//Constructor
Datalogger::Datalogger() {
}

void Datalogger::findfile(char* directory) {
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
    number+=1; //Number is global in header file
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

void Datalogger::setLogVars(int num) {
  logvars.zeros(num,1,"Vars to Log");
  logheader = (char**)malloc(num*sizeof(char*));
  length = num;
}

void Datalogger::printheaders() {
  for (int i = 0;i<length;i++) {
    fprintf(outfile,"%s ",logheader[i]);
    //printf("%s ",logheader[i]);
    if (i < length - 1) {
      fprintf(outfile,"%s",",");
    }
  }
  fprintf(outfile,"%s ","\n");
  IsHeader = 1;
}

//Print function
void Datalogger::print(MATLAB out) {
  out.vecfprintf(outfile);
}

void Datalogger::print() {
  logvars.vecfprintf(outfile);
}

void Datalogger::println(MATLAB out) {
  out.vecfprintfln(outfile);
}

void Datalogger::println() {
  logvars.vecfprintfln(outfile);
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

int Datalogger::ImportFile(char* filename,MATLAB* data,char* name,int length) {
  //cout << "Reading " << filename << endl;
  //Gonna open this file the old school way
  FILE *file;
  file = fopen(filename,"r");
  char dummy[256];
  if (file) {
    if (length == -99) {
      length = 0;
      while (!feof(file)) {
	fgets(dummy,256,file);
	//cout << dummy << endl;
	length++;
      }
      fclose(file);
    }
    //cout << "File length is " << length << endl;
    //Now that we know how big the file is we need to make a MATLAB vector the same size
    data->zeros(length,1,name); //Because we passed a pointer we have to use the -> instead of .		//data.disp();
    //Then we open the file with FSTREAM and throw it in there. Done. Boom.
    fstream datafile;
    datafile.open(filename);
    if (datafile.is_open()) {
      string input;
      for (int idx = 0;idx<length;idx++) {
	      getline(datafile,input);
      	data->set(idx+1,1,atof(input.c_str()));
      }
      //For debug purposes let's make sure we read everything correctly. File I/O in C++
      //is always hit or miss for me.
      //data->disp();
    } else {
      cout << "Something went wrong in FSTREAM. Maybe the file wasn't closed properly?" << endl;
      return 0;
    }
  } else {
    cout << "File not found = " << filename << endl;
    return 0;
  }
  //This code will automatically generate a MATLAB vector based on how many rows are in
  //the file. Probably need to do an FSEEK or something and then do a ZEROS call to a MATLAB
  //Array.
  //If everything checks out we just return 1
  return 1;
}

