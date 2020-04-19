#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "mathp.h"
using namespace std;

int main(int argc,char* argv[])
{
  double input = atof(argv[1]);
  cout << "Input = " << input << endl;
  double output = sat(input,0.1,1.0);
  cout << "Ouput = " << output << endl;
}
