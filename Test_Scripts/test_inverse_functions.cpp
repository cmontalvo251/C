//Ok let's heavily comment this shit. First we need a bunch of built in libraries
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
using namespace std;

//If we were just doing basic multiplication and substraction we could just use
//those but if we want to do fancy matrix multiplication and inverse routines we
//need something better. I've done this already. It's called MATLAB++ (MATLAB for C++)
//the quotes are because it's in this folder. Make sure to include the cpp file
//as well when you code your stuff
#include "../MATLAB.h"
//^^All of the routines for matrix multiplication are in there.
//The problem is you need to compile MATLAB.cpp into a module using the -c objective

//So first
//g++ ../MATLAB.cpp -c -w
//this will create a .o or module file

//Then you need
//g++ MATLAB.o rk4_using_MATLAB++.cpp

//This will create an a.out file

int main() {
  //Let's make a NxN matrix but loop it. You'll notice that some inverse routines
  //below throw errors depending on the size
  int N = 6;
  MATLAB A;
  A.zeros(N,N,"NxN");
  for (int idx = 1;idx<=N;idx++){
    A.set(idx,idx,idx);
  }
  A.disp();
  //And invert it using inv
  MATLAB Ainv;
  Ainv.inv(A,"NxN inv");
  Ainv.disp();
  //Let's now invert it using the inverse function
  MATLAB Ainverse;
  Ainverse.zeros(N,N,"NxN inverse");
  Ainverse.overwrite(A);
  Ainverse.inverse();
  Ainverse.disp();
  //And finally do the same with the matrix_inverse routine
  MATLAB Amatrix_inverse;
  Amatrix_inverse.zeros(N,N,"NxN matrix_inverse");
  Amatrix_inverse.matrix_inverse(A,N);
  Amatrix_inverse.disp();
}
