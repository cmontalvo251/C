#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "MATLAB.h"
#include "mathp.h"

class environment {
 private:
  int GRAVITY;
 public:
   MATLAB FGRAV;
   void init(int G);
   void gravitymodel();
   environment(); //constructor   
};

#endif
