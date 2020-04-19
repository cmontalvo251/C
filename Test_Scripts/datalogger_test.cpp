#include "../Datalogger.cpp"

int main () {
  printf("Hello World - Let's output some data \n");
  MATLAB varout;
  varout.zeros(5,1,"varout");
  varout.disp();
  Datalogger log("output.txt");
  for (int idx = 0;idx<10;idx++) {
    log.println(varout);
  }
  log.close();
}
