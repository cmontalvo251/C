#include "BarometerTemperature.h"

///////////////Barometer Threading Setup////////////////
//This works, and the barometer data is being recorded!!
void* acquireMS5611Data(void * thread_IN){
  BarometerTemperature* MS5611_thread = (BarometerTemperature*)thread_IN;

  float Normalized_Pressure;

  printf("Polling the barometer in the thread with an infinite while loop \n");

  while(1){

    MS5611_thread->refreshP();
    MS5611_thread->wait();
    MS5611_thread->readP();
    
    MS5611_thread->refreshT();
    MS5611_thread->wait();
    MS5611_thread->readT();

    MS5611_thread->process();

    sleep(0.5); //SLOW DOWN THIS THREAD!!!
  }

  pthread_exit(NULL);
  
}

void BarometerTemperature::wait() {
  usleep(10000); //Waiting for pressure and temperature data to be ready
}

void BarometerTemperature::refresh() {
  refreshP();
  refreshT();
}

void BarometerTemperature::refreshP() {
  sensor.refreshPressure();
}

void BarometerTemperature::refreshT() {
  sensor.refreshTemperature();
}

void BarometerTemperature::read() {
  readP();
  readT();
}

void BarometerTemperature::readP() {
  sensor.readPressure();
}

void BarometerTemperature::readT() {
  sensor.readTemperature();
}

void BarometerTemperature::process() {

  sensor.calculatePressureAndTemperature();
  Pressure = sensor.getPressure();
  //printf("Pressure(thread) = %lf ",Pressure);
  
  Temperature = sensor.getTemperature();
  //printf("Temperature(thread) = %lf \n",baro_thread->baroTemperature);

  float Normalized_Pressure = P_avg/Pressure;
  //printf("Normalized_Pressure = %f, ",Normalized_Pressure);
    
  Altitude = ((pow(Normalized_Pressure,0.19022)-1.0)*T_avg)/0.0065;
  //printf("Altitude = %f \n",Altitude);
}

void BarometerTemperature::ComputeAveragePT(){

  float TempSum = 0.0;
  float BaroSum = 0.0;
  printf("%s %d %s \n","Averaging out Pressure and Temperature this will take",NREADINGS,"seconds \n");
  for (int i = 0;i<NREADINGS;i++) {
    if (THREAD) {
      sleep(1);
    } else {
      refreshP();
      wait();
      readP();
      refreshT();
      wait();
      readT();
      process();
    }
    TempSum += (Temperature + 273.15); //These variables are set inside a threading routine
    BaroSum += Pressure;
    printf("Pressure Temperature (calibration) = %lf %lf \n",Pressure,Temperature);
    printf("Pressure Temperature (SUM) = %lf %lf \n",BaroSum,TempSum);
    sleep(1);
  }
  T_avg = TempSum/(double)NREADINGS;
  P_avg = BaroSum/(double)NREADINGS;

  //Refresh one more time to be ready for reading
  refreshP();

  usleep(10000);

  printf("T_avg = %f, ",T_avg);
  printf("P_avg = %f \n",P_avg);
  
}


void setupMS5611(BarometerTemperature* barotemp) {

  printf("Initializing MS5611 \n");
  barotemp->sensor.initialize();
  printf("MS5611 Initialized \n");

  //Only kick off a thread if you need to
  if (barotemp->THREAD) {
    printf("Creating Thread for Pressure and Temperature \n");
    if(pthread_create(&barotemp->sensor_thread, NULL, acquireMS5611Data, (void*)&barotemp)){
    printf("Error: Failed to create barometer thread\n");
    }
    printf("Created Barometer Thread \n");
    sleep(1);
  } else {
    barotemp->refreshP();
    barotemp->wait();
    barotemp->readP();
    barotemp->refreshT();
    barotemp->wait();
    barotemp->readT();
    barotemp->process();
  }

  barotemp->ComputeAveragePT(); //This only averages temperature and pressure

  ///////////////////////////////////////////////
  
}
