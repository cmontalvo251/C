#include "magnetorquers.h"

////////MAGNETORQUERS/////////////////
//Constructor Class
Magnetorquers::Magnetorquers(char MAGNETOMETERFILENAME[]) {

  ifstream magfile;
  double floatvalue;
  string input;
  magfile.open(MAGNETOMETERFILENAME);
  if (magfile.is_open()) {

    //Number of magnetorquers
    getline(magfile,input);
    NUMMAGTORQUERS = atoi(input.c_str());

    ///Turns in magnetorquers
    getline(magfile,input);
    floatvalue = atof(input.c_str());
    magnnumturn = floatvalue;

    //Assuming a 10 cm x 20 cm magnetoruer wrapped around each solar panel
    //areamagnetorque = (10.0/100.0)*(20.0/100.0); //area of each magnetorque, m^2
    getline(magfile,input);
    floatvalue = atof(input.c_str());
    areamagnetorque = floatvalue;

    //Maximum Current
    getline(magfile,input);
    floatvalue = atof(input.c_str());
    maxcurrent = floatvalue;

    magfile.close();
  }
  else {
    printf("File: %s not found \n",MAGNETOMETERFILENAME);
    exit(1);
  }
  LMN.zeros(3,1,"Magnetorquer Torque (Magnetorquer Class)");
  currents.zeros(3,1,"Magnetorquer Current Values");
  desired_magmoments.zeros(3,1,"Mag Moment Desired from Controller");
  MMTVEC.zeros(3,1,"Magnetorquer Moment");
  BVEC_Tesla_OLD.zeros(3,1,"Magnetic Field in Tesla's (OLD)");
  BVEC_Tesla.zeros(3,1,"Magnetic Field in Tesla's");
  BVEC_.zeros(3,1,"Magnetic Field in Body Frame");
}

void Magnetorquers::compute_moments() {
  //You can add dynamics in here if you want but for now this is just
  //the total moments from the torquers

  //The eqaution for torque is 
  //LMN_Magnet_Torquers = mu x magnetic field

  //So first grab the magnetic field
  //This is done in the UpdateMagFieldParameters routine in Satellite
  //Physics. It's done there because the magnetic field is an engine variable

  //Then you need to compute mu
  //Unfortunately the desired_magmoments could be over the threshold set by the
  //circuitry so we need to check that
  currents.overwrite(desired_magmoments);
  currents.mult_eq(1.0/(magnnumturn*areamagnetorque));
  double norm = fabs(currents.get(1,1)) + fabs(currents.get(2,1)) + fabs(currents.get(3,1));
  if (norm > maxcurrent) {
    currents.mult_eq(maxcurrent/norm);
  }

  //with the saturation block set we can compute the actual magnetic moment
  MMTVEC.overwrite(currents);
  MMTVEC.mult_eq(magnnumturn*areamagnetorque);
  
  //Once you have the magnetic moment and magnetic field you can compute the total
  //torque placed on the satellite
  LMN.cross(MMTVEC, BVEC_Tesla);

}

double Magnetorquers::getCurrent(int row) {
  return currents.get(row,1);
}

void Magnetorquers::UpdateMagneticField(MATLAB BVECin) {
  //Put Tesla_OLD here for finite difference calcs
  BVEC_Tesla_OLD.overwrite(BVEC_Tesla);
  
  //Takes the magnetic field in the body frame and gives it to the
  //magtorquer class
  BVEC_.overwrite(BVECin);

  //Convert Magnetic field to Teslas rather than nanoTeslas
  BVEC_Tesla.mult(BVEC_,1e-9);
}

