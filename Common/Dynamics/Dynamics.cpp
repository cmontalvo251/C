#include "Dynamics.h"

//Constructor
Dynamics::Dynamics() {
  NUMSTATES = 13; //Using Quaternions
  //Always create these just because I don't want to think about
  //when we actually need them and it's really easy to create them
  cg.zeros(3,1,"Center of Mass");
  ptp.zeros(3,1,"Roll Pitch Yaw");
  FTOTALI.zeros(3,1,"Total Forces Inertial Frame");

  //Six DoF Model is always running
  q0123.zeros(4,1,"Quaternions");
  cgdotI.zeros(3,1,"Velocity Inertial");
  cgdotB.zeros(3,1,"Velocity Body Frame");
  ptpdot.zeros(3,1,"Euler Derivatives");
  pqr.zeros(3,1,"Angular Velocity Body Frame");
  pqrdot.zeros(3,1,"Derivative of Angular Velocity");
  uvwdot.zeros(3,1,"Derivaitves of Velocity Body Frame");
  //Initialize Forces and Moments
  FGNDB.zeros(3,1,"Ground Forces Body Frame");
  FTOTALB.zeros(3,1,"Total Forces Body Frame");
  MTOTALI.zeros(3,1,"Total Moments Inertial Frame");
  MTOTALB.zeros(3,1,"Total Moments Body Frame");
  MGNDB.zeros(3,1,"Ground Moments Body Frame");
  I_pqr.zeros(3,1,"I times pqr");
  pqrskew_I_pqr.zeros(3,1,"pqr cross I times pqr");
  Kuvw_pqr.zeros(3,1,"uvw cross pqr");

  //////////////////////////TELEMETRY SETUP/////////////////////////////
  #ifdef TELEMETRY
  printf("Opening Serial Port ttyAMA0 \n");
  serial.SerialInit("/dev/ttyAMA0",57600);
  printf("If no errors present, serial port is open \n");
  #endif
  //////////////////////////////////////////////////////////////////////

  
}
  
void Dynamics::setRates(double RCRATE,double CTLRATE,double TELEMRATE) {
  tRC = RCRATE;
  tCTL = CTLRATE;
  #ifdef TELEMETRY
  serial.period = TELEMRATE;
  #endif
}

void Dynamics::setState(MATLAB state_in,MATLAB statedot_in) {
  //next 4 states are the quaternions
  q0123.vecset(1,4,state_in,4);
  double norm = q0123.norm();
  q0123.mult_eq(1/norm);
  state_in.vecset(4,7,q0123,1);
  //Also set the entire state vector
  state.overwrite(state_in);
  statedot.overwrite(statedot_in);
  ///First 3 states are inertial position
  cg.vecset(1,3,state,1);
  //We need to convert the quaternions to ptp
  ptp.quat2euler(q0123);
  //next 3 states are the xbody velocities
  cgdotB.vecset(1,3,state,8);
  //The next 3 states are the pqr angular velocities
  pqr.vecset(1,3,state,11);
  //The next N states are the actuators
  if (NUMACTUATORS > 0) {
    actuatorState.vecset(1,NUMACTUATORS,state,14);
    actuatorStatedot.vecset(1,NUMACTUATORS,statedot,14);
  }
}
void Dynamics::setMassProps(MATLAB massdata) {
  m = massdata.get(1,1);
  I.zeros(3,3,"Inertia");
  I.set(1,1,massdata.get(2,1));
  I.set(2,2,massdata.get(3,1));
  I.set(3,3,massdata.get(4,1));
  Iinv.overwrite(I,"Iinv");
  Iinv.inverse();
  env.setMass(m);
}

void Dynamics::initExtForces(int F) {
  //If the AERO Model is on we initialize the aero model
  if (F) {
    MATLAB var;
    var.zeros(2,1,"external force vars");
    var.set(1,1,F); //Sending Forces Flag to this var 
    extforces.setup(var);
  }
}

void Dynamics::initStateVector() {
  state.zeros(NUMVARS,1,"Vehicle state vector (also includes actuators");
  statedot.zeros(NUMVARS,1,"Vehicle statedot vector (also includes actuators");
}

//?overloaded function to give us default actuators
void Dynamics::initActuators(int NUMSIGNALS) {
  //Initialize to default of whatever the same number of control signals
  actuatorError.zeros(NUMSIGNALS,1,"actuator_error");
  NUMVARS = NUMSTATES;
  initStateVector();
}

void Dynamics::initActuators(MATLAB actuatordata) {
  //this variable sets the amount of actuator error that is included in the sim
  ACTUATOR_ERROR_PERCENT = actuatordata.get(1,1); 
  //Number of Actuators
  NUMACTUATORS = actuatordata.get(2,1);
  if (NUMACTUATORS > 0){
    if (NUMACTUATORS != ctl.NUMSIGNALS) {
      printf("Number of Actuators needs to be the same as the Number of Control Signals \n");
      printf("Num Actuators = %d, Num Control Signals = %d \n",NUMACTUATORS,ctl.NUMSIGNALS);
      exit(1);      
    }
    ///Initialize some vectors
    actuatorTimeConstants.zeros(NUMACTUATORS,1,"actuatorTimeConstants");
    actuatorState.zeros(NUMACTUATORS,1,"actuatorState");
    actuatorStatedot.zeros(NUMACTUATORS,1,"actuatorStatedot");
    actuatorICs.zeros(NUMACTUATORS,1,"actuatorICs");
    /////Get Time Constants
    //actuatorTimeConstants.vecset(1,NUMACTUATORS,actuatordata,3);
    //This initial command above actually sets the settling time
    //Remember the settling time Ts = 4*tau so 
    //tau = Ts/4 but the time constant is 1/tau so 
    //timeConstant = 4/Ts
    for (int i = 1;i<=NUMACTUATORS;i++) {
      double val = 4.0/actuatordata.get(i+2,1);
      actuatorTimeConstants.set(i,1,val);
    }
    actuatorTimeConstants.disp();
    //Get initial conditions    
    actuatorICs.vecset(1,NUMACTUATORS,actuatordata,3+NUMACTUATORS);
    actuatorICs.disp();
    //to add errors to your control surfaces or thrusters. This variable is set in Simulation_Flags.txt
    actuatorError.zeros(NUMACTUATORS,1,"Actuator errors");
    actuatorErrorPercentage.zeros(NUMACTUATORS,1,"Actuator Percent Errors");
    for (int i = 0;i<NUMACTUATORS;i++) {
      actuatorErrorPercentage.set(i+1,1,(1+randnum(-1,1)*ACTUATOR_ERROR_PERCENT/100.0)); 
    }
    //PAUSE();
  }
  NUMVARS = NUMSTATES + NUMACTUATORS;
  initStateVector();
}

void Dynamics::initEnvironment(char ENVIRONMENTFILENAME[]) {
  //Initialize the Gravity model no matter what. 
  //this also imports the MultiSAT++ environment which contains
  //Sun and Earth Ephemeris as well as the magnetic field model
  env.init(ENVIRONMENTFILENAME);
}

void Dynamics::initController(int C){
  //Initialize control system model
  CONTROLLER_FLAG_INITIAL = C; //The ctl.loop routine can change the controller flag
  //in the ctl.loop so we need to save the initial control value in case we need
  //it for SIMONLY.
  if (C) {
    MATLAB var;
    var.zeros(5,1,"ctl vars");
    var.set(1,1,C);
    //Sending Mass props in case you need it for your control laws
    var.set(2,1,m);
    var.set(3,1,I.get(1,1));
    var.set(4,1,I.get(2,2));
    var.set(5,1,I.get(3,3));
    ctl.setup(var);
  }
}

void Dynamics::initErrModel(MATLAB sensordata) {
  err.initSensorErr(sensordata);
}

void Dynamics::rcio_init() {
  rcin.initialize();
  printf("Receiver Initialized \n");
  rcout.initialize(ctl.NUMSIGNALS);
  printf("PWM Outputs Initialized \n");
}

void Dynamics::loop(double t,double dt) {

  //printf("time = %lf tlastRCRead = %lf tRC = %lf \n",t,tlastRCread,tRC);
  
  ////////////////////Poll External Inputs////////////////////////////
  if (t > tlastRCread + tRC) {
    //printf("Reading RC State \n");
    rcin.readRCstate();
    //rcin.printRCstate(-4);    
    tlastRCread = t;
  }
  ////////////////////////////////////////////////////////////////////

  ////////////////SEND TELEMETRY DATA////////////////////////////////
  #ifdef TELEMETRY
  if ((t - serial.lastTime) > serial.period) {
    number_array[0] = err.errstate.get(4,1);
    number_array[1] = err.errstate.get(5,1);
    number_array[2] = err.errstate.get(6,1);
    number_array[3] = err.getLongitude();
    number_array[4] = err.getLatitude();
    number_array[5] = err.getAltitude();
    number_array[6] = t;
    serial.SerialSendArray(number_array,number_Telemetry_vars,1); //the trailing zero is to turn off echo
    serial.lastTime = t;
  }
  #endif
  //////////////////////////////////////////////////////////////////////


  ////////////////////////////////////////////////////////////////////////
  /*If you're running in SIMONLY. The Input_Files/Simulation_Flags.txt
  controls the CONTROLLER_FLAG. Since whomever is writing the controller.cpp
  has jurisdiction on what to do the best way is to simply set Aux1 to HIGH.
  In this way when you call the control loop you can turn the control
  system on or off if Aux1 is HIGH. It's also possible you're running
  in SIL mode and the joystick did not initialize
  */
  #if defined (SIMONLY) || (SIL)
  if ((CONTROLLER_FLAG_INITIAL == 1) && (rcin.joy_fd == -1)) {
    rcin.rxcomm[4] = STICK_MAX; //This automatically turns on the autopilot
    //rcin.rxcomm[0] = STICK_MID; //This will put throttle in the center
  }
  #endif
  //////////////////////////////////////////////////////////////////////

  ///////////////////////Call the Sensor Block////////////////////////
  #ifdef AUTO
  err.readSensors(t,dt); //this calls onboard sensors on the Navio
  #else
  //state.disp();
  err.readSensors(state,statedot,t); //this simulates sensors
  #endif
  ///////////////////////////////////////////////////////////////////
  
  //////////////////////////////////////////////////////////////////

  ////////////////////Call the Control loop////////////////////////
  ////////////////////Use the err state variables//////////////////
  if (t > tlastCTL + tCTL) {
    //printf("Running the controller \n");
    tlastCTL = t;
    ctl.loop(t,err.errstate,err.errstatedot,rcin.rxcomm);
  }
  /////////////////////////////////////////////////////////////////

  //////Send CtlComms to ESCs///////////////
  ///If you're running in SIL or SIMONLY this will just be a dummy function
  //I'm also assuming this happens as fast as possible
  for (int i = 0;i<rcout.NUMSIGNALS;i++){
    rcout.pwmcomms[i] = ctl.ctlcomms.get(i+1,1);    
  }
  rcout.write();
  //rcout.print();
  //printf("\n");
}

void Dynamics::printRC(int all) {
  rcin.printRCstate(all);  
}

void Dynamics::Derivatives(double t,MATLAB State,MATLAB k) {
  //The Derivatives are vehicle independent except for the 
  //forces and moments

  //Actuator Error Model is a simple first order filter
  if (NUMACTUATORS > 0) {
    ///Get the error actuator state
    double val = 0;
    for (int i = 0;i<NUMACTUATORS;i++) {  
      val = actuatorState.get(i+1,1)*actuatorErrorPercentage.get(i+1,1);
      actuatorError.set(i+1,1,val);  
    }
    //Integrate Actuator Dynamics
    //input will be ctlcomms and the output will be actuator_state
    for (int i = 0;i<NUMACTUATORS;i++) {
      k.set(i+14,1,actuatorTimeConstants.get(i+1,1)*(rcout.pwmcomms[i] - actuatorState.get(i+1,1)));
    }
  } else {
    //Otherwise just pass through the ctlcomms
    actuatorError.overwrite(ctl.ctlcomms);
  }

  ////////////////////KINEMATICS///////////////

  //Extract individual States from State Vector
  cgdotB.vecset(1,3,State,8);
  q0123.vecset(1,4,State,4);
  pqr.vecset(1,3,State,11);
  double q0 = q0123.get(1,1);
  double q1 = q0123.get(2,1);
  double q2 = q0123.get(3,1);
  double q3 = q0123.get(4,1);
  double p = pqr.get(1,1);
  double q = pqr.get(2,1);
  double r = pqr.get(3,1);
  //This is used to rotate things from body to inertial and vice versa
  ine2bod321.L321(q0123, 1);

  ///Linear Kinematics (xyzdot = TIB uvw)
  ine2bod321.rotateBody2Inertial(cgdotI,cgdotB);
  k.vecset(1,3,cgdotI,1);

  ///Rotational Kinematics (Quaternion Derivatives)
  k.set(4,1,(-p*q1-q*q2-r*q3)/2.0);
  k.set(5,1,(p*q0+r*q2-q*q3)/2.0);
  k.set(6,1,(q*q0-r*q1+p*q3)/2.0);
  k.set(7,1,(r*q0+q*q1-p*q2)/2.0);

  ////////////////FORCE AND MOMENT MODEL///////////////////////

  //External Forces Model
  //Send the external forces model the actuator_state instead of the ctlcomms
  extforces.ForceMoment(t,State,k,actuatorError);

  //Gravity Model and Magnetic Field model
  env.gravitymodel(State);
  env.groundcontactmodel(State,k);
  env.getCurrentMagnetic(t,State); 

  //Add Up Forces and Moments
  FTOTALI.overwrite(env.FGRAVI); //add gravity 

  //Rotate Forces to body frame
  ine2bod321.rotateInertial2Body(FGNDB,env.FGNDI);
  ine2bod321.rotateInertial2Body(FTOTALB,FTOTALI);

  //Add External Forces and Moments
  //FTOTALB.disp();
  //env.FGRAVI.disp();
  FTOTALB.plus_eq(extforces.FB);
  FTOTALB.plus_eq(FGNDB);
  //extforces.FB.disp();
  //FGNDB.disp();
  //FTOTALB.disp();  
  //if (FTOTALB.get(1,1) > 0) {
  //  PAUSE();
  //}

  //Translational Dynamics
  Kuvw_pqr.cross(pqr,cgdotB);
  FTOTALB.mult_eq(1.0/m); 
  uvwdot.minus(FTOTALB,Kuvw_pqr); 
  k.vecset(8,10,uvwdot,1);

  //Rotate Ground Contact Friction to Body
  //env.MGNDI.disp();
  ine2bod321.rotateInertial2Body(MGNDB,env.MGNDI);
  //MGNDB.disp();
  //PAUSE();

  //Moments vector
  MTOTALB.overwrite(extforces.MB);
  MTOTALB.plus_eq(MGNDB);

  ///Rotational Dynamics
  //pqrskew = [0 -r q;r 0 -p;-q p 0];  
  //pqrdot = Iinv*(LMN-pqrskew*I*pqr);
  //pqr.disp();
  I_pqr.mult(I,pqr);
  //I.disp();
  //I_pqr.disp();
  pqrskew_I_pqr.cross(pqr,I_pqr);
  //pqrskew_I_pqr.disp();
  MTOTALB.minus_eq(pqrskew_I_pqr);
  //LMN.disp();
  //Iinv.disp();
  pqrdot.mult(Iinv,MTOTALB);
  
  //Save pqrdot
  k.vecset(11,13,pqrdot,1);

  ////////////////////////////////////////////////////////////////
}

