#include "environment.h"

//When you invoke this command the computer is going to search for
//these models. If it can't find them it will throw an error like this
//Cannot open /usr/local/share/GeographicLib/gravity/egm2008.egm
//Note it's possible to send the code a different path so we will fix this by putting another
//#ifndef. If you're on windows you need to download the model otherwise
//we will simply tell the computer to look in our user specified directory
//Notice that in satellite_physics.h we created a pointer to grav and mag
//In order to initialize them we need to use the "new" function much like
//we use the new function in main.cpp when we create a new Satellite
// Gravity_Flag = Gravity_IN; -- These have been moved to gravity_magnetic.cpp
// Magnetic_Flag = Magnetic_IN;
//Note. Since these are variables, it would be much nicer to put these in the
//Satellite class rather than these massive global variables. We can let it slide
//for now since it works but just a thought.

//constructor
environment::environment() {
}

void environment::setMass(double m) {
  //printf("m = %lf \n",m);
  mass = m;
  //PAUSE();
}

void environment::gravitymodel(MATLAB State) {
  FGRAVI.mult_eq(0); //zero out gravity
  gSun.mult_eq(0); //zero out sun gravity as well

  double gx=0, gy=0, gz=0;
  double x = State.get(1, 1);
  double y = State.get(2, 1);
  double z = State.get(3, 1);
  double mu = -GSPACE*MEARTH;
  double muSun = -GSPACE*MSUN;
  double rSat = sqrt(x*x + y*y + z*z);

  //Use the egm2008 model
  if (Gravity_Flag == 1)
    {
      if (rSat > REARTH) { //Make sure you're outside the earth otherwise this routine will return a nan
        egm2008->W(x, y, z, gx, gy, gz);
      } else {
      	gx = 0;
        gy = 0;
        gz = 0;
      }
    }
  //Use the point mass model
  else if ((Gravity_Flag == 0) || (Gravity_Flag == 2)) //This is the point mass model here
    {
      if (rSat > REARTH) { //Make sure you're outside the earth
        	gx = (mu / pow(rSat, 3))*x;
        	gy = (mu / pow(rSat, 3))*y;
        	gz = (mu / pow(rSat, 3))*z;
      } else {
      	//I don't really like this. I'm just going to kill the program if you land inside the Earth. I mean, at that point the simulation is invalid.
      	//Why continue?
      	//This environment.cpp is now part of FAST.git which means
      	//there is a ground contact model now
      	//printf("Gravity model is on and running but a part of your spacecraft is inside the Earth.....sooooo....\n");
      	//printf("the simulation is going to get killed. Hope you are having an ok day. \n");
      	//printf("X,Y,Z = %lf %lf %lf \n",x,y,z);
      	//printf("rSat = %lf REARTH = %lf delx = %lf \n",rSat,REARTH,rSat-REARTH);
      	//exit(1);
      	gx = 0;
      	gy = 0;
      	gz = 0;
      }
      if (Gravity_Flag == 2) {
      	//Need to add Sun gravity
      	//First get the location of the satellite
      	for (int i = 1;i<=3;i++) {
      	  //plus(rSun2EarthToday,XYZ_current);
      	  rSun2Sat.set(i,1,rSun2EarthToday.get(i,1)+State.get(i,1));
      	}
      	//Then compute gSun
      	gSun.overwrite(rSun2EarthToday);
      	gSun.mult_eq(muSun/pow(rSun2Sat.norm(),3));
      }
    }
      else if (Gravity_Flag == -1) { //Gravity is off
        gx = 0;
        gy = 0;
        gz = 0;
      }
      else if (Gravity_Flag == 3) { //Constant Gravity
        gx = 0;
        gy = 0;
        gz = GEARTH;
  }

  //printf("GX,GY,GZ = %lf %lf %lf \n",gx,gy,gz);

  //Add Sun gravity
  FGRAVI.plus_eq(gSun);
  //Add Earth Gravity
  FGRAVI.plus_eq1(1,1,gx);
  FGRAVI.plus_eq1(2,1,gy);
  FGRAVI.plus_eq1(3,1,gz);
  //Multiply by Mass
  FGRAVI.mult_eq(mass);
  //FGRAVI.disp();
}

void environment::groundcontactmodel(MATLAB State,MATLAB k) {
  double x = State.get(1,1);
  double y = State.get(2,1);
  double z = State.get(3,1);
  double norm = sqrt(x*x + y*y + z*z);
  double xdot = k.get(1,1);
  double ydot = k.get(2,1);
  double zdot = k.get(3,1);
  double u = State.get(8,1);
  double v = State.get(9,1);
  double r = State.get(13,1);
  double N = mass*GRAVITYSI;

  //Check to see if we're inside Earth
  bool insideEarth = 0;
  //Flat Earth Model. Z is down so anything positive is under the surface
  if ((GRAVITY_FLAG == 1) & (z>0)) {
    insideEarth = 1;
  }
  //Globe Model
  if ((GRAVITY_FLAG == 2) & (norm<REARTH)) {
    insideEarth = 1;
  }
  if (insideEarth) {
    printf("INSIDE EARTH! \n");
    FGNDI.set(1,1,-N*GNDCOEFF*sat(xdot,0.1,1.0));
    FGNDI.set(2,1,-N*GNDCOEFF*sat(ydot,0.1,1.0));
    FGNDI.set(3,1,-z*GNDSTIFF-zdot*GNDDAMP);
    //if (abs(r)>0.01) {
    //  MGNDI.set(3,1,-0.0001*N*GNDSTIFF*sat(r,0.01,1.0));
    //} else {
    MGNDI.mult_eq(0);
    //} 
  } else {
    FGNDI.mult_eq(0);
    MGNDI.mult_eq(0);
  }
  //FGNDI.disp();
}

void environment::init(char ENVIRONMENTFILENAME[]) {
  FGRAVI.zeros(3,1,"FORCE OF GRAVITY INERTIAL");
  FGNDI.zeros(3,1,"Ground Forces Inertial Frame");
  MGNDI.zeros(3,1,"Ground Moments Inertial Frame");
  BVECINE.zeros(3,1,"Inertial Frame Vectors of Magnetic Field");
  BVECSPH.zeros(3,1,"Speherical Frame Vectors of Magnetic Field");
  BVECB_Tesla.zeros(3,1,"Environment Magnetic Field Body Frame (Tesla)");

  fstream envfile;
  string input;
  printf("Attempting to open file = %s \n",ENVIRONMENTFILENAME);

  envfile.open(ENVIRONMENTFILENAME);

  if (envfile.is_open()) {
    //Magnet and Gravity Model Stuff
    getline(envfile,input);
    Gravity_Flag = atoi(input.c_str());

    getline(envfile,input);
    Magnetic_Flag = atoi(input.c_str());
    
    //time_gravity_next = 1; //These have been removed because we have
    //to call the gravity model every timestep now
    getline(envfile,input);
    time_magnet_next = atof(input.c_str()); //This says only call the magnetic model every 1 second
    time_magnet = 0;
    //time_gravity = 0; //same with this one

    getline(envfile,input);
    julian_today = atof(input.c_str());

    double julian_2000 = 2451545;

    if (julian_today < 2451545) {
      //Must be a year
      yr = julian_today;
      julian_today = julian_2000 + (yr-2000)*365.25;
    } else {
      //Otherwise compute year based off julian day
      yr = int((julian_today - julian_2000)/365.25) + 2000;
    }

    printf("Julian Day = %lf \n",julian_today);
    printf("Year = %lf \n",yr);

    //Get Distances from Sun to Earth
    rSun2Earth2000.zeros(3,1,"XYZ of Earth from Sun on Jan 1,2000");
    rSun2EarthToday.zeros(3,1,"XYZ of Earth from Sun Today");

    //Compute the location of the earth on Jan 1 2000 using the Julian Day of that time
    EarthEphemeris(rSun2Earth2000,julian_2000);
    rSun2Earth2000.disp();
    //Now compute the location of Earth on the Julian Day supplied
    EarthEphemeris(rSun2EarthToday,julian_today);
    rSun2EarthToday.disp();

    //Now compute the location of the Earth today relative to the Earth on Jan 1st, 2000
    rEarth20002EarthToday.zeros(3,1,"XYZ of Earth Today from Earth 1/1/2000");
    rEarth20002EarthToday.minus(rSun2EarthToday,rSun2Earth2000);
    rEarth20002EarthToday.disp();

    rSun2Sat.zeros(3,1,"Sun 2 Satellite");
    gSun.zeros(3,1,"Sun Gravity Acceleration");

    getline(envfile,input);
    SOLARWINDMODEL = atoi(input.c_str());
    SOLAR_DIRECTION.zeros(3,1,"Direction of Solar Wind in Environment Model");
    XYZE.zeros(3,1,"X,Y,Z Coordinate for Solar Wind");

    if ((SOLARWINDMODEL == 0) || (SOLARWINDMODEL == 1)) {
      for (int idx = 0;idx<3;idx++) {
	      getline(envfile,input);
	      SOLAR_DIRECTION.set(idx+1,1,atof(input.c_str()));
      }
      if (SOLAR_DIRECTION.get(1,1) == -99) {
	      SOLAR_DIRECTION.overwrite(rEarth20002EarthToday);
      }
      //Do this just in case the user f*#!ked up
      SOLAR_DIRECTION.normalize();
      SOLAR_DIRECTION.disp();
    }
    if (SOLARWINDMODEL >= 2) {
      //Read the file name
      getline(envfile,input);
      //Find the Exclamation Point
      size_t found = input.find("!");
      //Remove everything after the !
      input = input.erase(found);
      //remove spaces
      input.erase(remove(input.begin(),input.end(), ' '),input.end());
      //Make the XFILENAME
      char XFILENAME[256];
      strcpy(XFILENAME,input.c_str());
      strcat(XFILENAME,"X.WIND");
      //Make the YFILENAME
      char YFILENAME[256];
      strcpy(YFILENAME,input.c_str());
      strcat(YFILENAME,"Y.WIND");
      //Make the ZFILENAME
      char ZFILENAME[256];
      strcpy(ZFILENAME,input.c_str());
      strcat(ZFILENAME,"Z.WIND");
      //Make the PARAM FILENAME
      char PARAMFILENAME[256];
      strcpy(PARAMFILENAME,input.c_str());
      strcat(PARAMFILENAME,"PARAM.WIND");
      printf("Reading Solar Wind Model Files from %s, %s, %s %s \n",XFILENAME,YFILENAME,ZFILENAME,PARAMFILENAME);

      //Read the PARAMFILENAME
      fstream paramfile;
      paramfile.open(PARAMFILENAME);
      if (paramfile.is_open()) {
	//From here things differentiate depending on which solar wind model is selected
	switch (SOLARWINDMODEL) {
	case 2: {
	  //If the SOLARWINDMODEL is 2 there is an extra line you need to read from the envfile
	  getline(envfile,input);
	  REFERENCEFRAME = atoi(input.c_str());
	  //Then proceed as before
	  cout << "Using Solar wind model 2D = f(x,y)" << endl;
	  if (REFERENCEFRAME) {
	    cout << "Placing Reference Frame at Confluence Point" << endl;
	  }
	  getline(paramfile,input);
	  X_SOLAR_MIN = atof(input.c_str());
	  getline(paramfile,input);
	  X_SOLAR_MAX = atof(input.c_str());
	  getline(paramfile,input);
	  Y_SOLAR_MIN = atof(input.c_str());
	  getline(paramfile,input);
	  Y_SOLAR_MAX = atof(input.c_str());
	  getline(paramfile,input);
	  GRIDX = atof(input.c_str());
	  getline(paramfile,input);
	  GRIDY = atof(input.c_str());
	  //Create X and Y vectors using linspace
	  XCOORD.linspace(X_SOLAR_MIN,X_SOLAR_MAX,GRIDX,"XCOORD");
	  YCOORD.linspace(Y_SOLAR_MIN,Y_SOLAR_MAX,GRIDY,"YCOORD");
	  XCOORD.disp();
	  YCOORD.disp();
	  //Import X,Y,Z solar direction vectors
	  SOLARX.zeros(GRIDX,GRIDY,"SOLARX");
	  SOLARX.dlmread(XFILENAME);
	  SOLARX.disp();
	
	  SOLARY.zeros(GRIDX,GRIDY,"SOLARY");
	  SOLARY.dlmread(YFILENAME);
	  SOLARY.disp();
	
	  SOLARZ.zeros(GRIDX,GRIDY,"SOLARZ");
	  SOLARZ.dlmread(ZFILENAME);
	  SOLARZ.disp();

	  //Try a test point
	  cout << "SOLAR X = " << SOLARX.interp2(XCOORD,YCOORD,0,0,1) << " at 0,0 " << endl;
	  cout << "SOLAR Y = " << SOLARY.interp2(XCOORD,YCOORD,0,0,1) << " at 0,0 " << endl;
	  cout << "SOLAR Z = " << SOLARZ.interp2(XCOORD,YCOORD,0,0,1) << " at 0,0 " << endl;
	  printf("Custom Solar Wind Environment Imported Successfully \n");
	  break; //END SOLAR WIND MODEL 2
	}
	case 3: {
	  cout << "Using Solar Wind Model 1D = f(t)" << endl;
	  //For 1D we need the param file which has the time coordinates
	  //and then X,Y and Z for the abcissa as a function of time
	  //First get the number of time points
	  getline(paramfile,input);
	  int number_of_time_points = atoi(input.c_str());
	  //then get the coordinates themselves
	  TCOORD.zeros(number_of_time_points,1,"TCOORD");
	  for (int idx = 0;idx<number_of_time_points;idx++) {
	    getline(paramfile,input);
	    TCOORD.set(idx+1,1,atof(input.c_str()));
	  }
	  //Then we read the SOLARX, Y and Z
	  GRIDX = number_of_time_points;
	  GRIDY = 1;
	  //Import X,Y,Z solar direction vectors
	  SOLARX.zeros(GRIDX,GRIDY,"SOLARX");
	  SOLARX.dlmread(XFILENAME);
	  SOLARX.disp();
	
	  SOLARY.zeros(GRIDX,GRIDY,"SOLARY");
	  SOLARY.dlmread(YFILENAME);
	  SOLARY.disp();
	
	  SOLARZ.zeros(GRIDX,GRIDY,"SOLARZ");
	  SOLARZ.dlmread(ZFILENAME);
	  SOLARZ.disp();
	  //Try a test point
	  cout << "SOLAR X = " << SOLARX.interp(TCOORD,0,0) << " at t=0 " << endl;
	  cout << "SOLAR Y = " << SOLARY.interp(TCOORD,0,0) << " at t=0 " << endl;
	  cout << "SOLAR Z = " << SOLARZ.interp(TCOORD,0,0) << " at t=0 " << endl;
	  printf("Custom Solar Wind Environment Imported Successfully \n");
	  break; //END SOLAR WIND MODEL 3
	}
	case 4: {
	  cout << "Using Solar Wind Model 2D = f(r,t) where r = ||x,y,z||" << endl;
	  getline(paramfile,input);
	  double T_SOLAR_MIN = atof(input.c_str());
	  getline(paramfile,input);
	  double T_SOLAR_MAX = atof(input.c_str());
	  getline(paramfile,input);
	  double R_SOLAR_MIN = atof(input.c_str());
	  getline(paramfile,input);
	  double R_SOLAR_MAX = atof(input.c_str());
	  getline(paramfile,input);
	  double GRIDT = atof(input.c_str());
	  getline(paramfile,input);
	  double GRIDR = atof(input.c_str());
	  getline(paramfile,input);

	  //Create T and R vectors using linspace
	  TCOORD.linspace(T_SOLAR_MIN,T_SOLAR_MAX,GRIDT,"TCOORD");
	  RCOORD.linspace(R_SOLAR_MIN,R_SOLAR_MAX,GRIDR,"RCOORD");
	  TCOORD.disp();
	  RCOORD.disp();
	  //Import X,Y,Z solar direction vectors
	  SOLARX.zeros(GRIDT,GRIDR,"SOLARX");
	  SOLARX.dlmread(XFILENAME);
	  SOLARX.disp();
	
	  SOLARY.zeros(GRIDT,GRIDR,"SOLARY");
	  SOLARY.dlmread(YFILENAME);
	  SOLARY.disp();
	
	  SOLARZ.zeros(GRIDT,GRIDR,"SOLARZ");
	  SOLARZ.dlmread(ZFILENAME);
	  SOLARZ.disp();
	  
	  cout << "SOLAR X = " << SOLARX.interp2(TCOORD,RCOORD,0,0,0) << " at t=0,r=0 " << endl;
	  cout << "SOLAR Y = " << SOLARY.interp2(TCOORD,RCOORD,0,0,0) << " at t=0,r=0 " << endl;
	  cout << "SOLAR Z = " << SOLARZ.interp2(TCOORD,RCOORD,0,0,0) << " at t=0,r=0 " << endl;
	  printf("Custom Solar Wind Environment Imported Successfully \n");
	  break;
	}
	} //END SWITCH CASE 
      } else {
	printf("!!!!!!!!!!! File: %s not found !!!!!!!!!!!! \n",PARAMFILENAME);
	exit(1);
      } //Opening param file
    } //End SOLAR WINDMODEL 2 and 3
  } else {
    printf("File: %s not found \n",ENVIRONMENTFILENAME);
    exit(1);
  } //End error on environment file

  char COEFFFILENAME[256];

  #ifdef USEHILPATH
  sprintf(COEFFFILENAME,"%s","../../GeographicLib/EGM_EMM");
  #else
  sprintf(COEFFFILENAME,"%s","GeographicLib/EGM_EMM");
  #endif

  if (Gravity_Flag == 1) {
    #ifdef __linux__
    egm2008 = new GravityModel("egm2008",COEFFFILENAME); 
    #else
    egm2008 = new GravityModel("egm2008"); //Initializing gravity model
    #endif
    printf("Gravity Model Imported \n");
  }
  if (Magnetic_Flag == 1) {
    #ifdef __linux__
    emm2015 = new MagneticModel("emm2015",COEFFFILENAME); //Initializing magnetic model
    printf("Magnetic Model Imported Using %s \n",COEFFFILENAME);
    #else
    emm2015 = new MagneticModel("emm2015",COEFFFILENAME); 
    printf("Magnetic Model Imported Using %s \n",COEFFFILENAME);
    #endif
  }
  sph_coord.zeros(3,1,"Spherical Coordinate (Phi and Theta)");
  printf("Gravity and Magnet Models Imported but you might need to double check the .emm and .egm file \n");
}

double environment::getSOLARWINDMODEL(MATLAB SOLAR_DIRECTIONin,MATLAB XYZ,double time,MATLAB XYZc) {
  //In order to make sure we don't screw up the XYZ vector
  //Make a copy
  XYZE.overwrite(XYZ);

  //If the solar wind model is 2 substract XYZc from XYZ 
  if (SOLARWINDMODEL == 2) {
    XYZE.minus_eq(XYZc);
  }
  
  //then proceed as before
  return getSOLARWINDMODEL(SOLAR_DIRECTIONin,XYZE,time);
}

double environment::getSOLARWINDMODEL(MATLAB SOLAR_DIRECTIONin,MATLAB XYZ,double time) {
  double SOLARWINDMAGNITUDE = 1;
  switch (SOLARWINDMODEL) {
  case 0: {
    //Constant Force and direction
    SOLAR_DIRECTIONin.overwrite(SOLAR_DIRECTION);
    SOLARWINDMAGNITUDE = 1;
    break;
  }
  case 1: {
    //Exponential Force and constant direction
    SOLAR_DIRECTIONin.overwrite(SOLAR_DIRECTION);
    rSun2Sat.plus(rSun2EarthToday,XYZ);
    double rSun2SatNORM = rSun2Sat.norm();
    double rSun2EarthNORM = rSun2EarthToday.norm();
    double ratio = rSun2EarthNORM/rSun2SatNORM;
    SOLARWINDMAGNITUDE = pow(ratio,1.0); //This was originall 7/6 but apparently it is 7/7 now or 1
    break;
  }
  case 2: {
    //Use SOLARX,SOLARY,SOLARZ based on X and Y coordinates from XYZ
    //Determine if I need to substract the confluence point
    //Hmm. How do I get the confluence point???? Let's make an overloaded function
    SOLAR_DIRECTIONin.set(1,1,SOLARX.interp2(XCOORD,YCOORD,XYZ.get(1,1),XYZ.get(2,1),1));
    SOLAR_DIRECTIONin.set(2,1,SOLARY.interp2(XCOORD,YCOORD,XYZ.get(1,1),XYZ.get(2,1),1));
    SOLAR_DIRECTIONin.set(3,1,SOLARZ.interp2(XCOORD,YCOORD,XYZ.get(1,1),XYZ.get(2,1),1));
    //These file have a magnitude and direction so first save the magnitude
    SOLARWINDMAGNITUDE = SOLAR_DIRECTIONin.norm();
    //Then normalize the vector
    //SOLAR_DIRECTIONin.disp();
    //cout << "MAGNITUDE = " << SOLARWINDMAGNITUDE << endl;
    SOLAR_DIRECTIONin.normalize();
    //SOLAR_DIRECTIONin.disp();
    break;
  }
  case 3: {
    //Use SOLARX,SOLARY,SOLARZ based on time coordinates
    SOLAR_DIRECTIONin.set(1,1,SOLARX.interp(TCOORD,time,0));
    SOLAR_DIRECTIONin.set(2,1,SOLARY.interp(TCOORD,time,0));
    SOLAR_DIRECTIONin.set(3,1,SOLARZ.interp(TCOORD,time,0));
    //These file have a magnitude and direction so first save the magnitude
    SOLARWINDMAGNITUDE = SOLAR_DIRECTIONin.norm();
    //Then normalize the vector
    //SOLAR_DIRECTIONin.disp();
    //cout << "MAGNITUDE = " << SOLARWINDMAGNITUDE << endl;
    SOLAR_DIRECTIONin.normalize();
    //SOLAR_DIRECTIONin.disp();
    break;
  }
  case 4: {
    //Use SOLARX,SOLARY,SOLARZ based on T and R coordinates from XYZ
    SOLAR_DIRECTIONin.set(1,1,SOLARX.interp2(TCOORD,RCOORD,time,XYZ.norm(),0));
    SOLAR_DIRECTIONin.set(2,1,SOLARY.interp2(TCOORD,RCOORD,time,XYZ.norm(),0));
    SOLAR_DIRECTIONin.set(3,1,SOLARZ.interp2(TCOORD,RCOORD,time,XYZ.norm(),0));
    //These file have a magnitude and direction so first save the magnitude
    SOLARWINDMAGNITUDE = SOLAR_DIRECTIONin.norm();
    //Then normalize the vector
    //SOLAR_DIRECTIONin.disp();
    //cout << "MAGNITUDE = " << SOLARWINDMAGNITUDE << endl;
    SOLAR_DIRECTIONin.normalize();
    //SOLAR_DIRECTIONin.disp();
    break;
  }
  }
  return SOLARWINDMAGNITUDE;
}

void environment::getCurrentMagnetic(double simtime,MATLAB State) {
  
  //This routine will only update the magnetic field once
  if (time_magnet_next == -99 && BVECINE.get(1,1) != 0) { 
    time_magnet = simtime + 1e10;
  }

  //If it's time to update the magnetic field go ahead and
  //increment the timer and then proceed
  if (simtime >= time_magnet)  {
    time_magnet += time_magnet_next;
  } else {
    //otherwise return prematurely
    return;
  }
  
  if (Magnetic_Flag == 1)
    {
      double b_east, b_north, b_vertical;
      double bx,by,bz;
      double x = State.get(1, 1);
      double y = State.get(2, 1);
      double z = State.get(3, 1);
      //x = rho*sin(phi)*cos(theta);
      //y = rho*sin(phi)*sin(theta);
      // y/x = tan(theta)
      //z = rho*cos(phi);
      double rho = sqrt((pow(x, 2) + pow(y, 2) + pow(z, 2)));
      double phi;
      if (rho > 0) {
        phi = (acos(z / rho));
      } else {
        phi = 0;
      }
      double the = atan2(y , x);
      double lat = 90 - phi*(180 / PI);
      double lon = the*(180/PI);
      double h = rho-REARTH; //need to send the model the height above the earth's surface
      //printf("x,y,z,rho,H = %lf,%lf,%lf,%lf,%lf \n",x,y,z,rho,h);
      if (h > 0) {
      	/**
      	 * Evaluate the components of the geomagnetic field.
      	 *
      	 * @param[in] t the time (years).
      	 * @param[in] lat latitude of the point (degrees).
      	 * @param[in] lon longitude of the point (degrees).
      	 * @param[in] h the height of the point above the ellipsoid (meters).
      	 * @param[out] Bx the easterly component of the magnetic field (nanotesla).
      	 * @param[out] By the northerly component of the magnetic field (nanotesla).
      	 * @param[out] Bz the vertical (up) component of the magnetic field (nanotesla).
      	 **********************************************************************/
      	emm2015->operator()(yr, lat, lon, h, b_east, b_north, b_vertical);

      	double b_down = -b_vertical; //vertical must be inverted to match NED

      	//Our "spherical reference frame has the following coordinate system
      	bx = b_north;
      	by = b_east;
      	bz = b_down;
      } else {
      	bx = 0;
      	by = 0;
      	bz = 0;
      }

      BVECSPH.set(1, 1, bx); //Btdubs these are all in nT
      BVECSPH.set(2, 1, by);
      BVECSPH.set(3, 1, bz);

      //Our inertial frame is set such that x goes out the equator at
      //the prime meridian, y is orthogonal to x and z goes through
      //the north pole

      //In order to go from spherical to inertial we need to
      //understand that aircraft convention uses the 3-2-1 Euler angle
      //convention

      //Rotation about the z-axis (psiE) - 3
      //rotation about the y-axis (thetaE) - 2
      //rotation about the x-axis (phiE) - 1
      //However, these are not the same as phi and the from the
      //spherical reference frame. Longitude is measure to the right
      //but this is actually a positive rotation about z. Thus
      double psiE = the;
      //Furthermore, phi is a positive rotation about the y-axis but you need to add pi to make sure
      //the z-component points downwards
      double thetaE = phi+PI;
      //Finally, there is no rotation about the x-axis
      double phiE = 0;

      //With these "Euler" Angles defined we can convert the spherical
      //coordinates to inertial coordinates.
      sph2ine(phiE, thetaE, psiE);
    }
  else
    {
      //If the magnetic field model is off just set them to zero
      BVECSPH.mult_eq(0.0);
      BVECINE.mult_eq(0.0);
    }
  return;
}

void environment::sph2ine(double phi, double the, double psi)
{
  sph_coord.set(1,1,phi);
  sph_coord.set(2,1,the);
  sph_coord.set(3,1,psi);
  //sph_coord.disp();
  sph2ine32.L321(sph_coord,0); //0 for Euler Angles
  //sph2ine32.disp();
  //vecSPH.disp();
  sph2ine32.rotateBody2Inertial(BVECINE,BVECSPH);
  //vecI.disp();
}

double environment::getCurrentDensity() {
  return RHOSLSI;
}

void environment::EarthEphemeris(MATLAB Si) {
  //Overloaded function call. Use the day month and year to call Earth Ephemeris
  EarthEphemeris(Si,julian_today);
}

void environment::EarthEphemeris(MATLAB Si,double julian_day) {
  //First step to Figure this out is I need to now where the Sun is. Well that's easy. It's 0,0,0. 
  //Really when people say the Sun's ephemeris they mean the Earth's Ephemeris data
  //I've learned this in my Orbital Mechanics Text book but I may as well just look this up again.
  //I don't want to make this fancy with input files so I'm just going to hard code everything.
  //So JPL has this system called the HORIZONS
  //https://ssd.jpl.nasa.gov/?horizons
  //In order to access the HORIZONS System simply type the following command

  //$ telnet horizons.jpl.nasa.gov 6775

  //You can use crude models here
  //https://ssd.jpl.nasa.gov/txt/p_elem_t1.txt -- I've saved this txt to the Gitlab repo as well
  //So First step is to follow the PDF titled - aprx_pos_planets.pdf
  //An example Implementation of this code is in Fundamentals_Astrodynamics using the routine julian_day_orbit.py
  //Really you need to look at the Universe.py module. Anywho here is how that code works.

  //First we need to get some orbital elements of the Earth which are defined from Jan 1st 2000
  double a0 = 1.00000261; //Semi major axis
  double e0 = 0.01671123; //eccentricity (0 is a perfect circle)
  double i0 = -0.00001531; //inclination in degrees
  double L0 = 100.46457166; //mean longitude in degrees
  double wbar0 = 102.93768193; //longitude of the perihelion
  double OMEGA0 = 0.0; //longitude of the ascending node
  //In order to get these parameters at the particular date in time we need to compute the rate 
  double adot = 0.00000562; //these are all /century
  double edot = -0.00004392;
  double idot = -0.01294668;
  double Ldot = 35999.37244981;
  double wbardot = 0.32327364;
  double OMEGAdot = 0.0;
  //#compute the orbital Elements for this particular Julian Day (2451545 is Jan 1st 2000)
  double T = (julian_day - 2451545.0)/36525.0;
  double AU = 149597870700.0; //this is 1 Astronaumical Unit in meters - the distance from Earth to the Sun
  double a = (a0 + T*adot)*AU;
  double e = e0 + T*edot; //##This is in radians
  double i = i0 + T*idot; //this is in degrees
  double L = L0 + T*Ldot; //deg
  double wbar = wbar0 + T*wbardot; //deg
  double OMEGA = OMEGA0 + T*OMEGAdot; //deg
  //If you're trying to find the orbital elements of any planet past Mars you need to add correction factors but 
  //Earth doesn't need any
  //#Compute estar
  double estar = e*180.0/PI;
  //#Argument of the perihelion
  double w = wbar - OMEGA;
  //#Mean Anomaly - No correction factors
  double M = L - wbar;
  //#Need to modulus M
  while (M > 180) { 
    M -= 360;
  }
  while (M < -180) {
    M += 360;
  }
  //#Solve for Eccentric Anomaly
  //#M = E - estar*sin(E)
  double E = M + estar*sin(M*PI/180.0);
  double dM = 1;
  double dE = 0;
  while (abs(dM) > 1e-6) {
    dM = M - (E - estar*sin(E*PI/180.0));
    dE = dM/(1.0-e*cos(E*PI/180.0));
    E += dE;
  }        
  //#Compute the semi latus rectum
  double p = a*(1-e*e);
  //#Compute coordinate of planet in ecliptic plane of the planet
  double xprime = a*(cos(E*PI/180.0)-e);
  double yprime = a*sqrt(1-e*e)*sin(E*PI/180.0);
  double zprime = 0.0;

  //#Convert certain parameters to radians
  w *= PI/180.0;
  OMEGA *= PI/180.0;
  i *= PI/180.0;

  //#Compute coordinate of planet in the J2000 frame or the ecliptic plane of the sun
  double x0 = (cos(w)*cos(OMEGA) - sin(w)*sin(OMEGA)*cos(i))*xprime + (-sin(w)*cos(OMEGA)-cos(w)*sin(OMEGA)*cos(i))*yprime;
  double y0 = (cos(w)*sin(OMEGA) + sin(w)*cos(OMEGA)*cos(i))*xprime + (-sin(w)*sin(OMEGA)+cos(w)*cos(OMEGA)*cos(i))*yprime;
  double z0 = (sin(w)*sin(i))*xprime + (cos(w)*sin(i))*yprime;
  //If you did it right you should get the following coordinates for julian_day = 2458485 (jan 1st 2019)
  //in the J2000 frame 
  //xyz = [-26831520440.7 144634231633.0 -6248426.35536]

  Si.set(1,1,x0);
  Si.set(2,1,y0);
  Si.set(3,1,z0);
}

