EXETYPE=-DSIMONLY #-DSIL #-DSIMONLY #-DSIL #-DHIL #-DAUTO 
HARDWARE=-DDESKTOP #-DRPI #-DDESKTOP #-DRPI -DARDUINO
CONTROLLER=-DXBOX #-DRCTECH #-DXBOX
MODELPATH=PortalCube/src

#Flags

#MODELTYPE
#This needs to point to a source folder with aerodynamics.* and controller.*

##EXETYPE
#SIMONLY - this as fast as possible to generate plots
#SIL - this is software in the loop running in realtime
#HIL - this is in real time and also requires Serial communication betwee desktop and computer
#AUTO - this is no simulation at all and will run on control board

#HARDWARE
#DESKTOP - running on a laptop or desktop macine
#RPI - running on a raspbery pi 
#ARDUINO - running on an arduino (may require a separate ino)

##NON USER FLAGS
##When running on Desktop control defaults to JOYSTICK, and sensors default to SENSORBLOCK
##When running on RPI control defaults to RECEIVER, and sensors default to IMURPI
##When running HIL on any platform REALTIME flag is set
##When running in HIL/SIL on Desktop - OpenGL is compiled by default

##CONTROL TYPES - JOYSTICK, RECEIVER
##SENSOR TYPES - SENSORBLOCK, IMURPI
##OTHER - REALTIME (for HIL)

CC=g++
EXECUTABLE=FASTSim.exe
CSOURCES=$(wildcard *.c)
CPPSOURCES=$(wildcard *.cpp ../MATLAB/MATLAB.cpp ../Datalogger/Datalogger.cpp ../6DOF/Rotation3.cpp ../Sensors/sensors.cpp ../RCInput/RCInput.cpp ${MODELPATH}/aerodynamics.cpp ${MODELPATH}/controller.cpp ../RK4/RK4.cpp ../Mathp/mathp.cpp)
COBJECTS=$(CSOURCES:.c=.o)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(COBJECTS) $(CPPOBJECTS)
COMPILE=-c -w -O3
FLAGS=-DDEBUG
LIB=-L/usr/local/lib
INCLUDE=-I../MATLAB/ -I../Mathp/ -I../6DOF/ -I../RCInput/ -I${MODELPATH} -I../Timer/ -I../Sensors/ -I../Datalogger/ -I../RK4/
#No RENDER IF RPI
RENDER=-lGL -lGLU -lglut #you need these if you're compiling OpenGL #sudo apt-get install freeglut3-dev
THREAD=-lpthread -lboost_system -lboost_thread -lboost_date_time #We are using this for the rendering pipeline so yea #sudo apt-get install libboost-all-dev


all: $(CSOURCES) $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB) $(INCLUDE) $(RENDER) $(THREAD)
.c.o:
	$(CC) $(COMPILE) $(FLAGS) $(CONTROLLER) $(HARDWARE) $(EXETYPE) $< -o $@ $(LIB) $(INCLUDE) $(RENDER) $(THREAD)
.cpp.o:
	$(CC) $(COMPILE) $(FLAGS) $(CONTROLLER) $(HARDWARE) $(EXETYPE) $< -o $@ $(LIB) $(INCLUDE) $(RENDER) $(THREAD)
clean:
	echo ' ' > logs/d.txt
	rm -rf $(EXECUTABLE) *.o
	rm logs/*.txt
	rm ../MATLAB/*.o
	rm ../Datalogger/*.o
	rm ../Mathp/*.o
	rm ../6DOF/*.o
	rm ../RCInput/*.o
	rm ${MODELPATH}/*.o
	rm ../Timer/*.o
	rm ../Sensors/*.o
	rm ../RK4/*.o
rebuild:
	make clean
	make
