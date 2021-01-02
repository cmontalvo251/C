CC=g++
EXECUTABLE=FASTSim.exe
CSOURCES=$(wildcard *.c)
CPPSOURCES=$(wildcard *.cpp)
COBJECTS=$(CSOURCES:.c=.o)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(COBJECTS) $(CPPOBJECTS)
COMPILE=-c -w -O3
VEHICLE=-DPOINTMASS #-DPORTALCUBE #-DAIRCRAFT #-DQUADCOPTER #-DX8
EXETYPE=-DSIMONLY #-DSIL #-DHIL #-DAUTO 
HARDWARE=-DDESKTOP #-DRPI -DARDUINO
FLAGS=-DDEBUG
LIB= -L/usr/local/lib 

#Flags

##VEHICLE
#Vehicle can be anything
#Point Mass, Portal Cube, Aircraft, Quadcopter or DX8

##EXETYPE
#SIMONLY - this as fast as possible to generate plots
#SIL - this is software in the loop 
#HIL - this is in real time and thus 
#AUTO - this is no simulation at all and will run on control board

#HARDWARE
#DESKTOP - 
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

all: $(CSOURCES) $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB)
.c.o:
	$(CC) $(COMPILE) $(FLAGS) $(HARDWARE) $(EXETYPE) $(VEHICLE) $< -o $@ $(LIB)
.cpp.o:
	$(CC) $(COMPILE) $(FLAGS) $(HARDWARE) $(EXETYPE) $(VEHICLE) $< -o $@ $(LIB)
clean:
	rm -rf $(EXECUTABLE) *.o
rebuild:
	make clean
	make
