CC=g++
EXECUTABLE=FASTSim.exe
CSOURCES=$(wildcard *.c)
CPPSOURCES=$(wildcard *.cpp)
COBJECTS=$(CSOURCES:.c=.o)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(COBJECTS) $(CPPOBJECTS)
COMPILE=-c -w -O3
EXETYPE=-DSIL #-DSIMONLY #-DSIL #-DHIL #-DAUTO 
HARDWARE=-DDESKTOP #-DRPI -DARDUINO
FLAGS=-DDEBUG
LIB= -L/usr/local/lib 
RENDER=-lGL -lGLU -lglut #you need these if you're compiling OpenGL #sudo apt-get install freeglut3-dev
THREAD=-lpthread -lboost_system -lboost_thread -lboost_date_time #We aren't using this anymore so yea #sudo apt-get install libboost-all-dev

#Flags

#VEHICLE TYPE IS SELECTED IN THE MAKE LINKS FILE

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

all: $(CSOURCES) $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB) $(RENDER) $(THREAD)
.c.o:
	$(CC) $(COMPILE) $(FLAGS) $(HARDWARE) $(EXETYPE) $< -o $@ $(LIB) $(RENDER) $(THREAD)
.cpp.o:
	$(CC) $(COMPILE) $(FLAGS) $(HARDWARE) $(EXETYPE) $< -o $@ $(LIB) $(RENDER) $(THREAD)
clean:
	rm -rf $(EXECUTABLE) *.o
rebuild:
	make clean
	make
