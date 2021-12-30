#########FAST MAKEFILE##########
MODEL=PortalCube
RX=KEYBOARD
PLATFORM=DESKTOP
TYPE=SIMONLY
EXECUTABLE=simonly.exe
MAIN=SIM/main.o
OPENGLSOURCES=
RENDER=
THREAD=
WIRINGPI=

###COMPILER AND OTHER FLAGS
CC=g++
COMMON=Common
HARDWARE=Hardware
GEO=GeographicLib
COMPILE=-c -w -std=c++11 -Wno-psabi
FLAGS=-DDEBUG
LIB=-L/usr/local/lib -L./
THREAD=-lpthread -lboost_system -lboost_thread -lboost_date_time #We are using this for the rendering pipeline so yea #sudo apt-get install libboost-all-dev
MODELPATH=Models/$(MODEL)/src
INCLUDE=-I${COMMON} -I${HARDWARE} -I${MODELPATH} -I./ -I${GEO}
###COMMON
COMMONSOURCES=$(wildcard $(COMMON)/*/*.cpp)
###HARDWARE
HARDWARESOURCES=$(wildcard $(HARDWARE)/*/*.cpp)
###MODEL
MODELSOURCES=$(wildcard Models/$(MODEL)/src/*.cpp)
###GEOGRAPHIC LIB
GEOSOURCES=$(wildcard ${GEO}/*.cpp)

###COMBINE ALL SOURCES
SOURCES=$(COMMONSOURCES) $(HARDWARESOURCES) $(MODELSOURCES) $(OPENGLSOURCES) $(GEOSOURCES)
OBJECTS=$(SOURCES:.cpp=.o)

##First target is default if you just type make
simonly:
	#MAKING SIMONLY
	make all

#Target SIL needs type to be SIL
sil:
	#MAKING SIL
	#You need these if you're compiling OpenGL 
	#sudo apt-get install freeglut3-dev
    #We are also using boost for the rendering pipeline 
    #sudo apt-get install libboost-all-dev
	make all TYPE="SIL" EXECUTABLE="sil.exe" RENDER="-lGL -lGLU -lglut" OPENGLSOURCES="OpenGL/opengl.cpp" THREAD="-lpthread -lboost_system -lboost_thread -lboost_date_time"

#Target HIL has two different options on PI and DESKTOP need to make sure you pick one
#or the other
#This is going to be somewhat difficult because on the pi you need wiringpi
hil: 
	make all TYPE="HIL" EXECUTABLE="hil.exe"

#Target auto is always on Rpi with the FLYSKY transmitter you also need wiring pi
auto:
	make all TYPE="AUTO" EXECUTABLE="auto.exe" PLATFORM="RPI" RX="FLYSKY" WIRINGPI="-lwiringpi"

#Demo is in auto mode on RPI but main is DEMO/demo.cpp
demo:
	make all TYPE="AUTO" EXECUTABLE="demo.exe" PLATFORM="RPI" RX="FLYSKY" MAIN="DEMO/demo.o" WIRINGPI="-lwiringpi"

##Logger is on RPI but all it does is take data
logger:
	make all TYPE="AUTO" EXECUTABLE="logger.cpp" PLATFORM="RPI" MAIN="LOGGER/logger.cpp" WIRINGPI="-lwiringpi"

##Target to make all depends on the MAIN, OBJECTS and the EXECUTABLE
all: $(OBJECTS) $(MAIN) $(EXECUTABLE) 

##Rule for executable depends on the OBJECTS and MAIN
$(EXECUTABLE): $(OBJECTS) $(MAIN)
	$(CC) $(OBJECTS) $(MAIN) -o $(EXECUTABLE) $(LIB) $(INCLUDE) $(RENDER) $(THREAD) $(WIRINGPI)

##Target MAIN depends on it's respective cpp and h file
$(MAIN): $(MAIN:.o=.cpp) $(MAIN:.o=.h)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(MAIN:.o=.cpp) -o $(MAIN) $(WIRINGPI)

##The rule for the objects depends on the sources
.cpp.o: $(SOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(LIB) $(WIRINGPI) $< -o $@

##Rebuild function
rebuild:
	make clean
	make simonly

##Clean function
clean:
	echo ' ' > d.exe
	rm *.exe
	echo ' ' > SIM/d.o
	rm SIM/*.o
	echo ' ' > LOGGER/d.o
	rm LOGGER/*.o
	echo ' ' > DEMO/d.o
	rm DEMO/*.o
	echo ' ' > $(COMMON)/Datalogger/d.o
	rm $(COMMON)/*/*.o
	echo ' ' > $(HARDWARE)/Serial/d.o
	rm $(HARDWARE)/*/*.o
	echo ' ' > ${GEO}/d.o
	rm ${GEO}/*.o
	echo ' ' > OpenGL/*.o
	rm OpenGL/*.o

###Help function
help:
	##WHEN COMPILING YOU MUST SELECT THE MODEL,PLATFORM and RX
	## MODEL=Tank or Airplane or PortalCube or X8 or Quadcopter
	## RX=RCTECH or KEYBOARD or FLYSKY or XBOX
	## PLATFORM=DESKTOP or RPI or ARDUINO
	## If you select nothing the defaults will be used
	## Defaults are PortalCube, KEYBOARD, DEKSTOP
	## The type of simulation is dictated by what follows after make
	## Type include: simonly, sil, hil and auto
	## Here's an example for sil on DESKTOP
	## make sil MODEL="Airplane"
