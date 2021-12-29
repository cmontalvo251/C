#########FAST MAKEFILE##########
MODEL=PortalCube
RX=KEYBOARD
PLATFORM=DESKTOP
TYPE=SIMONLY
EXECUTABLE=simonly.exe
MAIN=SIM/main.o

###COMPILER AND OTHER FLAGS
CC=g++
COMMON=Common
HARDWARE=Hardware
COMPILE=-c -w -std=c++11 -Wno-psabi
FLAGS=-DDEBUG
LIB=-L/usr/local/lib
THREAD=-lpthread -lboost_system -lboost_thread -lboost_date_time #We are using this for the rendering pipeline so yea #sudo apt-get install libboost-all-dev
MODELPATH=Models/$(MODEL)/src
INCLUDE=-I${COMMON} -I${HARDWARE} -I${MODELPATH}
###COMMON
COMMONSOURCES=$(wildcard $(COMMON)/*/*.cpp)
###HARDWARE
HARDWARESOURCES=$(wildcard $(HARDWARE)/*/*.cpp)
###MODEL
MODELSOURCES=$(wildcard Models/$(MODEL)/src/*.cpp)

###COMBINE ALL SOURCES
SOURCES=$(COMMONSOURCES) $(HARDWARESOURCES) $(MODELSOURCES)
OBJECTS=$(SOURCES:.cpp=.o)

##First target is default if you just type make
simonly:
	#MAKING SIMONLY
	make all

#Target SIL needs type to be SIL
sil:
	#MAKING SIL
	make all TYPE="SIL" EXECUTABLE="sil.exe"

##Target to make all depends on the MAIN, OBJECTS and the EXECUTABLE
all: $(OBJECTS) $(MAIN) $(EXECUTABLE) 

##Rule for executable depends on the OBJECTS and MAIN
$(EXECUTABLE): $(OBJECTS) $(MAIN)
	$(CC) $(OBJECTS) $(MAIN) -o $(EXECUTABLE)

##Target MAIN depends on it's respective cpp and h file
$(MAIN): $(MAIN:.o=.cpp) $(MAIN:.o=.h)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(RX) -D$(PLATFORM) -D$(TYPE) $(MAIN:.o=.cpp) -o $(MAIN)

##The rule for the objects depends on the sources
.cpp.o: $(SOURCES)
	$(CC) $(COMPILE) $(FLAGS) $(INCLUDE) -D$(RX) -D$(PLATFORM) -D$(TYPE) $< -o $@

##Rebuild function
rebuild:
	make clean
	make simonly

##Clean function
clean:
	echo ' ' > simonly
	rm simonly
	echo ' ' > SIM/d.o
	rm SIM/*.o
	echo ' ' > $(COMMON)/Datalogger/*.o
	rm $(COMMON)/*/*.o
	echo ' ' > $(HARDWARE)/Serial/*.o
	rm $(HARDWARE)/*/*.o

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
