#DO NOT CHANGE THIS
CPPDIR=../

##############CHANGE EVERYTHING BELOW HERE####################

###############WHAT SIMULATION TYPE DO YOU WANT###############
#SIMONLY - this as fast as possible to generate plots
#SIL - this is software in the loop running in realtime
#HIL - this is in real time and also requires Serial communication betwee desktop and computer
#AUTO - this is no simulation at all and will run on control board
#EXETYPE=-DSIMONLY
EXETYPE=-DSIL
#EXETYPE=-DAUTO
#############################################################

#################WHAT HARDWARE ARE YOU RUNNING ON#############
#DESKTOP - running on a laptop or desktop macine
#RPI - running on a raspbery pi 
#ARDUINO - running on an arduino (may require a separate ino)
HARDWARE=-DDESKTOP
#HARDWARE=-DRPI
###########################################################

###################RENDER????########################
#Either OpenGL or nothing
#RENDER=
#OPENGLSOURCES=
RENDER=-lGL -lGLU -lglut #you need these if you're compiling OpenGL #sudo apt-get install freeglut3-dev
OPENGLSOURCES=${CPPDIR}/OpenGL/opengl.cpp
################################################################

###################WHAT TYPE OF TRANSMITTER ARE YOU USING?#########
#RCTECH - USB Controller in FASTLab
#XBOX - USB XBOX controller at home
#FLYSKY - Iris+ transmitter in FASTLab for Iris+ drones
CONTROLLER=-DXBOX
#CONTROLLER=-DRCTECH
#CONTROLLER=-DFLYSKY
############################################################

#########WHAT MODEL DO YOU WANT??##############
#This needs to point to a source folder with aerodynamics.* and controller.*
#MODELPATH=PortalCube/src
#MODELPATH=Quadcopter/src
#MODELPATH=Tank/src
MODELPATH=Airplane/src
###########################################

#####################WHERE IS HIL LOCATED?#####################
#HPATH=/home/mjcobar13/Documents/GIT/CMontalvo251/HIL
HPATH=/home/carlos/Git_Repos/Github/HIL
#HPATH=/home/carlos/Files/Git_Repos/Github/HIL
#HPATH=/home/pi/HIL
##########################################################

#################DO NOT CHANGE ANYTHING BELOW HERE###########

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
CPPSOURCES=$(wildcard *.cpp ${CPPDIR}Timer/timer.cpp ${CPPDIR}MATLAB/MATLAB.cpp ${CPPDIR}Mathp/mathp.cpp ${CPPDIR}/Datalogger/Datalogger.cpp ${CPPDIR}/6DOF/Rotation3.cpp ${CPPDIR}/Sensors/sensors.cpp ${CPPDIR}/RK4/RK4.cpp)
IMUSOURCES=$(wildcard ${HPATH}/IMU/*.cpp)
GPSSOURCES=$(wildcard ${HPATH}/GPS/*.cpp)
BAROSOURCES=$(wildcard ${HPATH}/Baro/*.cpp)
ADCSOURCES=$(wildcard ${HPATH}/ADC/*.cpp)
RCIOSOURCES=$(wildcard ${HPATH}/RCIO/*.cpp ${HPATH}/Util/Util.cpp)
MODELSOURCES=$(wildcard ${MODELPATH}/*.cpp)
SOURCES=${CPPSOURCES} ${IMUSOURCES} ${RCIOSOURCES} ${MODELSOURCES} ${OPENGLSOURCES} ${GPSSOURCES} ${BAROSOURCES} ${ADCSOURCES}
OBJECTS=$(SOURCES:.cpp=.o)
#COMPILE=-c -w -O3 #MIGHT NEED THIS LATER FOR SIL AND SIMONLY
COMPILE=-c -w -std=c++11 -Wno-psabi
FLAGS=-DDEBUG
LIB=-L/usr/local/lib
INCLUDE=-I${MODELPATH} -I../ -I${HPATH}
THREAD=-lpthread -lboost_system -lboost_thread -lboost_date_time #We are using this for the rendering pipeline so yea #sudo apt-get install libboost-all-dev


all: $(SOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB) $(INCLUDE) $(RENDER) $(THREAD)
.cpp.o:
	$(CC) $(COMPILE) $(FLAGS) $(CONTROLLER) $(HARDWARE) $(EXETYPE) $< -o $@ $(LIB) $(INCLUDE) $(RENDER) $(THREAD)
clean:
	echo ' ' > logs/d.txt
	rm logs/*.txt
	echo ' ' > d.exe
	echo ' ' > d.o
	rm *.exe *.o
	echo ' ' > ../MATLAB/d.o
	rm ../MATLAB/*.o
	echo ' ' > ../Datalogger/d.o
	rm ../Datalogger/*.o
	echo ' ' > ../Mathp/d.o
	rm ../Mathp/*.o
	echo ' ' > ../6DOF/d.o
	rm ../6DOF/*.o
	echo ' ' > ${HPATH}/Util/d.o
	rm ${HPATH}/Util/*.o
	echo ' ' > ${HPATH}/RCIO/d.o
	rm ${HPATH}/RCIO/*.o
	echo ' ' > ${MODELPATH}/d.o
	rm ${MODELPATH}/*.o
	echo ' ' > ../Timer/d.o
	rm ../Timer/*.o
	echo ' ' > ../Sensors/d.o
	rm ../Sensors/*.o
	echo ' ' > ../RK4/d.o
	rm ../RK4/*.o
	echo ' ' > ../OpenGL/d.o
	rm ../OpenGL/*.o
	echo ' ' > ${HPATH}/IMU/d.o
	rm ${HPATH}/IMU/*.o
	echo ' ' > ${HPATH}/Baro/d.o
	rm ${HPATH}/Baro/*.o
	echo ' ' > ${HPATH}/GPS/d.o
	rm ${HPATH}/GPS/*.o
	echo ' ' > ${HPATH}/ADC/d.o
	rm ${HPATH}/ADC/*.o
rebuild:
	make clean
	make
