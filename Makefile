#############UNCOMMENT HPATH, ARGS and MODEL IF YOU WANT TO DO 
##############THIS THE OLD FASHIONED WAY

#########WHAT MODEL DO YOU WANT??##############
#This needs to point to a source folder with aerodynamics.* and controller.*
#MODEL=PortalCube
#MODEL=Quadcopter
#MODEL=Tank
#MODEL=X8
#MODEL=Airplane
###########################################

###################WHAT TYPE OF TRANSMITTER ARE YOU USING?#########
#SET THE RX VAR
#RCTECH - USB Controller in FASTLab
#XBOX - USB XBOX controller at home
#FLYSKY - Iris+ transmitter in FASTLab for Iris+ drones
############################################################

#################WHAT HARDWARE ARE YOU RUNNING ON#############
#DESKTOP - running on a laptop or desktop macine
#RPI - running on a raspbery pi 
#ARDUINO - running on an arduino (may require a separate ino)
###########################################################

###############WHAT SIMULATION TYPE DO YOU WANT###############
#SIMONLY - this as fast as possible to generate plots
#SIL - this is software in the loop running in realtime
#HIL - this is in real time and also requires Serial communication betwee desktop and computer
#AUTO - this is no simulation at all and will run on control board
#############################################################

#################DO NOT CHANGE ANYTHING BELOW HERE###########

##NON USER FLAGS
##When running on Desktop control defaults to JOYSTICK, and sensors default to SENSORBLOCK
##When running on RPI control defaults to RECEIVER, and sensors default to IMURPI
##When running HIL on any platform REALTIME flag is set
##When running in HIL/SIL on Desktop - OpenGL is compiled by default

##CONTROL TYPES - JOYSTICK, RECEIVER
##SENSOR TYPES - SENSORBLOCK, IMURPI
##OTHER - REALTIME (for HIL)

all:
	### DEFAULT MAKE ALL
	make $(OBJECTS) TYPE="SIMONLY" RX="KEYBOARD" MODEL="Tank" PLATFORM="DESKTOP"
	make $(EXECUTABLE) TYPE="SIMONLY" RX="KEYBOARD" MODEL="Tank" PLATFORM="DESKTOP"
simonly: $(SOURCES) $(EXECUTABLE)
sil: $(SOURCES) $(EXECUTABLE)
hil_server: $(SOURCES) $(EXECUTABLE)
hil_client: $(SOURCES) $(EXECUTABLE)
auto: $(SOURCES) $(EXECUTABLE)
logger: $(SOURCES) $(EXECUTABLE)


ifeq ($(TYPE),AUTO)
	RENDER=-lGL -lGLU -lglut #you need these if you're compiling OpenGL #sudo apt-get install freeglut3-dev
	OPENGLSOURCES=${CPPDIR}/OpenGL/opengl.cpp
	WIRINGPI=-lwiringpi
else
	RENDER=
	OPENGLSOURCES=
	WIRINGPI=
endif	
CC=g++
HPATH=Hardware/
CPPDIR=Common/
EXECUTABLE=FAST.exe
MODELPATH=Models/$(MODEL)/src
CPPSOURCES=$(wildcard ${CPPDIR}Timer/timer.cpp ${CPPDIR}MATLAB/MATLAB.cpp ${CPPDIR}Mathp/mathp.cpp ${CPPDIR}/Datalogger/Datalogger.cpp ${CPPDIR}/6DOF/Rotation3.cpp ${CPPDIR}/Sensors/sensors.cpp ${CPPDIR}/RK4/RK4.cpp ${CPPDIR}Dynamics/Dynamics.cpp)
IMUSOURCES=$(wildcard ${HPATH}/IMU/*.cpp)
GPSSOURCES=$(wildcard ${HPATH}/GPS/*.cpp)
BAROSOURCES=$(wildcard ${HPATH}/Baro/*.cpp)
ADCSOURCES=$(wildcard ${HPATH}/ADC/*.cpp)
RCIOSOURCES=$(wildcard ${HPATH}/RCIO/*.cpp ${HPATH}/Util/Util.cpp)
MODELSOURCES=$(wildcard ${MODELPATH}/*.cpp)
TELEMETRYSOURCES=$(wildcard ${HPATH}/Serial/Telemetry.cpp)
SOURCES=${CPPSOURCES} ${IMUSOURCES} ${RCIOSOURCES} ${MODELSOURCES} ${OPENGLSOURCES} ${GPSSOURCES} ${BAROSOURCES} ${ADCSOURCES} ${TELEMETRYSOURCES}
OBJECTS=$(SOURCES:.cpp=.o)
#COMPILE=-c -w -O3 #MIGHT NEED THIS LATER FOR SIL AND SIMONLY
COMPILE=-c -w -std=c++11 -Wno-psabi
FLAGS=-DDEBUG
LIB=-L/usr/local/lib
INCLUDE=-I${MODELPATH} -I${CPPDIR} -I${HPATH}
THREAD=-lpthread -lboost_system -lboost_thread -lboost_date_time #We are using this for the rendering pipeline so yea #sudo apt-get install libboost-all-dev

$(EXECUTABLE):
	$(CC) $(OBJECTS) -o $@ $(LIB) $(WIRINGPI) $(INCLUDE) $(RENDER) $(THREAD)
$(OBJECTS): $(SOURCES)
	$(CC) $(COMPILE) $(FLAGS) -D$(RX) -D$(PLATFORM) -D$(TYPE) $< -o $@ $(WIRINGPI) $(LIB) $(INCLUDE) $(RENDER) $(THREAD)
clean:
	echo ' ' > logs/d.txt
	rm logs/*.txt
	echo ' ' > d.exe
	echo ' ' > d.o
	rm *.exe *.o
	echo ' ' > ${CPPDIR}MATLAB/d.o
	rm ${CPPDIR}MATLAB/*.o
	echo ' ' > ${CPPDIR}Datalogger/d.o
	rm ${CPPDIR}Datalogger/*.o
	echo ' ' > ${CPPDIR}Mathp/d.o
	rm ${CPPDIR}Mathp/*.o
	echo ' ' > ${CPPDIR}Rotation/d.o
	rm ${CPPDIR}Rotation/*.o
	echo ' ' > ${CPPDIR}Dynamics/d.o
	rm ${CPPDIR}Dynamics/*.o
	echo ' ' > ${HPATH}/Util/d.o
	rm ${HPATH}/Util/*.o
	echo ' ' > ${HPATH}/RCIO/d.o
	rm ${HPATH}/RCIO/*.o
	echo ' ' > Models/Airplane/d.o
	rm Models/Airplane/*.o
	echo ' ' > Models/PortalCube/d.o
	rm Models/PortalCube/*.o
	echo ' ' > Models/Quadcopter/d.o
	rm Models/Quadcopter/*.o	
	echo ' ' > Models/Tank/d.o
	rm Models/Tank/*.o
	echo ' ' > Models/X8/d.o
	rm Models/X8/*.o
	echo ' ' > ${CPPDIR}Timer/d.o
	rm ${CPPDIR}Timer/*.o
	echo ' ' > ${HPATH}Sensors/d.o
	rm ${HPATH}Sensors/*.o
	echo ' ' > ${CPPDIR}RK4/d.o
	rm ${CPPDIR}RK4/*.o
	echo ' ' > ${CPPDIR}OpenGL/d.o
	rm ${CPPDIR}OpenGL/*.o
	echo ' ' > ${HPATH}/IMU/d.o
	rm ${HPATH}/IMU/*.o
	echo ' ' > ${HPATH}/Baro/d.o
	rm ${HPATH}/Baro/*.o
	echo ' ' > ${HPATH}/GPS/d.o
	rm ${HPATH}/GPS/*.o
	echo ' ' > ${HPATH}/ADC/d.o
	rm ${HPATH}/ADC/*.o
	echo ' '> ${HPATH}/Serial/d.o
	rm ${HPATH}/Serial/*.o
rebuild:
	make clean
	make
help:
	##########FROM NOW ON RUN YOUR MAKE FILE LIKE THIS###
	# For AUTO Mode on Pi for Airplane
	# make rebuild ARGS="-DFLYSKY -DRPI -DAUTO" MODEL="Airplane" HPATH="/home/pi/HIL" RENDER="" OPENGLSOURCES="" WIRINGPI="-lwiringPi"
	# For SIMONLY on Desktop for Tank
	# make rebuild ARGS="-DRCTECH -DDESKTOP -DSIMONLY" MODEL="Tank" HPATH="/home/carlos/Git_Repos/Github/HIL"
	# Your choices for models are:
	# PortalCube
	# Airplane
	# Quadcopter
	# Tank
	# X8
	# Note you can always default back to just
	# make rebuild
	# provided you set ARGS, MODEL, and HPATH on the first 3 lines of code
	####################################################
