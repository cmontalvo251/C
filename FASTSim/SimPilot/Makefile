#####################WHERE IS HIL LOCATED?#####################
#HPATH=/home/mjcobar13/Documents/GIT/CMontalvo251/HIL
#HPATH=/home/carlos/Git_Repos/Github/HIL
HPATH=/home/carlos/Files/Git_Repos/Github/HIL
#HPATH=/home/pi/HIL
##########################################################

##############WHAT VEHICLE ARE YOU RUNNING THIS ON###########
#Quadcopter,PortalCube
MODEL=Quadcopter
############################################################

#########################WHAT MODE ARE YOU RUNNING IN?#############
#AUTO,SIL
#EXETYPE=-DAUTO ##For running in autonomous mode rather than simulation
EXETYPE=-DSIL ##For running in real time
########################################################

###############################HARDWARE#######################
#DESKTOP, RPI
HARDWARE=-DDESKTOP #on Desktop
#HARDWARE=-DRPI #on the raspberry pi
########################################################

#####################WHAT TYPE OF TRANSMITTER ARE YOU USING?###############
#FLYSKY, XBOX, RCTECH
#CONTROLLER=-DFLYSKY ##IRIS 3DR transmitters in FASTLab
#CONTROLLER=-DXBOX
CONTROLLER=-DRCTECH
#######################################################################

CC=g++
EXECUTABLE=SimPilot.exe
CPPDIR=../../
MODELPATH=../
CPPSOURCES=$(wildcard *.cpp ${CPPDIR}Timer/timer.cpp ${CPPDIR}MATLAB/MATLAB.cpp ${CPPDIR}Mathp/mathp.cpp)
IMUSOURCES=$(wildcard ${HPATH}/IMU/*.cpp)
RCIOSOURCES=$(wildcard ${HPATH}/RCIO/*.cpp ${HPATH}/Util/Util.cpp)
MODELSOURCES=$(wildcard ${MODELPATH}${MODEL}/src/*.cpp)
SOURCES=${CPPSOURCES} ${IMUSOURCES} ${RCIOSOURCES} ${MODELSOURCES}
OBJECTS=$(SOURCES:.cpp=.o)
COMPILE=-c -w -O3
LIB=-L/usr/local/lib
INCLUDE=-I../../ -I${HPATH} -I${MODELPATH}${MODEL}/src

all: $(SOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB) $(INCLUDE)
.cpp.o:
	$(CC) $(COMPILE) $(EXETYPE) $(HARDWARE) $(CONTROLLER) $< -o $@ $(LIB) $(INCLUDE) $(RENDER) $(THREAD)
who:
	echo 'CPP Sources' ${CPPSOURCES}
	echo 'IMU Sources' ${IMUSOURCES}
	echo 'RCIO Sources' ${RCIOSOURCES}
	echo 'Sources' ${SOURCES}
	echo 'Objects' ${OBJECTS}
clean:
	echo ' ' > d.exe
	echo ' ' > d.o
	rm *.exe *.o
	echo ' ' > ${CPPDIR}MATLAB/d.o
	rm ${CPPDIR}MATLAB/*.o
	echo ' ' > ${CPPDIR}Mathp/d.o
	rm ${CPPDIR}Mathp/*.o
	echo ' ' > ${HPATH}/RCIO/d.o
	rm ${HPATH}/RCIO/*.o
	echo ' ' > ${HPATH}/IMU/d.o
	rm ${HPATH}/IMU/*.o
	echo ' ' > ${HPATH}/Util/d.o
	rm ${HPATH}/Util/*.o
	echo ' ' > ${CPPDIR}Timer/d.o
	rm ${CPPDIR}Timer/*.o
	echo ' ' > ${MODELPATH}${MODEL}/src/d.o
	rm ${MODELPATH}${MODEL}/src/*.o
rebuild:
	make clean
	make
