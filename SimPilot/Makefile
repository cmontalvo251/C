#####################WHERE IS HIL LOCATED?#####################
#HPATH=/home/mjcobar13/Documents/GIT/CMontalvo251/HIL
HPATH=/home/carlos/Git_Repos/Github/HIL
#HPATH=/home/carlos/Files/Git_Repos/Github/HIL
#HPATH=/home/pi/HIL
##########################################################

#########################WHAT MODE ARE YOU RUNNING IN?#############
EXETYPE=-DAUTO
########################################################

###############################HARDWARE#######################
HARDWARE=-DRPI
########################################################

#####################WHAT TYPE OF TRANSMITTER ARE YOU USING?###############
CONTROLLER=-DFLYSKY
#######################################################################

CC=g++
EXECUTABLE=SimPilot.exe
CPPSOURCES=$(wildcard *.cpp ../Timer/timer.cpp ../MATLAB/MATLAB.cpp ${HPATH}/RCIO/RCInput.cpp ../Mathp/mathp.cpp ${HPATH}/IMU/IMU.cpp ${HPATH}/IMU/AHRS.cpp ${HPATH}/IMU/MPU9250.cpp ${HPATH}/IMU/LSM9DS1.cpp)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(CPPOBJECTS)
COMPILE=-c -w -O3
LIB=-L/usr/local/lib
INCLUDE=-I../ -I${HPATH}

all: $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB) $(INCLUDE)
.cpp.o:
	$(CC) $(COMPILE) $(EXETYPE) $(HARDWARE) $(CONTROLLER) $< -o $@ $(LIB) $(INCLUDE) $(RENDER) $(THREAD)
clean:
	echo ' ' > d.exe
	echo ' ' > d.o
	rm *.exe *.o
	echo ' ' > ../MATLAB/d.o
	rm ../MATLAB/*.o
	echo ' ' > ../Mathp/d.o
	rm ../Mathp/*.o
	echo ' ' > ${HPATH}/RCIO/d.o
	rm ${HPATH}/RCIO/*.o
	echo ' ' > ${HPATH}/IMU/d.o
	rm ${HPATH}/IMU/*.o
	echo ' ' > ../Timer/d.o
	rm ../Timer/*.o
rebuild:
	make clean
	make
