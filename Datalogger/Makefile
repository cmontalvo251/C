CC=g++
EXECUTABLE=Datalogger.exe
CPPSOURCES=$(wildcard *.cpp ../MATLAB/MATLAB.cpp ../mathp.cpp ../timer.cpp)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(CPPOBJECTS)
COMPILE=-c -w -O3 #-c means precompile, -w is no warnings -O3 all the optimizers to compile
LIB=-L../MATLAB -L../
INCLUDE=-I../MATLAB -I../

all: $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(INCLUDE) $(LIB)
.c.o:
	$(CC) $(COMPILE) $(FLAGS) $< -o $@ $(LIB) $(INCLUDE)
.cpp.o:
	$(CC) $(COMPILE) $(FLAGS) $< -o $@ $(LIB) $(INCLUDE)
clean:
	rm -rf $(EXECUTABLE) *.o
rebuild:
	make clean
	make
