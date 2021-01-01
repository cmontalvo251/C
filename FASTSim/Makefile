CC=g++
EXECUTABLE=FASTSim.exe
CSOURCES=$(wildcard *.c)
CPPSOURCES=$(wildcard *.cpp)
COBJECTS=$(CSOURCES:.c=.o)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(COBJECTS) $(CPPOBJECTS)
COMPILE=-c -w -O3
TIME=#-DREALTIME
HARDWARE=-DDESKTOP
VEHICLE=-DPOINTSPACE
FLAGS=-DDEBUG
LIB= -L/usr/local/lib 

all: $(CSOURCES) $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB)
.c.o:
	$(CC) $(COMPILE) $(FLAGS) $(TIME) $(HARDWARE) $(VEHICLE) $< -o $@ $(LIB)
.cpp.o:
	$(CC) $(COMPILE) $(FLAGS) $(TIME) $(HARDWARE) $(VEHICLE) $< -o $@ $(LIB)
clean:
	rm -rf $(EXECUTABLE) *.o
rebuild:
	make clean
	make
