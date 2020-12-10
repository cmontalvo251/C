CC=g++
EXECUTABLE=FASTSim.exe
CSOURCES=$(wildcard *.c)
CPPSOURCES=$(wildcard *.cpp)
COBJECTS=$(CSOURCES:.c=.o)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(COBJECTS) $(CPPOBJECTS)
COMPILE=-c -w -O3
DIRECTIVES=#-DREALTIME
LIB= -L/usr/local/lib 

all: $(CSOURCES) $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB)
.c.o:
	$(CC) $(COMPILE) $(DIRECTIVES) $< -o $@ $(LIB)
.cpp.o:
	$(CC) $(COMPILE) $(DIRECTIVES) $< -o $@ $(LIB)
clean:
	rm -rf $(EXECUTABLE) *.o
rebuild:
	make clean
	make
