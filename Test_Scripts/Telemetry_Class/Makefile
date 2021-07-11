CC=g++
EXECUTABLE=telemetry.exe
CPPSOURCES=$(wildcard *.cpp ../../Serial/Telemetry.cpp)
CPPOBJECTS=$(CPPSOURCES:.cpp=.o)
OBJECTS=$(CPPOBJECTS)
COMPILE=-c -w -O3
DIRECTIVES=
LIB=-L/usr/local/lib
INCLUDE=-I../../

all: $(CPPSOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIB)
.cpp.o:
	$(CC) $(COMPILE) $(DIRECTIVES) $< -o $@ $(LIB) $(INCLUDE)
clean:
	rm ../../Serial/*.o
	rm -rf $(EXECUTABLE) *.o
rebuild:
	make clean
	make
