CC=g++ #Define compiler (CC) to be gfortran compiler
CFLAGS=-c -w #-O1 #-g for debug mode
DFLAGS= #-DRUN_VISUALIZER # Directives for #ifdefs
LIB_DIRS = -L../../ #-L/usr/local/lib #used to include boost and glut directories
BOOST_DIRS = #-lpthread -lboost_system -lboost_thread -lboost_date_time
OPENGL_DIRS = #-lGL -lGLU -lglut
SOURCES=main.cpp ../../timer.cpp 
#This means all the objects are the sources except replace .cpp with .o
OBJECTS = $(SOURCES:.cpp=.o)
INCLUDES=-I../../
EXECUTABLE=Run.exe

##Tells make to compile all the sources and executable
all: $(SOURCES) $(EXECUTABLE)

#Rule for executable, will only do this if the objects have changed
#The $@ symbol tells the makefile to compile the thing in blue
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(INCLUDES) $(LDFLAGS) $(OBJECTS) -o $@ $(LIB_DIRS) $(BOOST_DIRS) $(OPENGL_DIRS)

#rule for all of the .cpp files 
.cpp.o:
	$(CC) $(INCLUDES) $(CFLAGS) $(DFLAGS) $< -o $@
clean:
	rm -rf *.o $(EXECUTABLE)
rebuild:
	make clean
	make
