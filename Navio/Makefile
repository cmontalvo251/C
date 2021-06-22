CXX = g++
CFLAGS = -std=c++11 -Wno-psabi -w -c -I .
LIB=-L../ -L../Mathp/
INCLUDE=-I../ -I../Mathp/
SRC=$(wildcard */*.cpp)
OBJECTS = $(SRC:.cpp=.o)

%.o: %.cpp
	$(CXX) $(CFLAGS) $(LIB) $(INCLUDE) -o $@ $<

all: $(OBJECTS)
	ar rcs libnavio.a $(OBJECTS)

clean:
	rm -f */*.o *.a
