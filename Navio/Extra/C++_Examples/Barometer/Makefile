CXX ?= g++
NAVIO =../../../
INCLUDES = -I${NAVIO}

all:
	$(MAKE) -C ${NAVIO} all
	$(CXX) $(INCLUDES) Barometer.cpp -L$(NAVIO) -lnavio -o Barometer

clean:
	rm -f Barometer
