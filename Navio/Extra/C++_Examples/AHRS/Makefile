CXX ?= g++
NAVIO = ../../../
INCLUDES = -I ../../../

all:
	$(MAKE) -C ../../../ all
	$(CXX) -std=c++11 $(INCLUDES) AHRS.cpp -L$(NAVIO) -lnavio -o AHRS -lrt -lpthread

clean:
	rm -f AHRS
	rm ../../../Common/*.o
	rm ../../../Navio2/*.o
