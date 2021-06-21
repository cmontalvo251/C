#ifndef ADC_FAST_H
#define ADC_FAST_H

#include <Navio2/ADC_Navio2.h>
#include "MATLAB.h"

class ADC_FAST
{
	private:
		ADC_Navio2 converter;
		int channel_count;
	public:
		//Constructor
		ADC_FAST(); 
		void get_results();
		void print_results();
		MATLAB results;
};


#endif