#ifndef ADC_FAST_H
#define ADC_FAST_H

#include <ADC/ADC_Navio2.h>
#include <MATLAB/MATLAB.h>

class ADC_FAST
{
	private:
		ADC_Navio2 converter;		
	public:
		//Constructor
		ADC_FAST(); 
		void get_results();
		void print_results();
		int channel_count;
		MATLAB results;
};


#endif
