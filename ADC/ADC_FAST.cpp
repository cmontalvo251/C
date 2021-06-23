#include "ADC_FAST.h"

ADC_FAST::ADC_FAST() {
	printf("Initializing ADC \n");	
	#ifndef DESKTOP
	//On Desktop we don't need to initialize the channels
	converter.initialize();
	#else
	printf("Using DESKTOP configuration so no initialization required \n");
	#endif
	channel_count = converter.get_channel_count();
	printf("ADC Channel Count = %d \n",channel_count);
	//Now we create a MATLAB vector
	results.zeros(channel_count,1,"ADC Results");
}

void ADC_FAST::get_results() {
	for (int i = 0; i < channel_count; i++)
        {
            results.set(i+1,1,converter.read(i)/1000.0);
        }
}

void ADC_FAST::print_results() {
	printf(" ADC = ");
	for (int i = 0;i < channel_count;i++) {
		printf("%lf ",results.get(i+1,1));
	}
}

