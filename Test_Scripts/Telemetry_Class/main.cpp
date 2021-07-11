#include <Serial/Telemetry.h>
#include <stdio.h>

using namespace std;

int main(int argc, char** argv) {
	printf("Testing Telemetry \n");
	printf("Opening Serial Port ttyACM0 \n");
	Telemetry serial;
	serial.SerialInit("/dev/ttyACM0",115200);
	printf("If no errors present, serial port is open \n");
	serial.SerialPutHello();
	float number_array[7];
	number_array[0] = 3.4;
	number_array[1] = -2.3;
	number_array[2] = 0.4;
	number_array[3] = -0.1;
	number_array[4] = 5.8;
	number_array[5] = 300.0;
	number_array[6] = -300.0;
	serial.SerialSendArray(number_array,7);
	return 0;
}