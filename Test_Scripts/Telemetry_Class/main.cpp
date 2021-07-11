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
	return 0;
}