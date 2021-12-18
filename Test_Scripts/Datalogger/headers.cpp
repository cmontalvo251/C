#include <iostream> //these are standard includes
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
using namespace std;


int main() {
	printf("Create Array \n");
	char** array;
	int num = 2;
	printf("Dynamically Allocate Array\n");
	array = (char**)malloc(num*sizeof(char*));
	printf("Save First 2 Arrays\n");
	array[0] = "Hello";
	array[1] = "World";
	printf("Print Array\n");
	for (int i = 0;i<num;i++) {
		printf("%s \n",array[i]);
	}
}