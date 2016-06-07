
#include <string>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


#include "Adafruit_BMP280.h"

using namespace std;

int main( int argc, char **argv )
{
	const string i2c_filename = "/dev/i2c-0";

	int fd= -1;
	if ((fd = open(i2c_filename.c_str(), O_RDWR)) < 0) {
	    /* ERROR HANDLING: you can check errno to see what went wrong */
	    cerr << "Failed to open the i2c bus" << i2c_filename;
	    exit(1);
	}

Adafruit_BMP280 bmp280( fd );

	if( ! bmp280.initialize() ) {
		cerr << "Couldn't initiaize BMP280" << endl;
		exit(1);
	}

	float t = bmp280.readTemperature();
	cout << "Temperature is " << t << endl;

	float p = bmp280.readPressure();
	cout << "Pressure is " << p << endl;


	if( fd > 0 ) close( fd );
}
