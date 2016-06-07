
#include <string>
#include <iostream>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <mutex>
#include <thread>

#include "Adafruit_BMP280.h"

using namespace std;


int _fd;
std::mutex _mutex;
typedef std::lock_guard< std::mutex > LockGuard;
bool _stopping;


void sighandler( int sig )
{
	if( sig == SIGHUP || sig == SIGINT )
		_stopping = true;
}


void readTemperature( void )
{
	Adafruit_BMP280 bmp280( _fd );

	{
		LockGuard guard( _mutex );
		if( ! bmp280.initialize() ) {
			cerr << "Couldn't initiaize BMP280" << endl;
			return;
		}
	}

	while( !_stopping )
	{
			std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
			{
				LockGuard guard( _mutex );
				bmp280.read();
				cout << "Temperature is " << bmp280.temperature() << endl;
				cout << "Pressure is " << bmp280.pressure() << endl;
			}

			std::this_thread::sleep_until( now + std::chrono::seconds(1) );
	}
}


int main( int argc, char **argv )
{
	const string i2c_filename = "/dev/i2c-0";

	signal( SIGINT, sighandler );
	signal( SIGHUP, sighandler );


	int fd= -1;
	if ((fd = open(i2c_filename.c_str(), O_RDWR)) < 0) {
	    /* ERROR HANDLING: you can check errno to see what went wrong */
	    cerr << "Failed to open the i2c bus" << i2c_filename;
	    exit(1);
	}

	std::thread tempThread( readTemperature );

	while( !_stopping ) {

		std::this_thread::sleep_for( std::chrono::seconds(1) );
	}

	tempThread.join();

	if( fd > 0 ) close( fd );
}
