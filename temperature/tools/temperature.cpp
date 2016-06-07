
#include <string>
#include <iostream>
#include <vector>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <libudev.h>

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


struct ThermalZoneUdev {
	ThermalZoneUdev( struct udev_device *d )
		: _dev( d ) {;}

	~ThermalZoneUdev()
		{ udev_device_unref(_dev); }

	const char *name( void ) { return udev_device_get_sysattr_value(_dev,"type"); }
	float temperature( void ) {
		const char *t = udev_device_get_sysattr_value(_dev,"temp");
		if( !t ) return -1;

		return atof(t)/1000.0;
	}

	struct udev_device *_dev;
};


void readTemperature( void )
{
	Adafruit_BMP280 bmp280( _fd );

	{
		LockGuard guard( _mutex );
		if( ! bmp280.initialize() ) {
			cerr << "Couldn't initialize BMP280" << endl;
			return;
		}
	}

	vector< struct ThermalZoneUdev > zones;

	// And initialize libudev
	struct udev *udev;
		struct udev_enumerate *enumerate;
		struct udev_list_entry *devices, *dev_list_entry;
		struct udev_device *dev;

		/* Create the udev object */
		udev = udev_new();
		if (!udev) {
			cerr << "Can't create udev";
			return;
		}

		enumerate = udev_enumerate_new(udev);
			udev_enumerate_add_match_subsystem(enumerate, "thermal");
			udev_enumerate_scan_devices(enumerate);
			devices = udev_enumerate_get_list_entry(enumerate);
			/* For each item enumerated, print out its information.
			   udev_list_entry_foreach is a macro which expands to
			   a loop. The loop will be executed for each member in
			   devices, setting dev_list_entry to a list entry
			   which contains the device's path in /sys. */
			udev_list_entry_foreach(dev_list_entry, devices) {
				const char *path;
				/* Get the filename of the /sys entry for the device
				   and create a udev_device object (dev) representing it */
				path = udev_list_entry_get_name(dev_list_entry);
				dev = udev_device_new_from_syspath(udev, path);

				/* usb_device_get_devnode() returns the path to the device node
				   itself in /dev. */
				//printf("Device Node Path: %s\n", udev_device_get_devnode(dev));

			// Only thermal devices have temp
			if( !udev_device_get_sysattr_value(dev, "temp") ) {
				udev_device_unref(dev);
				continue;
			}

			zones.emplace_back( dev );

				// printf("  type/temp: %s %s\n",
				// 		        udev_device_get_sysattr_value(dev,"type"),
				// 		        udev_device_get_sysattr_value(dev, "temp"));

	}
	udev_enumerate_unref(enumerate);

	while( !_stopping )
	{
			std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
			{
				LockGuard guard( _mutex );
				bmp280.read();
				cout << "Temperature is " << bmp280.temperature() << endl;
				cout << "Pressure is " << bmp280.pressure() << endl;
			}

			for( auto &zone : zones ) {
				cout << zone.name() << ": " << zone.temperature() << endl;
			}

			std::this_thread::sleep_until( now + std::chrono::seconds(1) );

	}

	udev_unref(udev);
}


int main( int argc, char **argv )
{
	const string i2c_filename = "/dev/i2c-0";

	signal( SIGINT, sighandler );
	signal( SIGHUP, sighandler );


	if ((_fd = open(i2c_filename.c_str(), O_RDWR)) < 0) {
	    /* ERROR HANDLING: you can check errno to see what went wrong */
	    cerr << "Failed to open the i2c bus" << i2c_filename;
	    exit(1);
	}

	std::thread tempThread( readTemperature );

	while( !_stopping ) {

		std::this_thread::sleep_for( std::chrono::seconds(1) );
	}

	tempThread.join();

	if( _fd > 0 ) close( _fd );
}
