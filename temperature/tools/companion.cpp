
#include <string>
#include <iostream>
#include <vector>
#include <fstream>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


#include <mutex>
#include <thread>

#include <tclap/CmdLine.h>

#include "companion.h"

using namespace std;


int _fd;
std::mutex _mutex;
typedef std::lock_guard< std::mutex > LockGuard;
bool _stopping;
ostream *_temperatureOut;


class CompanionConfig {
public:
	CompanionConfig( int argc, char **argv )
		: _cmd( "Companion", ' ', "0.1" ),
			_logFile("o","log-file","",false,"","", _cmd )
	{ parseCmdLine( argc, argv ); }

	void parseCmdLine( int argc, char **argv )
	{
		try {
			_cmd.parse(argc, argv );
		}
		catch (TCLAP::ArgException &e)  // catch any exceptions
		{
			cerr << "error: " << e.error() << " for arg " << e.argId();
			exit(-1);
		}
	}

	std::string logFile( void ) { return _logFile.getValue(); }
	bool 				logFileSet( void ) { return _logFile.isSet(); }
protected:

	TCLAP::CmdLine _cmd;
	TCLAP::ValueArg< string > _logFile;
};

void sighandler( int sig )
{
	if( sig == SIGHUP || sig == SIGINT )
		_stopping = true;
}



int main( int argc, char **argv )
{
	CompanionConfig _conf( argc, argv );

	std::streambuf * buf;
	std::ofstream of;

	if( _conf.logFileSet() ) {
		    of.open(_conf.logFile());

				if( !of.is_open() ) {
					cerr << "Couldn't open log file " << _conf.logFile() << endl;
					exit(1);
				}

		    buf = of.rdbuf();
		} else {
		    buf = std::cout.rdbuf();
		}

		_temperatureOut = new ostream(buf);

	const string i2c_filename = "/dev/i2c-0";

	signal( SIGINT, sighandler );
	signal( SIGHUP, sighandler );

	if ((_fd = open(i2c_filename.c_str(), O_RDWR)) < 0) {
	    /* ERROR HANDLING: you can check errno to see what went wrong */
	    cerr << "Failed to open the i2c bus" << i2c_filename;
	    exit(1);
	}

 std::vector<LightPWM> _lights;
	_lights.emplace_back( _fd, 0x12 );
	_lights.emplace_back( _fd, 0x13 );

	std::thread tempThread( temperatureThread );

	while( !_stopping ) {

		std::this_thread::sleep_for( std::chrono::seconds(1) );
	}

	tempThread.join();

	if( _fd > 0 ) close( _fd );

	delete _temperatureOut;
}
