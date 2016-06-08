
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
#include <iomanip>

#include <tclap/CmdLine.h>

#include "companion.h"

using namespace std;


int _fd;
std::mutex _mutex;
typedef std::lock_guard< std::mutex > LockGuard;
bool _stopping;
ofstream _logFile;


class CompanionConfig {
public:
	CompanionConfig( int argc, char **argv )
		: _cmd( "Companion", ' ', "0.1" ),
			_logFile("o","log-file","",false,"","", _cmd ),
			_noLights("d","no-lights","", _cmd, false),
			_autoLogFile("a","auto-log","", _cmd, false)
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

	bool				noLights( void ) { return _noLights.getValue(); }
	bool 				autoLog( void ) { return _autoLogFile.getValue(); }
protected:

	TCLAP::CmdLine _cmd;
	TCLAP::ValueArg< string > _logFile;
	TCLAP::SwitchArg _noLights;
	TCLAP::SwitchArg _autoLogFile;
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

	std::string logFileName( _conf.logFile() );

	if( _conf.autoLog() ) {
		char filename[80];
		std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
		std::time_t now_c = std::chrono::system_clock::to_time_t(now);

		strftime( filename, 79, "companion-%Y%m%d%H%M%S.log", localtime( &now_c ) );
		logFileName = filename;

		cout << "Saving to log file " << logFileName << endl;
	}

	if( !logFileName.empty() ) {
		_logFile.open(logFileName);

		if( !_logFile.is_open() ) {
			cerr << "Couldn't open log file " << logFileName << endl;
			exit(1);
		}
	}

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

		//std::this_thread::sleep_for( std::chrono::seconds(1) );
		int id = -1,  val = -1;
		char buf[80];
		cout << "light_id, value(0-255);  [q] to quit" << endl;
		cin.getline( buf, 79 );

		if( buf[0] == 'q' || buf[0] == 'Q' ) {
			_stopping = true;
			continue;
		}

		if( sscanf( buf, "%d %d", &id, &val ) != 2 ) continue;

		if( id <= 0  || id > _lights.size() ) {
			cerr << "Only light IDs 1 and 2 are acepted" << endl;
			continue;
		} else if( val < 0 || val > 255 ) {
				cerr << "Value should be between 0 and 255" << endl;
				continue;
		}

		cout << "Setting light " << id << " to " << val << endl;
		if( ! _conf.noLights() ) _lights[id-1].set( val );
	}

	tempThread.join();

	if( _logFile.is_open() ) _logFile.close();

	_lights[0].set( 0 );
	_lights[1].set( 0 );

	if( _fd > 0 ) close( _fd );

	exit(0);
}
