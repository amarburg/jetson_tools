#pragma once

#include <fstream>
#include <mutex>

#include <LightPWM.h>

typedef std::lock_guard< std::mutex > LockGuard;

extern int _fd;
extern std::mutex _mutex;
extern bool _stopping;

extern std::ofstream _logFile;


void temperatureThread( void );
