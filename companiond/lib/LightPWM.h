#pragma once

#include <stdint.h>

class LightPWM {
public:

	LightPWM( int fd, uint8_t addr );

	void set( uint8_t level );

protected:

	int _fd;
	uint8_t _addr;
};
