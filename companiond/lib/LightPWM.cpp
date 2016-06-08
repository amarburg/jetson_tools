
#include "LightPWM.h"
#include "I2C.h"

LightPWM::LightPWM( int fd, uint8_t addr )
	: _fd(fd), _addr(addr)
{;}


void LightPWM::set( uint8_t level )
{
	set_slave_addr( _fd, _addr );
	i2c_write_byte( _fd, level );
}
