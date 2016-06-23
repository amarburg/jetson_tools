#include "I2C.h"

#include <iostream>
using namespace std;

#include <errno.h>
#include <string.h>

//== From i2ctools
/* smbus_access read or write markers */
#define I2C_SMBUS_READ  1
#define I2C_SMBUS_WRITE 0

/* SMBus transaction types (size parameter in the above functions)
101    Note: these no longer correspond to the (arbitrary) PIIX4 internal codes! */
#define I2C_SMBUS_QUICK         0
#define I2C_SMBUS_BYTE          1
#define I2C_SMBUS_BYTE_DATA     2
#define I2C_SMBUS_WORD_DATA     3
#define I2C_SMBUS_PROC_CALL     4
#define I2C_SMBUS_BLOCK_DATA        5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7       /* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8

#define I2C_SMBUS_BLOCK_MAX 32

// union i2c_smbus_data {
// 	uint8_t byte;
// 	__u16 word;
// 	uint8_t block[I2C_SMBUS_BLOCK_MAX + 2]; /* block[0] is used for length */
// 	                                            /* and one more for PEC */
// };

int set_slave_addr(int file, uint8_t address )
{
	int force = 0;

	/* With force, let the user read from/write to the registers
	  even when a driver is also running */
	if (ioctl(file, force ? I2C_SLAVE_FORCE : I2C_SLAVE, address) < 0) {
	   cerr << "Error: Could not set address: " << strerror(errno) << endl;
	   return -errno;
	}

	return 0;
}

static inline int32_t i2c_access(int file, char read_write, uint8_t command,
                                       int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}

uint8_t i2c_write_byte(int file, uint8_t value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_access(file,I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, NULL);
}

uint8_t i2c_read_byte_data(int file, uint8_t command)
{
	union i2c_smbus_data data;
	if (i2c_access(file,I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data))
		return -1;
	else
		return 0x0FF & data.byte;
}

uint8_t i2c_write_byte_data(int file, uint8_t command, uint8_t value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_access(file,I2C_SMBUS_WRITE,command, I2C_SMBUS_BYTE_DATA, &data);
}

int32_t i2c_read_word_data(int file, uint8_t command)
{
	union i2c_smbus_data data;
	if (i2c_access(file,I2C_SMBUS_READ,command, I2C_SMBUS_WORD_DATA,&data))
	   return -1;
	else
	   return 0x0FFFF & data.word;
}

int32_t i2c_read_i2c_block_data(int file, uint8_t command, uint8_t length, uint8_t *values)
{
	union i2c_smbus_data data;
	int i;

	if (length > 32)
	    length = 32;
	data.block[0] = length;
	if (i2c_access(file,I2C_SMBUS_READ,command,
										length == 32 ? I2C_SMBUS_I2C_BLOCK_BROKEN :
										I2C_SMBUS_I2C_BLOCK_DATA,&data))
		return -1;
	else {
		for (i = 1; i <= data.block[0]; i++) values[i-1] = data.block[i];
		return data.block[0];
	}
}
