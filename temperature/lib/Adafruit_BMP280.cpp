/* Modified from Adafruit's BMP280 support library:
 *
 * https://github.com/adafruit/Adafruit_BMP280_Library
 */
/***************************************************************************
  This is a library for the BMP280 pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
// #include "Arduino.h"
// #include <Wire.h>
// #include <SPI.h>

#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <string.h>

#include <iostream>
using namespace std;

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "Adafruit_BMP280.h"

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
union i2c_smbus_data {
	__u8 byte;
	__u16 word;
	__u8 block[I2C_SMBUS_BLOCK_MAX + 2]; /* block[0] is used for length */
	                                            /* and one more for PEC */
};

static int set_slave_addr(int file, int address )
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

static inline __s32 i2c_access(int file, char read_write, __u8 command,
                                       int size, union i2c_smbus_data *data)
  {
      struct i2c_smbus_ioctl_data args;

      args.read_write = read_write;
      args.command = command;
      args.size = size;
      args.data = data;
      return ioctl(file,I2C_SMBUS,&args);
  }

static inline uint8_t i2c_read_byte_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (i2c_access(file,I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data))
		return -1;
	else
		return 0x0FF & data.byte;
}

static inline uint8_t i2c_write_byte_data(int file, __u8 command, __u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_access(file,I2C_SMBUS_WRITE,command,
		I2C_SMBUS_BYTE_DATA, &data);
}

static inline __s32 i2c_read_word_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (i2c_access(file,I2C_SMBUS_READ,command,
	                    I2C_SMBUS_WORD_DATA,&data))
	   return -1;
	else
	   return 0x0FFFF & data.word;
}

static inline __s32 i2c_read_i2c_block_data(int file, __u8 command,
                                                    __u8 length, __u8 *values)
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
	    for (i = 1; i <= data.block[0]; i++)
	        values[i-1] = data.block[i];
	    return data.block[0];
	}
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

Adafruit_BMP280::Adafruit_BMP280( int fd, uint8_t addr )
  : _fd( fd ), _i2caddr( addr )
{ }

bool Adafruit_BMP280::initialize()
{
	set_slave_addr( _fd, BMP280_ADDRESS );
	uint8_t chipid = i2c_read_byte_data( _fd, BMP280_REGISTER_CHIPID );
	cout << "Read chipid " << ios::hex << chipid << endl;
  if ( chipid != 0x58 )
    return false;

  readCoefficients();
  i2c_write_byte_data( _fd, BMP280_REGISTER_CONTROL, 0x3F);
  return true;
}


/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
uint16_t Adafruit_BMP280::read16(uint8_t reg)
{
	return i2c_read_word_data( _fd, reg );
}

uint16_t Adafruit_BMP280::read16_LE(uint8_t reg) {
	uint16_t temp = read16(reg);
	return (temp >> 8) | (temp << 8);

}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/
int16_t Adafruit_BMP280::readS16(uint8_t reg)
{
  return (int16_t)read16(reg);

}

int16_t Adafruit_BMP280::readS16_LE(uint8_t reg)
{
  return (int16_t)read16_LE(reg);

}


/**************************************************************************/
/*!
    @brief
*/
/**************************************************************************/

uint32_t Adafruit_BMP280::read24(uint8_t reg)
{
	uint8_t values[3] = {0,0,0};
	if( i2c_read_i2c_block_data( _fd, reg, 3, values ) < 0 ) {
		return 0;
	}

	return values[0] << 16 | values[1] << 8 | values[2];
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void Adafruit_BMP280::readCoefficients(void)
{
    _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);

		cout << "dig_T1: " << _bmp280_calib.dig_T1 << endl;
		cout << "dig_T2: " << _bmp280_calib.dig_T2 << endl;
		cout << "dig_T3: " << _bmp280_calib.dig_T3 << endl;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
float Adafruit_BMP280::readTemperature(void)
{
  int32_t var1, var2;

  int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);

	cout << "adc_T: " << adc_T << endl;

  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
	   ((int32_t)_bmp280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
	     ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
	   ((int32_t)_bmp280_calib.dig_T3)) >> 14;

  _t_fine = var1 + var2;
  _temperature = ( (_t_fine * 5 + 128) >> 8 ) / 100;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_BMP280::read(void) {
  int64_t var1, var2, p;

	_pressure = _temperature = 0.0;

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
	cout << "adc_p: " << adc_P << endl;

  adc_P >>= 4;

  var1 = ((int64_t)_t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

  if (var1 == 0) {
    return;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
  _pressure = (float)p/256;
}

float Adafruit_BMP280::readAltitude(float seaLevelhPa) {
  float altitude;

  float pressure = _pressure/100; // Convert to Pa

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}
