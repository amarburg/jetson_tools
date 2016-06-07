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

#define I2C_SMBUS_BLOCK_MAX 32
union i2c_smbus_data {
	__u8 byte;
	__u16 word;
	__u8 block[I2C_SMBUS_BLOCK_MAX + 2]; /* block[0] is used for length */
	                                            /* and one more for PEC */
};


/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/


Adafruit_BMP280::Adafruit_BMP280( int fd, uint8_t addr )
  : _fd( fd ), _i2caddr( addr )
{ }

bool Adafruit_BMP280::initialize()
{
	uint8_t chipid = read8(BMP280_REGISTER_CHIPID);
	cout << "Read chipid " << ios::hex << int(chipid) << endl;
  if ( chipid != 0x58)
    return false;

  readCoefficients();
  write8(BMP280_REGISTER_CONTROL, 0x3F);
  return true;
}

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command,
                                       int size, union i2c_smbus_data *data)
  {
      struct i2c_smbus_ioctl_data args;

      args.read_write = read_write;
      args.command = command;
      args.size = size;
      args.data = data;
      return ioctl(file,I2C_SMBUS,&args);
  }

	static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
	{
		union i2c_smbus_data data;
		if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
			I2C_SMBUS_BYTE_DATA,&data))
			return -1;
			else
			return 0x0FF & data.byte;
		}

		static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command,
			__u8 value)
			{
				union i2c_smbus_data data;
				data.byte = value;
				return i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
					I2C_SMBUS_BYTE_DATA, &data);
				}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
void Adafruit_BMP280::write8(uint8_t reg, uint8_t value)
{
 i2c_smbus_write_byte_data( _fd, reg, value );

	// uint8_t buf[2] = {reg, value};
	// if (ioctl(_fd, I2C_SLAVE, _i2caddr) < 0) {
  //   cerr << "Failed to acquire bus access and/or talk to slave." << endl;
  //   /* ERROR HANDLING; you can check errno to see what went wrong */
	// }
	//
	// if (write(_fd,buf,2) != 1) {
	// 	cerr << "Unable to write buffer" << endl;
	// }
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/

uint8_t Adafruit_BMP280::read8(uint8_t reg)
{
return i2c_smbus_read_byte_data( _fd, reg ) & 0xFF;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
uint16_t Adafruit_BMP280::read16(uint8_t reg)
{
  uint16_t value;

	if (ioctl(_fd, I2C_SLAVE, _i2caddr) < 0) {
    cerr << "Failed to acquire bus access and/or talk to slave." << endl;
    /* ERROR HANDLING; you can check errno to see what went wrong */
	}

	if ( write(_fd, (void *)reg,1) != 1) {
		cerr << "Unable to read from buffer" << endl;
	}

		// Watch for MSB/LSB problems here
	if( read( _fd, (void *)value, 2 ) != 1 ) {
		cerr << "Unable to read from buffer." << endl;
	}

  return value;
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
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/

uint32_t Adafruit_BMP280::read24(uint8_t reg)
{
	uint8_t b;
  uint8_t in[3] = {0,0,0};

	if (ioctl(_fd, I2C_SLAVE, _i2caddr) < 0) {
    cerr << "Failed to acquire bus access and/or talk to slave.";
    /* ERROR HANDLING; you can check errno to see what went wrong */
	}

	if (write( _fd, (void *)reg, 1) != 1) {
		cerr << "Unable to read from buffer" << endl;
	}

	// watch for byte ordering problems here
	if( read( _fd, (void *)in, 3 ) != 1 ) {
		cerr << "Unable to read from buffer." << endl;
	}

  return in[2] << 16 | in[1] << 8 | in[0];
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
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
float Adafruit_BMP280::readTemperature(void)
{
  int32_t var1, var2;

  int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
	   ((int32_t)_bmp280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
	     ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
	   ((int32_t)_bmp280_calib.dig_T3)) >> 14;

  t_fine = var1 + var2;

  float T  = (t_fine * 5 + 128) >> 8;
  return T/100;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
float Adafruit_BMP280::readPressure(void) {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
  return (float)p/256;
}

float Adafruit_BMP280::readAltitude(float seaLevelhPa) {
  float altitude;

  float pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}
