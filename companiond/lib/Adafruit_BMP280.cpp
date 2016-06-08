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

#include <math.h>

#include <iostream>
using namespace std;


#include "Adafruit_BMP280.h"
#include "I2C.h"

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

Adafruit_BMP280::Adafruit_BMP280( int fd, uint8_t addr )
  : _fd( fd ), _i2caddr( addr )
{ }

bool Adafruit_BMP280::initialize()
{
	uint8_t chipid = read8( BMP280_REGISTER_CHIPID );
	cout << "Read chipid: " << std::hex << int(chipid) << std::dec << "(" << int(chipid) << ")" << endl;
  if ( chipid != 0x58 )
    return false;

  readCoefficients();
  write8( BMP280_REGISTER_CONTROL, 0x3F);
  return true;
}


uint8_t Adafruit_BMP280::read8(uint8_t reg)
{
	set_slave_addr( _fd, BMP280_ADDRESS );
	return i2c_read_byte_data( _fd, reg );
}

void Adafruit_BMP280::write8(uint8_t reg, uint8_t data )
{
	set_slave_addr( _fd, BMP280_ADDRESS );
	i2c_write_byte_data( _fd, reg, data );
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
uint16_t Adafruit_BMP280::read16(uint8_t reg)
{
	set_slave_addr( _fd, BMP280_ADDRESS );
	return i2c_read_word_data( _fd, reg );
}

uint16_t Adafruit_BMP280::read16_LE(uint8_t reg) {
	return read16(reg);
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
	set_slave_addr( _fd, BMP280_ADDRESS );

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
		// Interestingly, the calibration data is stored as little-endian words,
		// while the T and P data is big-endian 20-bits (in a 24bits of register)
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
	// cout << "adc_T: " << adc_T << endl;

  var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
	   ((int32_t)_bmp280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
	     ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
	   ((int32_t)_bmp280_calib.dig_T3)) >> 14;

  _t_fine = var1 + var2;
  _temperature = float( (_t_fine * 5 + 128) >> 8 ) / 100;
}

/**************************************************************************/
/*!  Queries hardware for both temperature and pressure values.
*/
/**************************************************************************/
void Adafruit_BMP280::read(void) {
  int64_t var1, var2, p;

	_pressure = _temperature = 0.0;

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

	// cout << "adc_p: " << adc_P << endl;

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
