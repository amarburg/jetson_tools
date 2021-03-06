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
#ifndef __BMP280_H__
#define __BMP280_H__

#include <stdint.h>

// #if (ARDUINO >= 100)
//  #include "Arduino.h"
// #else
//  #include "WProgram.h"
// #endif
//
// #include <Adafruit_Sensor.h>
//
// #ifdef __AVR_ATtiny85__
//  #include "TinyWireM.h"
//  #define Wire TinyWireM
// #else
//  #include <Wire.h>
// #endif

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define BMP280_ADDRESS                (0x76)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
    } bmp280_calib_data;
/*=========================================================================*/

/*
class Adafruit_BMP280_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_BMP280_Unified(int32_t sensorID = -1);

    bool  begin(uint8_t addr = BMP280_ADDRESS);
    void  getTemperature(float *temp);
    void  getPressure(float *pressure);
    float pressureToAltitude(float seaLevel, float atmospheric, float temp);
    float seaLevelForAltitude(float altitude, float atmospheric, float temp);
    void  getEvent(sensors_event_t*);
    void  getSensor(sensor_t*);

  private:
    uint8_t   _i2c_addr;
    int32_t   _sensorID;
};

*/

class Adafruit_BMP280
{
  public:
    Adafruit_BMP280( int fd, uint8_t addr = BMP280_ADDRESS );

    bool  initialize();

    void read(void);

		// Reads temperature in degrees C
		float temperature( void ) const { return _temperature; }

		// Reads pressure in hPa (expect ~1000 +/- 50)
		float pressure( void ) const { return _pressure; }

    float readAltitude(float seaLevelhPa = 1013.25);

  private:

    void readCoefficients(void);

    void      write8(uint8_t reg, uint8_t value);
    uint8_t   read8(uint8_t reg);

    uint16_t  read16(uint8_t reg);
    uint32_t  read24(uint8_t reg);
    int16_t   readS16(uint8_t reg);

		// Intel and Jetson architecture are naturally LE,
		// these are trivial
    uint16_t  read16_LE(uint8_t reg); // little endian
    int16_t   readS16_LE(uint8_t reg); // little endian

    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int32_t _t_fine;

		float readTemperature(void);

		float _temperature, _pressure;

		int _fd;

    bmp280_calib_data _bmp280_calib;

};

#endif
