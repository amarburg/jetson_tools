#pragma once

#include <stdint.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

int set_slave_addr(int file, uint8_t address );

uint8_t i2c_read_byte_data(int file, uint8_t command);
uint8_t i2c_write_byte(int file, uint8_t value);
uint8_t i2c_write_byte_data(int file, uint8_t command, uint8_t value);

__s32 i2c_read_word_data(int file, uint8_t command);
__s32 i2c_read_i2c_block_data(int file, uint8_t command, uint8_t length, uint8_t *values);
