#include "24XX32A.h"

void byte_write(uint16_t address, uint8_t data) {
	if(address > _24XX32A_MAX_ADDR) return;
	while(!i2c_is_available(_24XX32A_I2C, _24XX32A_I2C_ADDR));
	uint8_t wr_buffer[3];
	wr_buffer[0] = (address >> 8) & 0xff;
	wr_buffer[1] = address & 0xff;
	wr_buffer[2] = data;
	i2c_transfer7(_24XX32A_I2C, _24XX32A_I2C_ADDR, wr_buffer, 3, NULL, 0);
}

void current_read(uint8_t *data) {
	while(!i2c_is_available(_24XX32A_I2C, _24XX32A_I2C_ADDR));
	i2c_transfer7(_24XX32A_I2C, _24XX32A_I2C_ADDR, NULL, 0, data, 1);	
}

void random_read(uint16_t address, uint8_t *data, uint8_t n) {
	if(address > _24XX32A_MAX_ADDR) return;
	while(!i2c_is_available(_24XX32A_I2C, _24XX32A_I2C_ADDR));
	uint8_t wr_buffer[2];
	wr_buffer[0] = (address >> 8) & 0xff;
	wr_buffer[1] = address & 0xff;	
	i2c_transfer7(_24XX32A_I2C, _24XX32A_I2C_ADDR, wr_buffer, 2, data, n);	
}
