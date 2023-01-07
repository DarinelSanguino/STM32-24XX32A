#include "24XX32A.h"
#include <string.h>

/**********************************************************************
* @brief byte_write() writes one byte of data to memory
* @param address Memory location to write
* @param data Byte to be saved in memory
**********************************************************************/
void byte_write(uint16_t address, uint8_t data) {
	if(address > _24XX32A_MAX_ADDR) return;
	while(!i2c_is_available(_24XX32A_I2C, _24XX32A_I2C_ADDR));
	uint8_t wr_buffer[3];
	wr_buffer[0] = (address >> 8) & 0xff;
	wr_buffer[1] = address & 0xff;
	wr_buffer[2] = data;
	i2c_transfer7(_24XX32A_I2C, _24XX32A_I2C_ADDR, wr_buffer, 3, NULL, 0);
}

/**********************************************************************
* @brief page_write() writes up to 32 bytes of data to memory
* @param address First memory location to write
* @param data Pointer to array of bytes to be saved in memory
* @param n Number of bytes to write
**********************************************************************/
void page_write(uint16_t address, uint8_t *data, uint8_t n) {
	if(address + n > _24XX32A_MAX_ADDR) return;
	while(n > 0) {
		while(!i2c_is_available(_24XX32A_I2C, _24XX32A_I2C_ADDR));
		uint8_t wr_buffer[_24XX32A_PAGE_SIZE + 2];
		wr_buffer[0] = (address >> 8) & 0xff;
		wr_buffer[1] = address & 0xff;
		//Remaining bytes in current page
		uint8_t rem_bytes = _24XX32A_PAGE_SIZE - (address % _24XX32A_PAGE_SIZE);
		uint8_t n_next_bytes = rem_bytes > n ? n : rem_bytes;
		memcpy(wr_buffer + 2, data, n_next_bytes);
		i2c_transfer7(_24XX32A_I2C, _24XX32A_I2C_ADDR, wr_buffer, n_next_bytes + 2, NULL, 0);
		n -= n_next_bytes;
		data += n_next_bytes;
		address += n_next_bytes;
	}
}

/************************************************************************************
* @brief current_read() reads one byte of data from memory address in address counter
* @param data buffer to save read byte
************************************************************************************/
void current_read(uint8_t *data) {
	while(!i2c_is_available(_24XX32A_I2C, _24XX32A_I2C_ADDR));
	i2c_transfer7(_24XX32A_I2C, _24XX32A_I2C_ADDR, NULL, 0, data, 1);	
}

/*****************************************************************************
* @brief random_read() reads bytes starting from specified memory address
* @param data buffer to save read bytes
* @param n Number of bytes to read
*****************************************************************************/
void random_read(uint16_t address, uint8_t *data, uint8_t n) {
	if(address > _24XX32A_MAX_ADDR) return;
	while(!i2c_is_available(_24XX32A_I2C, _24XX32A_I2C_ADDR));
	uint8_t wr_buffer[2];
	wr_buffer[0] = (address >> 8) & 0xff;
	wr_buffer[1] = address & 0xff;	
	i2c_transfer7(_24XX32A_I2C, _24XX32A_I2C_ADDR, wr_buffer, 2, data, n);	
}
