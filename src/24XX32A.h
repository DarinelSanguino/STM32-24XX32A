#include <libopencm3/stm32/i2c.h>

#ifndef _24XXC32A_I2C
#define _24XX32A_I2C 			I2C1
#endif
#define _24XX32A_I2C_ADDR		80
#define _24XX32A_PAGE_SIZE		32
#define _24XX32A_MAX_ADDR		0xFFF

void byte_write(uint16_t address, uint8_t data);
void page_write(uint16_t address, uint8_t *data, uint8_t n);
void current_read(uint8_t *data);
void random_read(uint16_t address, uint8_t *data, uint8_t n);
