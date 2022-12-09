#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include "24XX32A.h"

#define CLK_FREQ_MHZ        	25
#define I2C_CLK_FREQ_MHZ    	CLK_FREQ_MHZ
#define I2C_MAX_RISE_TIME_NS	1000
#define I2C_TPCLK_NS			125

#define I2C_GET_CCR_SM(i2c_speed) (500000LU * I2C_CLK_FREQ_MHZ / ((unsigned int)(i2c_speed)))
#define I2C_GET_CCR_FM(i2c_speed, i2c_duty) (i2c_duty == I2C_CCR_DUTY_DIV2 ? 1000000LU * I2C_CLK_FREQ_MHZ / (3 * i2c_speed) : 1000000LU * I2C_CLK_FREQ_MHZ / (25 * i2c_speed))
#define I2C_CALC_RISE_TIME(max_rise_time, TPCLK) ((max_rise_time / TPCLK) + 1)

#define GPIO_SDA1			GPIO7
#define GPIO_SCL1			GPIO6
#define GPIO_TX1			GPIO9

#define USART_BITS  8
#define USART_SPEED 9600

volatile uint32_t _millis = 0;

void delay_ms(uint32_t miliseconds);

int _write(int file, char *data, int len);

int _write(int file, char *data, int len) {
    int bytes_written;

    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
    {
        errno = EBADF;
        return -1;
    }

    for (bytes_written = 0; bytes_written < len; bytes_written++)
    {
        usart_send(USART1, *data);
        usart_send_blocking(USART1, *data);
        data++;
    }

    return bytes_written;
    return 0;
}

int main(void) {
	//Select HSE as System clock
	rcc_osc_on(RCC_HSE);
	while(!rcc_is_osc_ready(RCC_HSE));
	rcc_set_sysclk_source(RCC_CFGR_SW_HSE);
	rcc_wait_for_sysclk_status(RCC_HSE);

	//Config LED GPIO
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO13);
	gpio_set(GPIOC, GPIO13);

	//Config SysTick
	uint32_t ahb_freq = CLK_FREQ_MHZ * 1000000U;
	uint32_t SysTickInterv_ms = 1;
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	//One interrupt every 1 ms. Reload value equal to input clock frequency divided by SysTick interrupt frequency minus 1.
	//Equivalent to input clock frecuency multiplied by SysTick interrupt interval minus 1.
	systick_set_reload((ahb_freq * SysTickInterv_ms / 1000) - 1);
	systick_interrupt_enable();
	systick_counter_enable();

	//Config UART pins
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_TX1);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO_TX1);
	//Config UART
	rcc_periph_clock_enable(RCC_USART1);
	usart_set_databits(USART1, USART_BITS);
	usart_set_baudrate(USART1, USART_SPEED);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_enable(USART1);

	//Config I2C pins
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_SCL1);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO_SCL1);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO_SCL1);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_SDA1);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO_SDA1);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO_SDA1);
	//Config I2CL
	rcc_periph_clock_enable(RCC_I2C1);
	i2c_set_clock_frequency(_24XX32A_I2C, I2C_CLK_FREQ_MHZ);
	i2c_set_standard_mode(_24XX32A_I2C);
	i2c_set_dutycycle(_24XX32A_I2C, I2C_CCR_DUTY_DIV2);
	uint32_t i2c_speed = 100000U;
	uint32_t ccr = I2C_GET_CCR_SM(i2c_speed);
	i2c_set_ccr(_24XX32A_I2C, ccr);
	uint16_t trise = I2C_CALC_RISE_TIME(I2C_MAX_RISE_TIME_NS, I2C_TPCLK_NS);
	i2c_set_trise(_24XX32A_I2C, trise);
	i2c_peripheral_enable(_24XX32A_I2C);

	uint16_t address = 42;
	char text0[] = "Text saved starting at address 42.";
	page_write(address, (uint8_t *)text0, sizeof(text0));
	
	address = 4000;
	char text1[] = "Text saved starting at address 4000.";
	page_write(address, (uint8_t *)text1, sizeof(text1));

	address = _24XX32A_MAX_ADDR;
	char text2[] = "SUCCESS";
	page_write(address, (uint8_t *)text2, sizeof(text2));

	address = 42;
	char _text0[32];
	random_read(address, (uint8_t *)_text0, sizeof(text0));

	address = 4000;
	char _text1[64];
	random_read(address, (uint8_t *)_text1, sizeof(text1));

	address = _24XX32A_MAX_ADDR;
	char _text2[16];
	random_read(address, (uint8_t *)_text2, sizeof(text2));

	char message[] = "Attempt to write across the boundary of addresses 4095 and 0. If SUCCESS, that's the next word:";

	while(1) {
		printf("%s\n", _text0);
		printf("%s\n", _text1);
		printf("%s %s\n", message, _text2);
		gpio_toggle(GPIOC, GPIO13);
		delay_ms(1000);
	}
}

void sys_tick_handler(void) {
	_millis++;
}

void delay_ms(uint32_t milliseconds) {
	_millis = 0;
	while(_millis < milliseconds);
}
