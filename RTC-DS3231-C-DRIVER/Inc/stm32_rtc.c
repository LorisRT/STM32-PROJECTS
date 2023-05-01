/*
 * stm32_rtc.c
 *
 *  Created on: 30 April 2023
 *      Author: LorisRT
 */

#include <stdint.h>
#include "stm32_rtc.h"


/********************************/
/* Static Functions Declaration */
/********************************/

static inline void clear_addr_reg(void);
static void stm_clock_config(void);
static void stm_peripheral_clock_unable(void);
static stm_i2c_status_e stm_i2c_send_addr(uint8_t addr_sensor, uint8_t rw_bit);
static stm_i2c_status_e stm_i2c_send_data(uint8_t data);


/***********************/
/* Pointers Definition */
/***********************/

static volatile uint32_t *ptr_RCC_APB1ENR = (uint32_t *) RCC_APB1ENR;
static volatile uint32_t *ptr_RCC_AHB1ENR = (uint32_t *) RCC_AHB1ENR;
static volatile uint32_t *ptr_RCC_CR = (uint32_t *) RCC_CR;
static volatile uint32_t *ptr_RCC_CFGR = (uint32_t *) RCC_CFGR;
static volatile uint32_t *ptr_GPIOB_MODER = (uint32_t *) GPIOB_MODER;
static volatile uint32_t *ptr_GPIOB_OTYPER = (uint32_t *) GPIOB_OTYPER;
static volatile uint32_t *ptr_GPIOB_PUPDR = (uint32_t *) GPIOB_PUPDR;
static volatile uint32_t *ptr_GPIOB_AFRL = (uint32_t *) GPIOB_AFRL;
static volatile uint32_t *ptr_GPIOB_OSPEEDR = (uint32_t *) GPIOB_OSPEEDR;
static volatile uint16_t *ptr_I2C1_CR1 = (uint16_t *) I2C1_CR1;
static volatile uint16_t *ptr_I2C1_CR2 = (uint16_t *) I2C1_CR2;
static volatile uint16_t *ptr_I2C1_CCR = (uint16_t *) I2C1_CCR;
static volatile uint16_t *ptr_I2C1_TRISE = (uint16_t *) I2C1_TRISE;
static volatile uint16_t *ptr_I2C1_SR1 = (uint16_t *) I2C1_SR1;
static volatile uint16_t *ptr_I2C1_SR2 = (uint16_t *) I2C1_SR2;
static volatile uint16_t *ptr_I2C1_DR = (uint16_t *) I2C1_DR;


/************************/
/* Functions Definition */
/************************/

/**
 * @brief Function to read stream bytes of data from RTC DS3231 registers
 * @author LorisRT
 */
stm_i2c_status_e DS3231_I2C_readStream(uint8_t *buffer_array, uint8_t burst_size)
{
	uint8_t temp_idx = 0;

	/* Send slave ADDR with read condition */
	if(STM_I2C_OK != stm_i2c_send_addr(ADDR_RTC_DS3231, 1))
	{
		return STM_I2C_ADDR_FAIL;
	}

	/* Read stream burst on SDA line from sensor */
	while (burst_size-- > 1U)
	{
		while (GET_STM_I2C_RX_STATUS_FLAG() == 0x00);
		*(buffer_array + temp_idx++) = *ptr_I2C1_DR;
	}

	/* Disable ACK & generate STOP condition before reading last data byte */
	/* (reference: STM procedure from user manual for I2C peripheral) */
	*ptr_I2C1_CR1 &= ~(1 << 10);
	*ptr_I2C1_CR1 |= (1 << 9);

	/* Last read from data stream */
	while (GET_STM_I2C_RX_STATUS_FLAG() == 0x00);
	*(buffer_array + temp_idx++) = *ptr_I2C1_DR;

	return STM_I2C_OK;
}


/**
 * @brief Function to write one byte of data to RTC DS3231 register
 * @author LorisRT
 */
stm_i2c_status_e DS3231_I2C_writePointer(uint8_t slave_reg)
{
	/* Send slave ADDR with write condition */
	if (STM_I2C_OK != stm_i2c_send_addr(ADDR_RTC_DS3231, 0))
	{
		return STM_I2C_ADDR_FAIL;
	}

	/* Send sensor register ADDR on SDA line */
	if (STM_I2C_OK != stm_i2c_send_data(slave_reg))
	{
		return STM_I2C_WRITE_FAIL;
	}

	/* Send stop condition to SDA line */
	*ptr_I2C1_CR1 |= (1 << 9);

	return STM_I2C_OK;
}


/**
 * @brief Function to send 1 byte of data on SDA line for I2C1 peripherals
 * @author LorisRT
 */
static stm_i2c_status_e stm_i2c_send_data(uint8_t data)
{
	*ptr_I2C1_DR = data;
	while ((GET_STM_I2C_TX_STATUS_FLAG() == 0x00) & (GET_STM_I2C_ACK_STATUS_FLAG() == 0x00));
	if (GET_STM_I2C_ACK_STATUS_FLAG() == 0x01)
	{
		return STM_I2C_WRITE_FAIL;
	}

	return STM_I2C_OK;
}


/**
 * @brief Function to send address on SDA line for I2C1 peripherals
 * @author LorisRT
 */
static stm_i2c_status_e stm_i2c_send_addr(uint8_t addr_sensor, uint8_t rw_bit)
{
	/* Enable ACK and generate start condition*/
	*ptr_I2C1_CR1 |= (1 << 10);
	*ptr_I2C1_CR1 |= (1 << 8);
	while (GET_STM_I2C_START_STATUS_FLAG() == 0x00);

	/* Slave ADDR transmission */
	*ptr_I2C1_DR = (addr_sensor << 1) | (rw_bit << 0);
	while ((GET_STM_I2C_ADDR_STATUS_FLAG() == 0x00) && (GET_STM_I2C_ACK_STATUS_FLAG() == 0x00));
	if (GET_STM_I2C_ACK_STATUS_FLAG() == 0x01) /* Note: ADDR is not set after NACK reception */
	{
		return STM_I2C_ADDR_FAIL;
	}
	clear_addr_reg();

	return STM_I2C_OK;
}


/**
 * @brief Configure peripherals parameters for I2C1 communication
 * @author LorisRT
 */
void stm_i2c_config(void)
{
	/* Configure the desired AF (AF4 for I2C1)*/
	*ptr_GPIOB_AFRL = (*ptr_GPIOB_AFRL & ~(0xff << 24)) | (0x44 << 24); /* AF4 for GPIOB 6 (SCL) and 7 (SDA) */

	/* Configure GPIOB as AF mode for pin 6 and 7 */
	*ptr_GPIOB_MODER = (*ptr_GPIOB_MODER & ~(0xf << 12)) | (0xa << 12);

	/* Configure GPIOB speed as High speed */
	*ptr_GPIOB_OSPEEDR = (*ptr_GPIOB_OSPEEDR & ~(0xf << 12)) | (0xa << 12);

	/* Configure GPIOB as Open-Drain */
	*ptr_GPIOB_OTYPER |= (1 << 6);
	*ptr_GPIOB_OTYPER |= (1 << 7);

	/* Configure pull up resistor for i2c consideration (!! Requires additional external resistors !!) */
	*ptr_GPIOB_PUPDR = (*ptr_GPIOB_PUPDR & ~(0xf << 12)) | (0x5 << 12);

	/* Reset i2c registers before configuration */
	*ptr_I2C1_CR1 |= (1 << 15);
	*ptr_I2C1_CR1 &= ~(1 << 15);

	/* Configure SCL CLK value for 100kHz communication speed */
	*ptr_I2C1_CR2 = (*ptr_I2C1_CR2 & ~(0x3f << 0)) | (0b001000 << 0); /* 8MHz for APB1 CLK value from HSE */
	*ptr_I2C1_CCR = *ptr_I2C1_CCR & ~(1 << 15); /* 100kHz Slow Mode (SM) */
	*ptr_I2C1_CCR = (*ptr_I2C1_CCR & ~(0xfff << 0)) | (0x028 << 0); /* (T_high = CCR * T_clk) ==> (CCR = 5000ns/125ns = 40 = 0x28 */
	*ptr_I2C1_TRISE = ((*ptr_I2C1_TRISE) & ~(0b111111 << 0)) | (0b001001 << 0); /* (1000ns/125ns + 1) = 9 */

	/* Enable I2C peripheral after configuration */
	*ptr_I2C1_CR1 |= (1 << 0);
}


/**
 * @brief Enable STM CLK for I2C and GPIO peripherals
 * @author LorisRT
 */
void stm_enable_clock(void)
{
	stm_clock_config();
	stm_peripheral_clock_unable();
}


/**
 * @brief Configure STM32 CLK with HSE 8MHz
 * @author LorisRT
 */
static void stm_clock_config(void)
{
	/* Turn ON HSE CLK and wait for output to be stable */
	*ptr_RCC_CR |= (1 << 16);
	while ( (((uint8_t)(*ptr_RCC_CR >> 17)) & (0x01)) == 0x00 );

	/* Set HSE CLK as system CLK and wait for hardware indication that HSE has been set*/
	*ptr_RCC_CFGR = (*ptr_RCC_CFGR & ~(0x03 << 0)) | (0x01);
	while ( (((uint8_t)(*ptr_RCC_CFGR >> 2)) & (0x03)) != 0x01 );
}


/**
 * @brief Enable APB1 CLK for UART and I2C peripherals
 * @author LorisRT
 */
static void stm_peripheral_clock_unable(void)
{
	*ptr_RCC_AHB1ENR |= (1 << 1); /* GPIOB EN */
	*ptr_RCC_APB1ENR |= (1 << 21); /* I2C1 EN */
}


/**
 * @brief Procedure to clear addr once set
 * @author LorisRT
 */
static inline void clear_addr_reg(void)
{
	uint16_t dummyReadVar;
	dummyReadVar = *ptr_I2C1_SR1;
	dummyReadVar = *ptr_I2C1_SR2;
	(void) dummyReadVar;
}
