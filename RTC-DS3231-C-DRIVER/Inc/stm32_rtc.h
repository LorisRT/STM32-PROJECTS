/*
 * stm32_rtc.h
 *
 *  Created on: 30 April 2023
 *      Author: LorisRT
 */

#ifndef STM32_RTC_H_
#define STM32_RTC_H_

/* STM32F407 Register Address */
#define BASE_ADDR_I2C1		0x40005400
#define BASE_ADDR_RCC		0x40023800
#define BASE_ADDR_GPIOB		0x40020400

#define RCC_APB1ENR		(BASE_ADDR_RCC + 0x40)
#define RCC_AHB1ENR		(BASE_ADDR_RCC + 0x30)
#define RCC_CR			(BASE_ADDR_RCC + 0x00)
#define RCC_CFGR		(BASE_ADDR_RCC + 0x08)
#define GPIOB_MODER			(BASE_ADDR_GPIOB + 0x00)
#define GPIOB_OTYPER		(BASE_ADDR_GPIOB + 0x04)
#define GPIOB_PUPDR			(BASE_ADDR_GPIOB + 0x0c)
#define GPIOB_AFRL			(BASE_ADDR_GPIOB + 0x20)
#define GPIOB_OSPEEDR		(BASE_ADDR_GPIOB + 0x08)
#define I2C1_CR1		(BASE_ADDR_I2C1 + 0x00)
#define I2C1_CR2		(BASE_ADDR_I2C1 + 0x04)
#define I2C1_DR			(BASE_ADDR_I2C1 + 0x10)
#define I2C1_CCR		(BASE_ADDR_I2C1 + 0x1c)
#define I2C1_TRISE		(BASE_ADDR_I2C1 + 0x20)
#define I2C1_SR1		(BASE_ADDR_I2C1 + 0x14)
#define I2C1_SR2		(BASE_ADDR_I2C1 + 0x18)

/* RTC DS3231 Register Address */
#define ADDR_RTC_DS3231		0x68
#define RTC_REG_SEC			0x00 /* 00-59 */
#define RTC_REG_MIN			0x01 /* 00-59 */
#define RTC_REG_HOUR		0x02
#define RTC_REG_DAY			0x03 /*  1-7  */
#define RTC_REG_DATE		0x04 /* 01-31 */
#define RTC_REG_MONTH		0x05 /* 01-12 */
#define RTC_REG_YEAR		0x06 /* 00-99 */

/* Macros Definition */
#define GET_STM_I2C_START_STATUS_FLAG()	(((uint8_t)(*ptr_I2C1_SR1 >> 0)) & (0x01))
#define GET_STM_I2C_ADDR_STATUS_FLAG()	(((uint8_t)(*ptr_I2C1_SR1 >> 1)) & (0x01))
#define GET_STM_I2C_ACK_STATUS_FLAG() 	(((uint8_t)(*ptr_I2C1_SR1 >> 10)) & (0x01))
#define GET_STM_I2C_TX_STATUS_FLAG()	(((uint8_t)(*ptr_I2C1_SR1 >> 7)) & (0x01))
#define GET_STM_I2C_RX_STATUS_FLAG()	(((uint8_t)(*ptr_I2C1_SR1 >> 6)) & (0x01))

/* Enumeration and Structure Definition */
typedef enum {
	STM_I2C_OK,
	STM_I2C_ADDR_FAIL,
	STM_I2C_READ_FAIL,
	STM_I2C_WRITE_FAIL,
	STM_I2C_ERROR
} stm_i2c_status_e;

/* Functions Prototype */
void stm_enable_clock(void);
void stm_i2c_config(void);
stm_i2c_status_e DS3231_I2C_writePointer(uint8_t slave_reg);
stm_i2c_status_e DS3231_I2C_readStream(uint8_t *buffer_array, uint8_t burst_size);

#endif /* STM32_RTC_H_ */
