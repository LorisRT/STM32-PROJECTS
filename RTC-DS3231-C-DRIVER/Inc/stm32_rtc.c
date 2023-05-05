/**
 ******************************************************************************
 * @file           : stm32_rtc.c
 * @author         : LorisRT
 * @brief          : Board Support Package (BSP) for RTC DS3231 Module
 ******************************************************************************
 * @attention
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 * IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32_rtc.h"


/**********************************************/
/* Static Functions and Variables Declaration */
/**********************************************/
static volatile uint8_t bufferGet_RTC_DS3231[RTC_REG_SIZE];
static volatile uint8_t bufferSet_RTC_DS3231[RTC_REG_SET_SIZE];

static inline void clear_addr_reg(void);
static void stm_clock_config(void);
static void stm_peripheral_clock_unable(void);
static stm_i2c_status_e stm_i2c_send_addr(uint8_t addr_sensor, uint8_t rw_bit);
static stm_i2c_status_e stm_i2c_send_data(uint8_t data);
static stm_i2c_status_e DS3231_I2C_writePointer(uint8_t slave_reg);
static stm_i2c_status_e DS3231_I2C_readStream(volatile uint8_t *buffer_array, uint8_t burst_size);
static stm_i2c_status_e DS3231_I2C_writeStream(volatile uint8_t *buffer_array, uint8_t burst_size);


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
 * @brief: Function to extract the time measured by RTC DS3231 Module
 * @author: LorisRT
 */
rtc_ds3231_status_e rtc_getTime(time_t *t)
{
	/**
	 * As state in the data sheet: "If the register pointer is not
	 * written to before the initiation of a read mode, the first
	 * address that is read is the last one stored in the register pointer".
	 * For that reason, the RTC DS3231 register pointer is initialised
	 * to the first register (00h) before performing a stream read to be sure
	 * that the correct data format is saved in the buffer_RTC_DS3231 variable
     */
	if (STM_I2C_OK != DS3231_I2C_writePointer(RTC_REG_SEC))
	{
		return RTC_ERROR;
	}

	/* Get all RTC register content and store it in static buffer */
	if (STM_I2C_OK != DS3231_I2C_readStream(bufferGet_RTC_DS3231, RTC_REG_SIZE))
	{
		return RTC_ERROR;
	}

	/* Save current time and date into dedicated output structure */
	t->seconds = (uint8_t) ((((bufferGet_RTC_DS3231[0] >> 0) & 0x0f) * 1) + (((bufferGet_RTC_DS3231[0] >> 4) & 0x0f) * 10));
	t->minutes = (uint8_t) ((((bufferGet_RTC_DS3231[1] >> 0) & 0x0f) * 1) + (((bufferGet_RTC_DS3231[1] >> 4) & 0x0f) * 10));
	t->hours = (uint8_t) ((((bufferGet_RTC_DS3231[2] >> 0) & 0x0f) * 1) + (((bufferGet_RTC_DS3231[2] >> 4) & 0x01) * 10));
	t->date = (uint8_t) ((((bufferGet_RTC_DS3231[4] >> 0) & 0x0f) * 1) + (((bufferGet_RTC_DS3231[4] >> 4) & 0x03) * 10));
	t->month = (uint8_t) ((((bufferGet_RTC_DS3231[5] >> 0) & 0x0f) * 1) + (((bufferGet_RTC_DS3231[5] >> 4) & 0x01) * 10));
	t->year = (uint8_t) ((((bufferGet_RTC_DS3231[6] >> 0) & 0x0f) * 1) + (((bufferGet_RTC_DS3231[6] >> 4) & 0x0f) * 10));

	return RTC_OK;
}

/**
 * @brief: Function to set the time for the RTC DS3231 Module
 * @author: LorisRT
 */
rtc_ds3231_status_e rtc_setTime(time_t *t)
{
	/* Decimal to Binary-Coded Decimal (BCD) conversion */
	bufferSet_RTC_DS3231[0] = (((t->seconds)/10) << 4) | ((t->seconds) % 10);
	bufferSet_RTC_DS3231[1] = (((t->minutes)/10) << 4) | ((t->minutes) % 10);
	bufferSet_RTC_DS3231[2] = (((t->hours)/10) << 4) | ((t->hours) % 10);
	bufferSet_RTC_DS3231[3] = (uint8_t) 0; /* Day not set */
	bufferSet_RTC_DS3231[4] = (((t->date)/10) << 4) | ((t->date) % 10);
	bufferSet_RTC_DS3231[5] = (((t->month)/10) << 4) | ((t->month) % 10);
	bufferSet_RTC_DS3231[6] = (((t->year)/10) << 4) | ((t->year) % 10);

	if (STM_I2C_OK != DS3231_I2C_writePointer(RTC_REG_SEC))
	{
		return RTC_ERROR;
	}

	/* Get all RTC register content and store it in static buffer */
	if (STM_I2C_OK != DS3231_I2C_writeStream(bufferSet_RTC_DS3231, RTC_REG_SET_SIZE))
	{
		return RTC_ERROR;
	}

	return RTC_OK;
}


/**
 * @brief Function to read stream bytes of data from RTC DS3231 registers
 * @author LorisRT
 */
static stm_i2c_status_e DS3231_I2C_readStream(volatile uint8_t *buffer_array, uint8_t burst_size)
{
	uint8_t temp_idx = 0;

	/* Send slave ADDR with read condition */
	if(STM_I2C_OK != stm_i2c_send_addr(ADDR_RTC_DS3231, READ))
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
 * @brief: Function to write stream bytes of data to RTC DS3231 registers
 * 		   starting at address 00h to 12h
 * @author: LorisRT
 */
static stm_i2c_status_e DS3231_I2C_writeStream(volatile uint8_t *buffer_array, uint8_t burst_size)
{
	uint8_t temp_idx = 0, nack_flag = 0x00;

	/* Send slave ADDR with write condition */
	if(STM_I2C_OK != stm_i2c_send_addr(ADDR_RTC_DS3231, WRITE))
	{
		return STM_I2C_ADDR_FAIL;
	}

	/* Send sensor register ADDR on SDA line */
	if (STM_I2C_OK != stm_i2c_send_data(RTC_REG_SEC))
	{
		return STM_I2C_WRITE_FAIL;
	}

	/* Write stream burst on SDA line */
	while (burst_size-- > 0U && 0x01 != (nack_flag = GET_STM_I2C_ACK_STATUS_FLAG()))
	{
		while (0x00 == GET_STM_I2C_TX_STATUS_FLAG());
		*ptr_I2C1_DR = *(buffer_array + temp_idx++);
	}

	/* NACK from sensor: could not proceed data from SDA line */
	if (0x01 == nack_flag)
	{
		return STM_I2C_WRITE_FAIL;
	}

	/* Wait for TX=1 bit and send stop condition */
	while (0x00 == GET_STM_I2C_TX_STATUS_FLAG());
	*ptr_I2C1_CR1 |= (1 << 9);

	return STM_I2C_OK;
}


/**
 * @brief Function to write one byte of data to RTC DS3231 register
 * @author LorisRT
 */
static stm_i2c_status_e DS3231_I2C_writePointer(uint8_t slave_reg)
{
	/* Send slave ADDR with write condition */
	if (STM_I2C_OK != stm_i2c_send_addr(ADDR_RTC_DS3231, WRITE))
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
	while (0x00 == GET_STM_I2C_TX_STATUS_FLAG() && 0x00 == GET_STM_I2C_ACK_STATUS_FLAG());
	if (0x01 == GET_STM_I2C_ACK_STATUS_FLAG())
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
	/* Verify read/write bit argument provided to function */
	if (WRITE != rw_bit && READ != rw_bit)
	{
		return STM_I2C_ERROR;
	}

	/* Enable ACK and generate start condition */
	*ptr_I2C1_CR1 |= (1 << 10);
	*ptr_I2C1_CR1 |= (1 << 8);
	while (0x00 == GET_STM_I2C_START_STATUS_FLAG());

	/* Slave ADDR transmission */
	*ptr_I2C1_DR = (addr_sensor << 1) | (rw_bit << 0);
	while (0x00 == GET_STM_I2C_ADDR_STATUS_FLAG() && 0x00 == GET_STM_I2C_ACK_STATUS_FLAG());
	if (0x01 == GET_STM_I2C_ACK_STATUS_FLAG()) /* Note: ADDR is not set after NACK reception */
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
	*ptr_I2C1_CCR = (*ptr_I2C1_CCR & ~(0xfff << 0)) | (0x028 << 0); /* (T_high = CCR * T_clk) ==> (CCR = 5000ns/125ns = 40 = 0x28) */
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
	while (0x00 == GET_STM_HSE_STATUS_FLAG());

	/* Set HSE CLK as system CLK and wait for hardware indication that HSE has been set*/
	*ptr_RCC_CFGR = (*ptr_RCC_CFGR & ~(0x03 << 0)) | (0x01);
	while (0x01 != GET_STM_CLK_SWITCH_STATUS_FLAG());
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
 * @brief Procedure to clear ADDR bit once it is set
 * @author LorisRT
 */
static inline void clear_addr_reg(void)
{
	uint16_t dummyReadVar;
	dummyReadVar = *ptr_I2C1_SR1;
	dummyReadVar = *ptr_I2C1_SR2;
	(void) dummyReadVar;
}
