/**
 ******************************************************************************
 * @file           : stm32_rtc.h
 * @author         : LorisRT
 * @brief          : BSP RTC DS3231 Header File
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
#define RTC_REG_SIZE		19 /* Number of RTC registers from ds3231 module */
#define RTC_REG_SET_SIZE	7 /* Number of registers to set in ds3231 module */
#define RTC_REG_SEC			0x00 /* 00-59 */
#define RTC_REG_MIN			0x01 /* 00-59 */
#define RTC_REG_HOUR		0x02 /* 00-23 */
#define RTC_REG_DAY			0x03 /*  1-7  */
#define RTC_REG_DATE		0x04 /* 01-31 */
#define RTC_REG_MONTH		0x05 /* 01-12 */
#define RTC_REG_YEAR		0x06 /* 00-99 */

/* Macros Definition */
#define READ		1
#define WRITE		0
#define GET_STM_HSE_STATUS_FLAG()			( ((uint8_t)(*ptr_RCC_CR >> 17)) & (0x01) )
#define GET_STM_CLK_SWITCH_STATUS_FLAG()	( ((uint8_t)(*ptr_RCC_CFGR >> 2)) & (0x03) )
#define GET_STM_I2C_START_STATUS_FLAG()		( ((uint8_t)(*ptr_I2C1_SR1 >> 0)) & (0x01) )
#define GET_STM_I2C_ADDR_STATUS_FLAG()		( ((uint8_t)(*ptr_I2C1_SR1 >> 1)) & (0x01) )
#define GET_STM_I2C_ACK_STATUS_FLAG() 		( ((uint8_t)(*ptr_I2C1_SR1 >> 10)) & (0x01) )
#define GET_STM_I2C_TX_STATUS_FLAG()		( ((uint8_t)(*ptr_I2C1_SR1 >> 7)) & (0x01) )
#define GET_STM_I2C_RX_STATUS_FLAG()		( ((uint8_t)(*ptr_I2C1_SR1 >> 6)) & (0x01) )
#define GET_STM_I2C_BTF_STATUS_FLAG()		( ((uint8_t)(*ptr_I2C1_SR1 >> 2)) & (0x01) )

/* Enumeration and Structure Definition */
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t date;
	uint8_t month;
	uint8_t year;
} time_t;

typedef enum {
	RTC_OK,
	RTC_ERROR
} rtc_ds3231_status_e;

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
rtc_ds3231_status_e rtc_getTime(time_t *t);
rtc_ds3231_status_e rtc_setTime(time_t *t);

#endif /* STM32_RTC_H_ */
