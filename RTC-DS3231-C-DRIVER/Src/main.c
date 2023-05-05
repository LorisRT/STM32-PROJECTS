/**
 ******************************************************************************
 * @file           : main.c
 * @author         : LorisRT
 * @brief          : Application Layer Example for RTC DS3231 Module
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

#define SET_TIME_REQUIRED	0

/* Global variable definition for STM Studio variable viewer */
uint8_t var_seconds;
uint8_t var_minutes;
uint8_t var_hours;
uint8_t var_date;
uint8_t var_month;
uint8_t var_year;

int main(void)
{
	/**
	 * Structure definition for setting RTC time
	 * if SET_TIME_REQUIRE = 1
	 */
	time_t var_currentTime = {
		.seconds = 0,
		.minutes = 49,
		.hours = 14,
		.date = 5,
		.month = 5,
		.year = 23
	};

	/* Structure definition to save RTC time */
	time_t var_getTime;

	/* Clock initialisation for I2C1 and GPIOB */
	stm_enable_clock();

	/* I2C1 peripheral configuration and initialisation */
	stm_i2c_config();

	/* Set current time in RTC DS3231 Module if required */
	if (SET_TIME_REQUIRED && RTC_ERROR == rtc_setTime(&var_currentTime))
	{
		/**
		 * If the current time could not be set in
		 * RTC Module, loop forever
		 */
		for(;;);
	}

	/**
	 * While no error in I2C communication between
	 * STM32F407 Board and RTC DS3231 Module,
	 * read current time from RTC module and save
	 * data in global variable for STM Studio display
	 */
	while (RTC_OK == rtc_getTime(&var_getTime))
	{
		var_seconds = var_getTime.seconds;
		var_minutes = var_getTime.minutes;
		var_hours = var_getTime.hours;
		var_date = var_getTime.date;
		var_month = var_getTime.month;
		var_year = var_getTime.year;
	}

    /**
     * Loop forever:
     * Should never get here,
     * otherwise error during I2C communication
     */
	for(;;);
	return 0;
}
