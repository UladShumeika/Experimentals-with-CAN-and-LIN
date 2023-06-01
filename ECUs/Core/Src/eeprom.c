/**
  ******************************************************************************
  * @file    eeprom.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    31 May 2023
  *
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "eeprom.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
static osThreadId m_eeprom_read_write_memory_handle = {0};

//---------------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Static functions declaration
//---------------------------------------------------------------------------
static void eeprom_read_write_memory_task(void const *p_argument);
static uint32_t eeprom_init_gpio(void);

//---------------------------------------------------------------------------
// API
//---------------------------------------------------------------------------

/*!
 * @brief Initialize FreeRTOS objects.
 *
 * This function is used to initialize FreeRTOS objects (threads, queues, sems and etc.).
 *
 * @return None.
 */
void prj_eeprom_freertos_init(void)
{
	// Create the thread(s)
	// definition and creation of the read/write eeprom thread
	osThreadDef(read_write_eeprom, eeprom_read_write_memory_task, osPriorityLow, 0, 128);
	m_eeprom_read_write_memory_handle = osThreadCreate(osThread(read_write_eeprom), NULL);
}

//---------------------------------------------------------------------------
// STATIC
//---------------------------------------------------------------------------

/*!
 * @brief Function implementing the read/write eeprom thread.
 *
 * @param	p_argument - Not used.
 *
 * @return	None.
 */
static void eeprom_read_write_memory_task(void const *p_argument)
{
	/* Infinite loop */
	for(;;)
	{

	}
}

/*!
 * @brief Initialize GPIO peripherals for EEPROM memory.
 *
 * @return @ref PRJ_STATUS_OK if GPIO initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
static uint32_t eeprom_init_gpio(void)
{
	uint32_t status = PRJ_STATUS_OK;
	USH_GPIO_initTypeDef gpio_init = {0};

	/* Enable GPIOF clock */
	__RCC_GPIOF_CLOCK_ENABLE();

	/* I2C2 GPIO pins configuration
	   PF0	  ------> I2C_SDA
	   PF1    ------> I2C_SCL */
	gpio_init.GPIOx 		= GPIOF;
	gpio_init.Pin 			= (GPIO_PIN_0 | GPIO_PIN_1);
	gpio_init.Mode			= GPIO_MODE_ALTERNATE_OD;
	gpio_init.Pull			= GPIO_NOPULL;
	gpio_init.Speed			= GPIO_SPEED_VERY_HIGH;
	gpio_init.Alternate 	= GPIO_AF4_I2C2;
	status = GPIO_init(&gpio_init);

	return status;
}
