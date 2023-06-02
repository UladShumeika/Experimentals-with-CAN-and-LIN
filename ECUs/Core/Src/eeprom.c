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
