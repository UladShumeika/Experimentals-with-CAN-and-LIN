/**
  ******************************************************************************
  * @file    24lc256.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    09 June 2023
  * @brief	 This file contains the implementation of functions for working with
  * 		 eeprom 24lc256.
  *
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "24lc256.h"
#include "ush_stm32f4xx_conf.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Static functions declaration
//---------------------------------------------------------------------------
static void eeprom_24lc256_write_protection(uint32_t state);

//---------------------------------------------------------------------------
// API
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// STATIC
//---------------------------------------------------------------------------

/*!
 * @brief Enable or disable eeprom write protection.
 *
 * @param state[in] 	Enable or disable state.
 *
 * @return None.
 */
static void eeprom_24lc256_write_protection(uint32_t state)
{
	if(state == PRJ_STATE_ENABLE)
	{
		GPIO_writeBits(PRJ_24LC256_WP_PORT, PRJ_24LC256_WP_PIN, GPIO_PIN_SET);
	}
	else
	{
		GPIO_writeBits(PRJ_24LC256_WP_PORT, PRJ_24LC256_WP_PIN, GPIO_PIN_RESET);
	}
}
