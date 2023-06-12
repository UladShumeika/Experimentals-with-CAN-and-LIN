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
#include <stddef.h>

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------
#define PRJ_24LC256_TRIALS									(5U)

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
static prj_24lc256_dma_handlers_t m_dma_handlers = {0};
static prj_i2c_transmission_t m_i2c_tx = {0};
static prj_i2c_transmission_t m_i2c_rx = {0};

//---------------------------------------------------------------------------
// Static functions declaration
//---------------------------------------------------------------------------
#if(PRJ_24LC256_WP_ENABLED == 1U)
	static void eeprom_24lc256_write_protection(uint32_t state);
#endif

//---------------------------------------------------------------------------
// API
//---------------------------------------------------------------------------

/*!
 * @brief Check device availability on the bus.
 *
 * This function is used to check the specified device availability several times.
 * The number of checks is defined in @ref PRJ_24LC256_TRIALS.
 *
 * @param[in] dev_address	A target device address.
 *
 * @return @ref PRJ_STATUS_OK if the device is available.
 * @return @ref PRJ_STATUS_ERROR if a null pointer to the peripheral is passed
 * 		   or the device is not detected.
 * @return @ref PRJ_STATUS_TIMEOUT if the timeout has passed.
 */
uint32_t prj_eeprom_24lc256_connect_test(uint16_t dev_address)
{
	uint32_t status = PRJ_STATUS_OK;

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
#endif

	status = prj_i2c_is_device_ready(PRJ_24LC256_I2C_USED, dev_address, PRJ_24LC256_TRIALS);

	return status;
}

/*!
 * @brief Set dma handlers' pointers in 24LC256 structure.
 *
 * This function is used to get and to safe dma handlers' pointers and to link dma handlers and i2c transmission structures.
 *
 * @param[in] p_dma_tx		A ponter to dma tx handler.
 * @param[in] p_dma_rx		A ponter to dma rx handler.
 *
 * @return None.
 */
void prj_eeprom_24lc256_dma_handlers_set(prj_dma_handler_t* p_dma_tx, prj_dma_handler_t* p_dma_rx)
{
	/* Set DMA handlers' pointers */
	m_dma_handlers.p_dma_tx = p_dma_tx;
	m_dma_handlers.p_dma_rx = p_dma_rx;

	/* Link DMA tx handler and i2c tx transmission */
	m_i2c_tx.p_dma										= m_dma_handlers.p_dma_tx;
	m_dma_handlers.p_dma_tx->p_controls_peripherals 	= (void*)&m_i2c_tx;

	/* Link DMA rx handler and i2c rx transmission */
	m_i2c_rx.p_dma										= m_dma_handlers.p_dma_rx;
	m_dma_handlers.p_dma_rx->p_controls_peripherals 	= (void*)&m_i2c_rx;
}

//---------------------------------------------------------------------------
// STATIC
//---------------------------------------------------------------------------

#if(PRJ_24LC256_WP_ENABLED == 1U)
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
#endif
