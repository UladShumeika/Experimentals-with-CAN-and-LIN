/**
  ******************************************************************************
  * @file    24lc256.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    09 June 2023
  * @brief   Header file of EEPROM 24LC256 driver.
  *
  ******************************************************************************
  */

#ifndef __24lc256_h
#define __24lc256_h

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_gpio.h"
#include "ush_stm32f4xx_dma.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------
#define PRJ_24LC256_I2C_USED						 (I2C2)
#define PRJ_24LC256_I2C_CLOCK_SPEED					 (100000U)

#define PRJ_24LC256_WP_ENABLED						 (0U)

#if(PRJ_24LC256_WP_ENABLED == 1U)
	#define PRJ_24LC256_WP_PORT						 GPIOC
	#define PRJ_24LC256_WP_PIN						 GPIO_PIN_14
#endif

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------

/*!
 * @brief dma handlers' pointers structure definition.
 * @note  This structure is needed to store pointers to dma handlers.
 */
typedef struct
{
	prj_dma_handler_t* p_dma_tx;		/*!< A pointer to dma tx handler */

	prj_dma_handler_t* p_dma_rx;		/*!< A pointer to dma rx handler */

} prj_24lc256_dma_handlers_t;

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
uint32_t prj_eeprom_24lc256_connect_test(uint16_t dev_address);

#endif /* __24lc256_h */
