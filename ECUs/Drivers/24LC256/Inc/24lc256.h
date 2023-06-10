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

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------
#define PRJ_24LC256_WP_PORT							 GPIOC
#define PRJ_24LC256_WP_PIN							 GPIO_PIN_14

//---------------------------------------------------------------------------
// API
//---------------------------------------------------------------------------

/*!
 * @brief Check device availability on the bus.
 *
 * This function is used to check the specified device availability several times.
 * The number of checks is defined in @ref PRJ_24LC256_TRIALS.
 *
 * @param[in] p_i2c			A pointer to I2Cx peripheral.
 * @param[in] dev_address	A target device address.
 *
 * @return @ref PRJ_STATUS_OK if the device is available.
 * @return @ref PRJ_STATUS_ERROR if a null pointer to the peripheral is passed
 * 		   or the device is not detected.
 * @return @ref PRJ_STATUS_TIMEOUT if the timeout has passed.
 */
uint32_t prj_eeprom_24lc256_connect_test(I2C_TypeDef* p_i2c, uint16_t dev_address);

#endif /* __24lc256_h */
