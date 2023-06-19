/**
  ******************************************************************************
  * @file    eeprom.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    31 May 2023
  *
  ******************************************************************************
  */

#ifndef __eeprom_h
#define __eeprom_h

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "main.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------

#if defined(STM32F429xx)
	#define prj_eeprom_i2c_ev_irq_handler					I2C2_EV_IRQHandler
	#define prj_eeprom_i2c_er_irq_handler					I2C2_ER_IRQHandler

	#define prj_eeprom_i2c_dma_tx_irq_handler				DMA1_Stream7_IRQHandler
	#define prj_eeprom_i2c_dma_rx_irq_handler				DMA1_Stream2_IRQHandler
#endif

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
void prj_eeprom_freertos_init(void);

/*!
 * @brief Handle i2c event interrupt.
 */
void prj_eeprom_i2c_ev_irq_handler(void);

/*!
 * @brief Handle i2c error interrupt.
 */
void prj_eeprom_i2c_er_irq_handler(void);

/*!
 * @brief Handle dma stream rx global interrupt.
 */
void prj_eeprom_i2c_dma_rx_irq_handler(void);

/*!
 * @brief Handle dma stream tx global interrupt.
 */
void prj_eeprom_i2c_dma_tx_irq_handler(void);

#endif /* __eeprom_h */
