/**
  ******************************************************************************
  * @file    ush_stm32f4xx_conf.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    03-March-2023
  * @brief   Library configuration file.
  *
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Define to prevent recursive inclusion
//---------------------------------------------------------------------------
#ifndef __USH_STM32F4xx_CONF_H
#define __USH_STM32F4xx_CONF_H

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define IS_FUNCTIONAL_STATE(STATE) 		(((STATE) == DISABLE) || ((STATE) == ENABLE))

//---------------------------------------------------------------------------
// Structures and enumerations
//---------------------------------------------------------------------------

/**
 * @brief Periphery status enumeration.
 */
typedef enum
{
	STATUS_TIMEOUT		= 0,				/* Periphery status timeout */
	STATUS_OK	 		= !STATUS_TIMEOUT	/* Periphery status ok */
} USH_peripheryStatus;

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_misc.h"
#include "ush_stm32f4xx_gpio.h"
//#include "ush_stm32f4xx_dma.h"
//#include "ush_stm32f4xx_spi.h"
#include "ush_stm32f4xx_uart.h"
#include "ush_stm32f4xx_rcc.h"

//---------------------------------------------------------------------------
// Function's parameters check.
//---------------------------------------------------------------------------
#ifdef USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr - If expr is false, it calls assert_failed function
  *   			   which reports the name of the source file and the source
  *   			   line number of the call that failed.
  *   			   If expr is true, it returns no value.
  * @retval None.
  */
  	#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))

//---------------------------------------------------------------------------
// Function's parameters check.
//---------------------------------------------------------------------------
  	void assert_failed(uint8_t* file, uint32_t line);
#else
	#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif	/* __USH_STM32F4xx_CONF_H */
