/**
  ******************************************************************************
  * @file    ush_stm32f4xx_rcc.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    21 March 2023
  * @brief   Header file of RCC module.
  *
  * NOTE: This file is not a full-fledged RCC driver, but contains only some of
  * 	  the functions that are needed for the current project.
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Define to prevent recursive inclusion
//---------------------------------------------------------------------------
#ifndef __USH_STM32F4XX_RCC_H
#define __USH_STM32F4XX_RCC_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "stm32f4xx.h"

//---------------------------------------------------------------------------
// Enable peripheral clocking
//---------------------------------------------------------------------------

/**
 * @brief Power interface clock enable
 */
#define RCC_powerInterfaceClockEnable()				RCC->APB1ENR |= RCC_APB1ENR_PWREN

/**
 * @brief TIM14 clock enable
 */
#define RCC_TIM14_ClockEnable()				RCC->APB1ENR |= RCC_APB1ENR_TIM14EN

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

/**
 * @brief 	This function returns HSI status.
 * @retval	HSI status.
 */
uint8_t getStatusHSI(void);

#endif /* __USH_STM32F4XX_MISC_H */
