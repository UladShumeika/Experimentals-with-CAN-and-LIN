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
// Structures and enumerations
//---------------------------------------------------------------------------

/**
 * @brief HSE states enumeration
 */
typedef enum
{
	RCC_HSE_OFF			= 0x00U,										/* HSE clock disabled */
	RCC_HSE_ON			= RCC_CR_HSEON,									/* HSE clock enabled */
	RCC_HSE_BYPASS		= ((uint32_t)(RCC_CR_HSEON | RCC_CR_HSEBYP))	/* HSE clock bypass */
} USH_HSE_states;

/**
 * @brief RCC flags enumeration
 */
typedef enum
{
	RCC_FLAG_HSIRDY			= RCC_CR_HSIRDY,		/* HSI clock ready flag */
	RCC_FLAG_HSERDY			= RCC_CR_HSERDY,		/* HSE clock ready flag */
	RCC_FLAG_PLLRDY			= RCC_CR_PLLRDY,		/* PLL clock ready flag */
	RCC_FLAG_PLLI2SRDY		= RCC_CR_PLLI2SRDY,		/* PLLI2S clock ready flag */
	RCC_FLAG_PLLSAIRDY		= RCC_CR_PLLSAIRDY		/* PLLSAI clock ready flag */
} USH_RCC_flags;

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
