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
 * @brief Oscillator types enumeration.
 */
typedef enum
{
	RCC_OSCILLATORTYPE_NONE		= 0x00UL,	/* No oscillator will be configured */
	RCC_OSCILLATORTYPE_HSE		= 0x01UL,	/* HSE oscillator will be configured */
	RCC_OSCILLATORTYPE_HSI		= 0x02UL,	/* HSI oscillator will be configured */
	RCC_OSCILLATORTYPE_LSE		= 0x04UL,	/* LSE oscillator will be configured */
	RCC_OSCILLATORTYPE_LSI		= 0x08UL	/* LSI oscillator will be configured */
} USH_RCC_oscillatorTypes;

/**
 * @brief HSE states enumeration
 */
typedef enum
{
	RCC_HSE_OFF			= 0x00U,										/* HSE clock disabled */
	RCC_HSE_ON			= RCC_CR_HSEON,									/* HSE clock enabled */
	RCC_HSE_BYPASS		= ((uint32_t)(RCC_CR_HSEON | RCC_CR_HSEBYP))	/* HSE clock bypass */
} USH_RCC_HSE_states;

/**
 * @brief RCC PLL source enumeration.
 */
typedef enum
{
	RCC_PLLSOURCE_HSI	= RCC_PLLCFGR_PLLSRC_HSI,	/* HSI oscillator clock selected as PLL and PLLI2S clock entry */
	RCC_PLLSOURCE_HSE	= RCC_PLLCFGR_PLLSRC_HSE	/* HSE oscillator clock selected as PLL and PLLI2S clock entry */
} USH_RCC_PLL_source;

/**
 * @brief RCC PLL states enumeration.
 */
typedef enum
{
	RCC_PLL_NONE	= 0x00,		/* PLL is not used */
	RCC_PLL_ON		= 0x01,		/* PLL enabled */
	RCC_PLL_OFF		= 0x02		/* PLL disabled */
} USH_RCC_PLL_states;

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
// Macros
//---------------------------------------------------------------------------
#define IS_RCC_HSE_STATE(STATE)			           (((STATE) == RCC_HSE_OFF)	|| \
													((STATE) == RCC_HSE_ON)	  	|| \
													((STATE) == RCC_HSE_BYPASS))

#define IS_RCC_FLAGS(FLAG)						   (((FLAG) == RCC_FLAG_HSIRDY)		|| \
													((FLAG) == RCC_FLAG_HSERDY) 	|| \
													((FLAG) == RCC_FLAG_PLLRDY) 	|| \
													((FLAG) == RCC_FLAG_PLLI2SRDY) 	|| \
													((FLAG) == RCC_FLAG_PLLSAIRDY))

//---------------------------------------------------------------------------
// Enable peripheral clocking
//---------------------------------------------------------------------------

/**
 * @brief Power interface clock enable
 */
#define RCC_powerInterfaceClockEnable()		RCC->APB1ENR |= RCC_APB1ENR_PWREN

/**
 * @brief TIM14 clock enable
 */
#define RCC_TIM14_ClockEnable()				RCC->APB1ENR |= RCC_APB1ENR_TIM14EN

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

/**
 * @brief 	This function returns flags status.
 * @param	flags - DMA flags. This parameter can be a value of @ref USH_DMA_flags.
 * @retval	Flags status.
 */
FlagStatus RCC_getFlagStatus(USH_RCC_flags flags);

#endif /* __USH_STM32F4XX_MISC_H */
