/**
  ******************************************************************************
  * @file    ush_stm32f4xx_rcc.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    21 March 2023
  * @brief	 This file contains the implementation of functions for working with RCC.
  *
  * NOTE: This file is not a full-fledged RCC driver, but contains only some of
  * 	  the functions that are needed for the current project.
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_rcc.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define HSE_STARTUP_TIMEOUT					(100U)

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes the selected oscillator and initializes PLL as needed.
 * @param 	oscInitStructure - A pointer to a USH_RCC_oscInitTypeDef structure that contains the configuration
 * 							   information for oscillators and PLL.
 * @retval	The periphery status.
 */
USH_peripheryStatus RCC_oscInit(USH_RCC_oscInitTypeDef *oscInitStructure)
{
	uint32_t startTicks;

	// Check parameters
	if(oscInitStructure == 0) return STATUS_ERROR;


	/*------------------------------- HSE Configuration ------------------------*/
	if(((oscInitStructure->OscillatorTypes) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
	{
		RCC->CR |= oscInitStructure->HSE_state;

		startTicks = MISC_timeoutGetTick();
		while(RCC_getFlagStatus(RCC_FLAG_HSERDY) == RESET)
		{
			if((MISC_timeoutGetTick() - startTicks) > HSE_STARTUP_TIMEOUT)
			{
				return STATUS_TIMEOUT;
			}
		}
	}

	/*------------------------------- HSI Configuration ------------------------*/

	//TODO add HSI Configuration

	/*------------------------------- LSE Configuration ------------------------*/

	//TODO add LSE Configuration

	/*------------------------------- LSI Configuration ------------------------*/

	//TODO add LSI Configuration

	/*------------------------------- PLL Configuration ------------------------*/

	if(oscInitStructure->PLL.PLL_state != RCC_PLL_NONE)
	{

	}

	return STATUS_OK;
}

//---------------------------------------------------------------------------
// Library Functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function returns flags status.
 * @param	flags - DMA flags. This parameter can be a value of @ref USH_DMA_flags.
 * @retval	Flags status.
 */
FlagStatus RCC_getFlagStatus(USH_RCC_flags flags)
{
	uint32_t statusReg = 0;
	FlagStatus status = RESET;

	// Check parameters
	assert_param(IS_RCC_FLAGS(flags));

	// Read RCC->CR register
	statusReg = RCC->CR;

	// Set flags status
	if(statusReg & flags)
	{
		status = SET;
	} else
	{
		status = RESET;
	}

	return status;
}
