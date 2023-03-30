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
#define HSE_STARTUP_TIMEOUT					(100U)	// 100 ms
#define PLL_TIMEOUT							(2U)	// 2 ms

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------
static USH_peripheryStatus RCC_waitFlag(USH_RCC_flags flag, uint8_t timeout, FlagStatus expectedState);

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes the selected oscillator and initializes PLL as needed.
 * @note	In this version, only HSE is configured.
 * @param 	oscInitStructure - A pointer to a USH_RCC_oscInitTypeDef structure that contains the configuration
 * 							   information for oscillators and PLL.
 * @retval	The periphery status.
 */
USH_peripheryStatus RCC_oscInit(USH_RCC_oscInitTypeDef *oscInitStructure)
{
	uint32_t startTicks;

	// Check parameters
	if(oscInitStructure == 0) return STATUS_ERROR;
	assert_param(IS_RCC_OSCTYPES(oscInitStructure->OscillatorTypes));
	assert_param(IS_RCC_PLL_STATE(oscInitStructure->PLL.PLL_state));

	/*------------------------------- HSE Configuration ------------------------*/
	if(((oscInitStructure->OscillatorTypes) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
	{
		// Check parameters
		assert_param(IS_RCC_HSE_STATE(oscInitStructure->HSE_state));

		if(oscInitStructure->HSE_state != RCC_HSE_OFF)
		{
			if(oscInitStructure->HSE_state == RCC_HSE_BYPASS)
			{
				// In order to set bypass, HSE must be turned off
				if(RCC_getFlagStatus(RCC_FLAG_HSERDY) != RESET)
				{
					// Disable HSE
					RCC->CR &= ~RCC_CR_HSEON;

					// Wait till HSE is disabled
					if(!RCC_waitFlag(RCC_FLAG_HSERDY, HSE_STARTUP_TIMEOUT, RESET))
					{
						return STATUS_TIMEOUT;
					}
				}

				// Enable HSE bypass
				RCC->CR |= (RCC_CR_HSEON | RCC_CR_HSEBYP);

				// Wait till HSE is enabled
				if(!RCC_waitFlag(RCC_FLAG_HSERDY, HSE_STARTUP_TIMEOUT, SET))
				{
					return STATUS_TIMEOUT;
				}
			} else
			{
				// Enable HSE clock
				RCC->CR |= RCC_CR_HSEON;

				// Wait till HSE is enabled
				if(!RCC_waitFlag(RCC_FLAG_HSERDY, HSE_STARTUP_TIMEOUT, SET))
				{
					return STATUS_TIMEOUT;
				}
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

//---------------------------------------------------------------------------
// Static Functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function waits for a flag to be reset or set.
 * @param 	flag - A flag to watch.
 * @param 	timeout - timeout time for the specified flag.
 * @retval	Periphery status.
 */
static USH_peripheryStatus RCC_waitFlag(USH_RCC_flags flag, uint8_t timeout, FlagStatus expectedState)
{
	uint8_t startTicks = MISC_timeoutGetTick();

	// Check parameters
	assert_param(IS_RCC_FLAGS(flag));

	if(expectedState == RESET)
	{
		// Wait till the flag is disabled
		while(RCC_getFlagStatus(flag) != RESET)
		{
			if((MISC_timeoutGetTick() - startTicks) > timeout)
			{
				return STATUS_TIMEOUT;
			}
		}
	} else
	{
		// Wait till the flag is enabled
		while(RCC_getFlagStatus(flag) == RESET)
		{
			if((MISC_timeoutGetTick() - startTicks) > timeout)
			{
				return STATUS_TIMEOUT;
			}
		}
	}

	return STATUS_OK;
}
