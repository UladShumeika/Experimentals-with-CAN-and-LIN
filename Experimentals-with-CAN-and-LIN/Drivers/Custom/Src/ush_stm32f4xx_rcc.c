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




/**
 * @brief
 *
 */
void RCC_HSEConfig(RCC_HSE)
{

}
