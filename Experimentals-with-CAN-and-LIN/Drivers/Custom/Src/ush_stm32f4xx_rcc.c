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
 * @brief 	This function returns HSI status.
 * @retval	HSI status.
 */
uint8_t getStatusHSI(void)
{
	return (uint8_t)(RCC->CR & RCC_CR_HSIRDY);
}




/**
 * @brief
 *
 */
void RCC_HSEConfig(RCC_HSE)
{

}
