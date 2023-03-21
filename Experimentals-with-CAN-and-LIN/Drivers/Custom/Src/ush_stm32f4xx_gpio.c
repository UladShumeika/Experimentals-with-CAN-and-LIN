/**
  ******************************************************************************
  * @file    ush_stm32f4xx_gpio.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    13-January-2023
  * @brief	 This file contains the implementation of functions for working with GPIO.
  *
  *	NOTE: This file is not a full-fledged GPIO driver, but contains only some of
  * 	  the functions that are needed for the current project.
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_gpio.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define GPIO_NUMBER           				16U

#define OUTPUT_TYPE							((uint32_t)0x10)
#define OUTPUT_TYPE_POS						4U

#define GPIO_MODE							((uint32_t)0x03)
#define GPIO_OUTPUT							((uint32_t)0x01)
#define GPIO_AF								((uint32_t)0x02)

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes the GPIOx peripheral according to the specified parameters in the USH_GPIO_initTypeDef.
 * @param 	initStructure - a pointer to a USH_GPIO_initTypeDef structure that contains the configuration information
 * 						    for the specified GPIO peripheral.
 * @retval	None.
 */
void GPIO_init(USH_GPIO_initTypeDef *initStructure)
{
	uint32_t pinPos, pos, currentPin, temp;

	// Check parameters
	assert_param(IS_GPIO_ALL_INSTANCE(initStructure->GPIOx));
	assert_param(IS_GPIO_PIN(initStructure->Pin));
	assert_param(IS_GPIO_MODE(initStructure->Mode));
	assert_param(IS_GPIO_PULL(initStructure->Pull));
	assert_param(IS_GPIO_SPEED(initStructure->Speed));

	// Configure the port pins
	for(pinPos = 0x00; pinPos < GPIO_NUMBER; pinPos++)
	{
		pos = ((uint32_t)0x01) << pinPos;			// Get the IO position
		currentPin = initStructure->Pin & pos;		// Get the current IO position

		if(currentPin == pos)
		{
			// In case of Output or Alternate function mode selection
			if(((initStructure->Mode & GPIO_MODE) == GPIO_OUTPUT) || ((initStructure->Mode & GPIO_MODE) == GPIO_AF))
			{
				// Speed mode configuration
				temp = initStructure->GPIOx->OSPEEDR;
				temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinPos * 2U));
				temp |= (initStructure->Speed << (pinPos * 2U));
				initStructure->GPIOx->OSPEEDR = temp;

				// Output mode configuration
				temp = initStructure->GPIOx->OTYPER;
				temp &= ~(GPIO_OTYPER_OT_0 << ((uint16_t)pinPos));
				temp |= ((initStructure->Mode & OUTPUT_TYPE) >> OUTPUT_TYPE_POS);
				initStructure->GPIOx->OTYPER = temp;
			}

			if(initStructure->Mode != GPIO_MODE_ANALOG)
			{
				// Activate the Pull-up or Pull down resistor for the current IO
				temp = initStructure->GPIOx->PUPDR;
				temp &= ~(GPIO_PUPDR_PUPDR0 << (pinPos * 2U));
				temp |= (initStructure->Pull << (pinPos * 2U));
				initStructure->GPIOx->PUPDR = temp;
			}

			// In case of Alternate function mode selection
			if((initStructure->Mode & GPIO_MODE) == GPIO_AF)
			{
				// Check parameters
				assert_param(IS_GPIO_ALTERNATE(initStructure->Alternate));

				// Configure Alternate function mapped with the current IO
				temp = initStructure->GPIOx->AFR[pinPos >> 3U];
				temp &= ~((uint32_t)0x0F << ((pinPos) & 0x07UL) * 4U);
				temp |= (initStructure->Alternate << (((pinPos) & 0x07UL) * 4U));
				initStructure->GPIOx->AFR[pinPos >> 3U] = temp;
			}

			// Configure IO Direction mode (Input, Output, Alternate or Analog)
			temp = initStructure->GPIOx->MODER;
			temp &= ~(GPIO_MODER_MODER0 << (pinPos * 2U));
			temp |= (initStructure->Mode << (pinPos * 2U));
			initStructure->GPIOx->MODER = temp;
		}
	}
}

//---------------------------------------------------------------------------
// Library functions
//---------------------------------------------------------------------------

/**
 * @brief	This function clears the selected data port bits.
 * @note	This function uses GPIOx_BSRR register to allow atomic read/modify accesses. In this way,
 * 			there is no risk of an IRQ occurring between the read and the modify access.
 * @param 	GPIOx - A pointer to GPIOx peripheral to be used where x is between A to F.
 * @param 	gpioPins - The GPIO pins to be configured. This parameter can be any value of @ref USH_GPIO_pins.
 * @retval	None.
 */
void GPIO_resetBits(GPIO_TypeDef *GPIOx, USH_GPIO_pins gpioPins)
{
	// Check the parameters
	assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
	assert_param(IS_GPIO_PIN(gpioPins));

	GPIOx->BSRR = (uint32_t)gpioPins << GPIO_NUMBER;
}

/**
 * @brief	This function sets or clears the selected data port bits.
 * @param 	GPIOx - A pointer to GPIOx peripheral to be used where x is between A to F.
 * @param 	gpioPins - The GPIO pins to be configured. This parameter can be any value of @ref USH_GPIO_pins.
 * @param 	state - The value to be written to the selected bit. This parameter can be any value of @ref USH_GPIO_pinState.
 * @retval	None.
 */
void GPIO_writeBits(GPIO_TypeDef *GPIOx, USH_GPIO_pins gpioPins, USH_GPIO_pinState state)
{
	// Check the parameters
	assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
	assert_param(IS_GPIO_PIN(gpioPins));
	assert_param(IS_GPIO_STATE(state));

	if(state != GPIO_PIN_RESET)
	{
		GPIOx->BSRR = gpioPins;
	}
	else
	{
		GPIOx->BSRR = (uint32_t)gpioPins << GPIO_NUMBER;
	}
}

/**
 * @brief 	This function toggles the specified GPIO pins.
 * @param 	GPIOx - A pointer to GPIOx peripheral to be used where x is between A to F.
 * @param  	gpioPins - The GPIO pins to be configured. This parameter can be any value of @ref USH_GPIO_pins.
 * @retval	None.
 */
void GPIO_toggleBits(GPIO_TypeDef* GPIOx, USH_GPIO_pins gpioPins)
{
	// Check the parameters
	assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
	assert_param(IS_GPIO_PIN(gpioPins));

	GPIOx->ODR ^= gpioPins;
}
