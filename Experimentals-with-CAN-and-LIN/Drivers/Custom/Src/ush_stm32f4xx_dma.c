/**
  ******************************************************************************
  * @file    ush_stm32f4xx_dma.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    04-February-2023
  * @brief	 This file contains the implementation of functions for working with DMA.
  *
  *	NOTE: This file is not a full-fledged DMA driver, but contains only some of
  * 	  the functions that are needed for the current project.
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ush_stm32f4xx_dma.h"
#include "stm32f4xx.h"

//---------------------------------------------------------------------------
// Private variables
//---------------------------------------------------------------------------
static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes the DMAx peripheral according to the specified parameters in the USH_DMA_initTypeDef.
 * @param 	initStructure - A pointer to a USH_DMA_initTypeDef structure that contains the configuration
 * 							information for the specified DMA peripheral.
 * @retval	None.
 */
void DMA_init(USH_DMA_initTypeDef *initStructure)
{
	uint32_t tmpReg = 0;

	// check parameters
	assert_param(IS_DMA_STREAM_ALL_INSTANCE(initStructure->DMAy_Streamx));
	assert_param(IS_DMA_CHANNEL(initStructure->Channel));
	assert_param(IS_DMA_DIRECTION(initStructure->Direction));
	assert_param(IS_DMA_PERIPH_INC(initStructure->PeriphInc));
	assert_param(IS_DMA_MEM_INC(initStructure->MemInc));
	assert_param(IS_DMA_PERIPH_SIZE(initStructure->PeriphDataAlignment));
	assert_param(IS_DMA_MEM_SIZE(initStructure->MemDataAlignment));
	assert_param(IS_DMA_MODE(initStructure->Mode));
	assert_param(IS_DMA_PRIORITY(initStructure->Priority));
	assert_param(IS_DMA_MBURST(initStructure->MemBurst));
	assert_param(IS_DMA_PBURST(initStructure->PeriphBurst));
	assert_param(IS_DMA_FIFO_MODE(initStructure->FIFOMode));
	assert_param(IS_DMA_FIFO_THRESHOLD(initStructure->FIFOThreshold));

	DMA_state(initStructure->DMAy_Streamx, DISABLE);

	// Get the CR register value
	tmpReg = initStructure->DMAy_Streamx->CR;

	// Clear all bits except PFCTRL, TCIE, HTIE, TEIE, DMEIE, EN
	tmpReg &= 0x3FU;

	// Prepare the DMA Stream configuration
	tmpReg |= initStructure->Channel   | initStructure->MemDataAlignment 	| initStructure->MemInc	   |
			  initStructure->Direction | initStructure->PeriphDataAlignment	| initStructure->PeriphInc |
			  initStructure->Mode 	   | initStructure->Priority;

	// The memory burst and peripheral burst are not used when the FIFO is disabled
	if(initStructure->FIFOMode == DMA_FIFO_MODE_ENABLE)
	{
		tmpReg |= initStructure->MemBurst | initStructure->PeriphBurst;
	}

	// Write to DMA Stream CR register
	initStructure->DMAy_Streamx->CR = tmpReg;

	// Get the FCR register value
	tmpReg = initStructure->DMAy_Streamx->FCR;

	// Clear direct mode and FIFO threshold bits
	tmpReg &= ~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

	// Prepare the DMA stream FIFO configuration
	tmpReg |= initStructure->FIFOMode;

	if(initStructure->FIFOMode == DMA_FIFO_MODE_ENABLE)
	{
		tmpReg |= initStructure->FIFOThreshold;
	}

	// Write to DMA stream FCR
	initStructure->DMAy_Streamx->FCR = tmpReg;

	// Clear stream interrupt flags
	DMA_clearFlags(initStructure->DMAy_Streamx, DMA_FLAG_ALL);
}

//---------------------------------------------------------------------------
// Library Functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function enables and disables the selected DMA stream.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @param 	state - The state of the selected stream. This parameter can be a value of @ref FunctionalState.
 * @retval	None.
 */
void DMA_state(DMA_Stream_TypeDef *DMAy_Streamx, FunctionalState state)
{
	// Check parameters
	assert_param(IS_DMA_STREAM_ALL_INSTANCE(DMAy_Streamx));
	assert_param(IS_FUNCTIONAL_STATE(state));

	if(state == ENABLE)
	{
		DMAy_Streamx->CR |= DMA_SxCR_EN;
	} else
	{
		DMAy_Streamx->CR &= ~DMA_SxCR_EN;
	}
}

/**
 * @brief 	This function clears DMA flags.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @param 	flags - DMA flags. This parameter can be a value of @ref USH_DMA_flags.
 * @retval	None.
 */
void DMA_clearFlags(DMA_Stream_TypeDef *DMAy_Streamx, USH_DMA_flags flags)
{
	// Check parameters
	assert_param(IS_DMA_STREAM_ALL_INSTANCE(DMAy_Streamx));
	assert_param(IS_DMA_INTERRUPT_FLAGS(flags));

	DMA_TypeDef* DMAy;

	uint32_t streamNumber = ((uint32_t)DMAy_Streamx & 0xFFU) / 0x18U;	// 0xFF is a mask. 0x18 is a step between stream registers.
																		// For a better understanding of magic numbers. See the reference manual.
	DMAy = (DMAy_Streamx < DMA2_Stream0) ? DMA1 : DMA2;

	if(streamNumber < 4U)	// Stream 0-3 is LIFCR and stream 4-6 is HIFCR
	{
		DMAy->LIFCR = flags << flagBitshiftOffset[streamNumber];
	} else
	{
		DMAy->HIFCR = flags << flagBitshiftOffset[streamNumber];
	}
}

/**
 * @brief 	This function gets DMA flags.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @retval	DMA flags.
 */
uint32_t DMA_getFlags(USH_DMA_initTypeDef *initStructure)
{
	// Check parameters
	assert_param(IS_DMA_STREAM_ALL_INSTANCE(initStructure->DMAy_Streamx));

	uint32_t flags = 0;

	DMA_TypeDef* DMAy;

	uint32_t streamNumber = ((uint32_t)initStructure->DMAy_Streamx & 0xFFU) / 0x18U;	// 0xFF is a mask. 0x18 is a step between stream registers.
																						// For a better understanding of magic numbers. See the reference manual.
	DMAy = (initStructure->DMAy_Streamx < DMA2_Stream0) ? DMA1 : DMA2;

	if(streamNumber < 4U)	// Stream 0-3 is LIFCR and stream 4-6 is HIFCR
	{
		flags = DMAy->LISR >> flagBitshiftOffset[streamNumber];
	} else
	{
		flags = DMAy->HISR >> flagBitshiftOffset[streamNumber];
	}

	return flags;
}

/**
 * @brief 	This function returns number of data items to transfer.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @retval	Number of data items to transfer.
 */
uint16_t DMA_getNumberOfData(DMA_Stream_TypeDef *DMAy_Streamx)
{
	return (DMAy_Streamx->NDTR);
}

/**
 * @brief 	This function handles DMA interrupt request.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @retval	None.
 */
void DMA_IRQHandler(USH_DMA_initTypeDef *initStructure)
{
	// Get interrupt flags
	uint32_t flags = DMA_getFlags(initStructure);

	// Clear interrupt flags
	DMA_clearFlags(initStructure->DMAy_Streamx, DMA_FLAG_ALL);

	// Check transfer complete flag
	if((flags & DMA_FLAG_TCIF) && (initStructure->DMAy_Streamx->CR & DMA_SxCR_TCIE))
	{
		DMA_transferCompleteCallback(initStructure->DMAy_Streamx);
	}

	// Check half transfer complete flag
	if((flags & DMA_FLAG_HTIF) && (initStructure->DMAy_Streamx->CR & DMA_SxCR_HTIE))
	{
		DMA_halfTransferCompleteCallback(initStructure->DMAy_Streamx);
	}

	// Check transfer error flag
	if((flags & DMA_FLAG_TEIF) && (initStructure->DMAy_Streamx->CR & DMA_SxCR_TEIE))
	{
		DMA_transferErrorCallback(initStructure->DMAy_Streamx);
	}

	// Check direct mode error flag
	if((flags & DMA_FLAG_DMEIF) && (initStructure->DMAy_Streamx->CR & DMA_SxCR_DMEIE))
	{
		DMA_directModeErrorCallback(initStructure->DMAy_Streamx);
	}

	// Check FIFO error flag
	if((flags & DMA_FLAG_FEIF) && (initStructure->DMAy_Streamx->FCR & DMA_SxFCR_FEIE))
	{
		DMA_fifoErrorCallback(initStructure->DMAy_Streamx);
	}
}

//---------------------------------------------------------------------------
// DMA interrupt user callbacks
//---------------------------------------------------------------------------

/**
  * @brief  Tx Transfer completed callbacks.
  * 		NOTE: This function should not be modified, when the callback is needed,
           	   	  the DMA_transferCompleteCallback could be implemented in the user file.
  * @param  DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
  * @retval None.
  */
__WEAK void DMA_transferCompleteCallback(DMA_Stream_TypeDef *DMAy_Streamx)
{
	(void)DMAy_Streamx;
}

/**
  * @brief  Tx Half transfer completed callbacks.
  * 		NOTE: This function should not be modified, when the callback is needed,
           	   	  the DMA_halfTransferCompleteCallback could be implemented in the user file.
  * @param  DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
  * @retval None.
  */
__WEAK void DMA_halfTransferCompleteCallback(DMA_Stream_TypeDef *DMAy_Streamx)
{
	(void)DMAy_Streamx;
}

/**
  * @brief  Tx transfer error callbacks.
  * 		NOTE: This function should not be modified, when the callback is needed,
           	   	  the DMA_transferErrorCallback could be implemented in the user file.
  * @param  DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
  * @retval None.
  */
__WEAK void DMA_transferErrorCallback(DMA_Stream_TypeDef *DMAy_Streamx)
{
	(void)DMAy_Streamx;
}

/**
  * @brief  Direct mode error callbacks.
  * 		NOTE: This function should not be modified, when the callback is needed,
           	   	  the DMA_directModeErrorCallback could be implemented in the user file.
  * @param  DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
  * @retval None.
  */
__WEAK void DMA_directModeErrorCallback(DMA_Stream_TypeDef *DMAy_Streamx)
{
	(void)DMAy_Streamx;
}

/**
  * @brief  FIFO error callbacks.
  * 		NOTE: This function should not be modified, when the callback is needed,
           	   	  the DMA_fifoErrorCallback could be implemented in the user file.
  * @param  DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
  * @retval None.
  */
__WEAK void DMA_fifoErrorCallback(DMA_Stream_TypeDef *DMAy_Streamx)
{
	(void)DMAy_Streamx;
}
