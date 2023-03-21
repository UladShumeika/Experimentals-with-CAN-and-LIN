/**
  ******************************************************************************
  * @file    ush_stm32f4xx_dma.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    04-February-2023
  * @brief   Header file of DMA module
  *
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Define to prevent recursive inclusion
//---------------------------------------------------------------------------
#ifndef __USH_STM32F4XX_DMA_H
#define __USH_STM32F4XX_DMA_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "stm32f4xx.h"

//---------------------------------------------------------------------------
// Typedefs and enumerations
//---------------------------------------------------------------------------

/**
 * @brief DMA channels enumeration to select the channel for DMA
 */
typedef enum
{
	DMA_CHANNEL_0	= 0x00000000U,		/* DMA Channel 0 */
	DMA_CHANNEL_1	= 0x02000000U,		/* DMA Channel 1 */
	DMA_CHANNEL_2	= 0x04000000U,		/* DMA Channel 2 */
	DMA_CHANNEL_3	= 0x06000000U,		/* DMA Channel 3 */
	DMA_CHANNEL_4 	= 0x08000000U,		/* DMA Channel 4 */
	DMA_CHANNEL_5	= 0x0A000000U,		/* DMA Channel 5 */
	DMA_CHANNEL_6	= 0x0C000000U,		/* DMA Channel 6 */
	DMA_CHANNEL_7	= 0x0E000000U		/* DMA Channel 7 */
} USH_DMA_channels;

/**
 * @brief DMA data transfer direction enumeration
 */
typedef enum
{
	DMA_PERIPH_TO_MEMORY	= 0x00UL,			/* Peripheral to memory direction */
	DMA_MEMORY_TO_PERIPH	= DMA_SxCR_DIR_0,	/* Memory to peripheral direction */
	DMA_MEMORY_TO_MEMORY	= DMA_SxCR_DIR_1	/* Memory to memory direction */
} USH_DMA_dataTransferDirection;

/**
 * @brief DMA peripheral incremented mode enumeration
 */
typedef enum
{
	DMA_PINC_ENABLE		= DMA_SxCR_PINC,	/* Peripheral increment mode enable */
	DMA_PINC_DISABLE	= 0x00UL			/* Peripheral increment mode disable */
} USH_DMA_periphIncrement;

/**
 * @brief DMA memory incremented mode enumeration
 */
typedef enum
{
	DMA_MINC_ENABLE		= DMA_SxCR_MINC,	/* Memory increment mode enable */
	DMA_MINC_DISABLE	= 0x00UL			/* Memory increment mode disable */
} USH_DMA_memoryIncrement;

/**
 * @brief DMA peripheral data size enumeration
 */
typedef enum
{
	DMA_PERIPH_SIZE_BYTE		= 0x000UL,  			/* Peripheral data alignment: Byte */
	DMA_PERIPH_SIZE_HALFWORD	= DMA_SxCR_PSIZE_0,		/* Peripheral data alignment: Half word	*/
	DMA_PERIPH_SIZE_WORD		= DMA_SxCR_PSIZE_1		/* Peripheral data alignment: Word */
} USH_DMA_periphDataSize;

/**
 * @brief DMA memory data size enumeration
 */
typedef enum
{
	DMA_MEMORY_SIZE_BYTE		= 0x00UL,				/* Memory data alignment: Byte */
	DMA_MEMORY_SIZE_HALFWORD	= DMA_SxCR_MSIZE_0,		/* Memory data alignment: Half word */
	DMA_MEMORY_SIZE_WORD		= DMA_SxCR_MSIZE_1		/* Memory data alignment: Word	*/
} USH_DMA_memoryDataSize;

/**
 * @brief DMA mode enumeration
 */
typedef enum
{
	DMA_NORMAL_MODE			= 0x00UL,			/* Normal mode */
	DMA_CIRCULAR_MODE		= DMA_SxCR_CIRC,	/* Circular mode */
	DMA_DOUBLE_BUFFERING	= DMA_SxCR_DBM,  	/* Double buffering mode */
	DMA_PERIPH_CTRL_MODE	= DMA_SxCR_PFCTRL	/* Peripheral control mode */
} USH_DMA_mode;

/**
 * @brief DMA priority enumeration
 */
typedef enum
{
	DMA_PRIORITY_LOW		= 0x0000UL,		/* Priority level: Low */
	DMA_PRIORITY_MEDIUM		= 0x4000UL,		/* Priority level: Medium */
	DMA_PRIORITY_HIGH		= 0x8000UL,		/* Priority level: High */
	DMA_PRIORITY_VERY_HIGH	= 0x30000UL		/* Priority level: Low */
} USH_DMA_priority;

/**
 * @brief DMA memory burst enumeration
 */
typedef enum
{
	DMA_MBURST_SINGLE	= 0x0000000UL,	/* Single transfer configuration */
	DMA_MBURST_INCR4	= 0x0800000UL,	/* Incremental burst of 4 beats */
	DMA_MBURST_INCR8	= 0x1000000UL,	/* Incremental burst of 8 beats */
	DMA_MBURST_INCR16	= 0x1800000UL	/* Incremental burst of 16 beats */
} USH_DMA_memoryBurst;

/**
 * @brief DMA peripheral burst enumeration
 */
typedef enum
{
	DMA_PBURST_SINGLE	= 0x000000UL,	/* Single transfer configuration */
	DMA_PBURST_INCR4	= 0x200000UL,	/* Incremental burst of 4 beats */
	DMA_PBURST_INCR8	= 0x400000UL,	/* Incremental burst of 8 beats */
	DMA_PBURST_INCR16	= 0x600000UL	/* Incremental burst of 16 beats */
} USH_DMA_periphBurst;

/**
 * @brief DMA FIFO mode enumeration
 */
typedef enum
{
	DMA_FIFO_MODE_ENABLE	= 0x04U,	/* FIFO mode enable */
	DMA_FIFO_MODE_DISABLE	= 0x00U		/* FIFO mode disable */
} USH_DMA_fifoMode;

/**
 * @brief DMA FIFO threshold level enumeration
 */
typedef enum
{
	DMA_FIFO_THRESHOLD_1QUARTER		= 0x00U,	/* FIFO threshold 1 quart full configuration */
	DMA_FIFO_THRESHOLD_HALF			= 0x01U,	/* FIFO threshold half full configuration */
	DMA_FIFO_THRESHOLD_3QUARTER		= 0x02U,	/* FIFO threshold 3 quart full configuration */
	DMA_FIFO_THRESHOLD_FULL			= 0x03U		/* FIFO threshold full configuration */
} USH_DMA_fifoThresholdLevel;

/**
  * @brief DMA initialization structure definition
  */
typedef struct
{
	DMA_Stream_TypeDef *DMAy_Streamx;			/* A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7. */

	USH_DMA_channels Channel;					/* A channel to be used for the specified stream.
											   	   This parameter can be a value of @ref USH_DMA_channels */

	USH_DMA_dataTransferDirection Direction;	/* A direction to be used for the specified stream.
											   	   This parameter can be a value of @ref USH_DMA_dataTransferDirection */

	USH_DMA_periphIncrement PeriphInc;			/* This parameter specifies whether the peripheral address register should
	                                               be incremented or not. This parameter can be a value of @ref USH_DMA_periphIncrement */

	USH_DMA_memoryIncrement MemInc;				/* This parameter specifies whether the memory address register should
	 	 	 	 	 	 	 	 	 	 	   	   be incremented or not. This parameter can be a value of @ref USH_DMA_memoryIncrement */

	USH_DMA_periphDataSize PeriphDataAlignment;	/* This parameter specifies the peripheral data width.
	 	 	 	 	 	 	 	 	 	 	   	   This parameter can be a value of @ref USH_DMA_periphDataSize */

	USH_DMA_memoryDataSize MemDataAlignment;	/* This parameter specifies the memory data width.
											       This parameter can be a value of @ref USH_DMA_memoryDataSize */

	USH_DMA_mode Mode;							/* This parameter specifies the operation mode of the selected stream.
	 	 	 	 	 	 	 	 	 	 	   	   This parameter can be a value of @ref USH_DMA_mode */

	USH_DMA_priority Priority;					/* This parameter specifies the software priority for the selected stream.
	 	 	 	 	 	 	 	 	 	 	   	   This parameter can be a value of @ref USH_DMA_priority */

	USH_DMA_memoryBurst MemBurst;				/* This parameter specifies the burst transfer configuration for
											   	   the memory transfers. It specifies the amount of data to be
											   	   transferred in a single non interruptible transaction.
											   	   This parameter can be a value of @ref USH_DMA_memoryBurst */

	USH_DMA_periphBurst PeriphBurst;			/* This parameter specifies the burst transfer configuration for
	   	   	   	   	   	   	   	   	   	   	   	   the peripheral transfers. It specifies the amount of data to be
	   	   	   	   	   	   	   	   	   	   	   	   transferred in a single non interruptible transaction.
	   	   	   	   	   	   	   	   	   	   	   	   This parameter can be a value of @ref USH_DMA_periphBurst */

	USH_DMA_fifoMode FIFOMode;					/* This parameter specifies if the FIFO mode or Direct mode will be used
											   	   for the selected stream. This parameter can be a value of @ref USH_DMA_fifoMode */

	USH_DMA_fifoThresholdLevel FIFOThreshold;	/* This parameter specifies the FIFO threshold level.
	 	 	 	 	 	 	 	 	 	 	   	   This parameter can be a value of @ref USH_DMA_fifoThresholdLevel */

} USH_DMA_initTypeDef;

/**
 * @brief DMA flags enumeration
 */
typedef enum
{
	DMA_FLAG_FEIF		= 0x01U,	/* FIFO error interrupt flag */
	DMA_FLAG_DMEIF		= 0x04U,	/* Direct mode error interrupt flag */
	DMA_FLAG_TEIF		= 0x08U,	/* Transfer error interrupt flag */
	DMA_FLAG_HTIF		= 0x10U,	/* Half transfer interrupt flag */
	DMA_FLAG_TCIF		= 0x20U,	/* Transfer complete interrupt flag */
	DMA_FLAG_ALL		= 0x3DU		/* All flags */
} USH_DMA_flags;

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------

#define IS_DMA_STREAM_ALL_INSTANCE(INSTANCE)  		(((INSTANCE) == DMA1_Stream0)	|| \
										 	 	 	 ((INSTANCE) == DMA1_Stream1)   || \
													 ((INSTANCE) == DMA1_Stream2)	|| \
													 ((INSTANCE) == DMA1_Stream3)	|| \
													 ((INSTANCE) == DMA1_Stream4)	|| \
													 ((INSTANCE) == DMA1_Stream5)	|| \
													 ((INSTANCE) == DMA1_Stream6)	|| \
													 ((INSTANCE) == DMA1_Stream7)	|| \
													 ((INSTANCE) == DMA2_Stream0)	|| \
										 	 	 	 ((INSTANCE) == DMA2_Stream1)	|| \
													 ((INSTANCE) == DMA2_Stream2)	|| \
													 ((INSTANCE) == DMA2_Stream3)	|| \
													 ((INSTANCE) == DMA2_Stream4)	|| \
													 ((INSTANCE) == DMA2_Stream5)	|| \
													 ((INSTANCE) == DMA2_Stream6)	|| \
													 ((INSTANCE) == DMA2_Stream7))

#define IS_DMA_CHANNEL(CHANNEL)						(((CHANNEL) == DMA_CHANNEL_0)	|| \
													 ((CHANNEL) == DMA_CHANNEL_1)	|| \
													 ((CHANNEL) == DMA_CHANNEL_2)	|| \
													 ((CHANNEL) == DMA_CHANNEL_3)	|| \
													 ((CHANNEL) == DMA_CHANNEL_4)	|| \
													 ((CHANNEL) == DMA_CHANNEL_5)	|| \
													 ((CHANNEL) == DMA_CHANNEL_6)	|| \
													 ((CHANNEL) == DMA_CHANNEL_7))

#define IS_DMA_DIRECTION(DIRECTION)					(((DIRECTION) == DMA_PERIPH_TO_MEMORY) || \
													 ((DIRECTION) == DMA_MEMORY_TO_PERIPH) || \
													 ((DIRECTION) == DMA_MEMORY_TO_MEMORY))

#define IS_DMA_PERIPH_INC(COMMAND)					(((COMMAND) == DMA_PINC_ENABLE) || \
													 ((COMMAND) == DMA_PINC_DISABLE))

#define IS_DMA_MEM_INC(COMMAND)						(((COMMAND) == DMA_MINC_ENABLE) || \
													 ((COMMAND) == DMA_MINC_DISABLE))

#define IS_DMA_PERIPH_SIZE(PERIPH_SIZE)				(((PERIPH_SIZE) == DMA_PERIPH_SIZE_BYTE) 	 || \
													 ((PERIPH_SIZE) == DMA_PERIPH_SIZE_HALFWORD) || \
													 ((PERIPH_SIZE) == DMA_PERIPH_SIZE_WORD))

#define IS_DMA_MEM_SIZE(MEM_SIZE)					(((MEM_SIZE) == DMA_MEMORY_SIZE_BYTE) 	  || \
													 ((MEM_SIZE) == DMA_MEMORY_SIZE_HALFWORD) || \
													 ((MEM_SIZE) == DMA_MEMORY_SIZE_WORD))

#define IS_DMA_MODE(MODE)							(((MODE) == DMA_NORMAL_MODE)		|| \
													 ((MODE) == DMA_CIRCULAR_MODE)		|| \
													 ((MODE) == DMA_DOUBLE_BUFFERING)	|| \
													 ((MODE) == DMA_PERIPH_CTRL_MODE))

#define IS_DMA_PRIORITY(PRIORITY)					(((PRIORITY) == DMA_PRIORITY_LOW) 	  || \
													 ((PRIORITY) == DMA_PRIORITY_MEDIUM)  || \
													 ((PRIORITY) == DMA_PRIORITY_HIGH)	  || \
													 ((PRIORITY) == DMA_PRIORITY_VERY_HIGH))

#define IS_DMA_MBURST(MBURST)						(((MBURST) == DMA_MBURST_SINGLE)  || \
													 ((MBURST) == DMA_MBURST_INCR4)   || \
													 ((MBURST) == DMA_MBURST_INCR8)   || \
													 ((MBURST) == DMA_MBURST_INCR16))

#define IS_DMA_PBURST(PBURST)						(((PBURST) == DMA_PBURST_SINGLE)  || \
													 ((PBURST) == DMA_PBURST_INCR4)   || \
													 ((PBURST) == DMA_PBURST_INCR8)   || \
													 ((PBURST) == DMA_PBURST_INCR16))

#define IS_DMA_FIFO_MODE(MODE)						(((MODE) == DMA_FIFO_MODE_ENABLE) || \
													 ((MODE) == DMA_FIFO_MODE_DISABLE))

#define IS_DMA_FIFO_THRESHOLD(THRESHOLD)			(((THRESHOLD) == DMA_FIFO_THRESHOLD_1QUARTER) || \
													 ((THRESHOLD) == DMA_FIFO_THRESHOLD_HALF)     || \
													 ((THRESHOLD) == DMA_FIFO_THRESHOLD_3QUARTER) || \
													 ((THRESHOLD) == DMA_FIFO_THRESHOLD_FULL))

#define IS_DMA_INTERRUPT_FLAGS(FLAG)				(((FLAG) == DMA_FLAG_FEIF)  || \
													 ((FLAG) == DMA_FLAG_DMEIF) || \
													 ((FLAG) == DMA_FLAG_TEIF)  || \
													 ((FLAG) == DMA_FLAG_HTIF)  || \
													 ((FLAG) == DMA_FLAG_TCIF)  || \
													 ((FLAG) == DMA_FLAG_ALL))

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes the DMAx peripheral according to the specified parameters in the USH_DMA_initTypeDef.
 * @param 	initStructure - A pointer to a USH_DMA_initTypeDef structure that contains the configuration
 * 							information for the specified DMA peripheral.
 * @retval	None.
 */
void DMA_init(USH_DMA_initTypeDef *initStructure);

/**
 * @brief 	This function enables and disables the selected DMA stream.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @param 	state - The state of the selected stream. This parameter can be a value of @ref FunctionalState.
 * @retval	None.
 */
void DMA_state(DMA_Stream_TypeDef *DMAx_Streamy, FunctionalState state);

/**
 * @brief 	This function clears DMA flags.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @param 	flags - DMA flags. This parameter can be a value of @ref USH_DMA_flags.
 * @retval	None.
 */
void DMA_clearFlags(DMA_Stream_TypeDef *DMAy_Streamx, USH_DMA_flags flags);

/**
 * @brief 	This function gets DMA flags.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @retval	DMA flags.
 */
uint32_t DMA_getFlags(USH_DMA_initTypeDef *initStructure);

/**
 * @brief 	This function returns number of data items to transfer.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @retval	Number of data items to transfer.
 */
uint16_t DMA_getNumberOfData(DMA_Stream_TypeDef *DMAy_Streamx);

/**
 * @brief 	This function handles DMA interrupt request.
 * @param 	DMAy_Streamx - A pointer to Stream peripheral to be used where y is 1 or 2 and x is from 0 to 7.
 * @retval	None.
 */
void DMA_IRQHandler(USH_DMA_initTypeDef *initStructure);

//---------------------------------------------------------------------------
// DMA interrupt user callbacks
//---------------------------------------------------------------------------
__WEAK void DMA_transferCompleteCallback(DMA_Stream_TypeDef *DMAy_Streamx);
__WEAK void DMA_halfTransferCompleteCallback(DMA_Stream_TypeDef *DMAy_Streamx);
__WEAK void DMA_transferErrorCallback(DMA_Stream_TypeDef *DMAy_Streamx);
__WEAK void DMA_directModeErrorCallback(DMA_Stream_TypeDef *DMAy_Streamx);
__WEAK void DMA_fifoErrorCallback(DMA_Stream_TypeDef *DMAy_Streamx);

#endif /* __USH_STM32F4XX_DMA_H */
