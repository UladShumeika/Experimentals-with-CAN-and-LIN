/**
  ******************************************************************************
  * @file    ush_stm32f4xx_usart.h
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    21-February-2023
  * @brief   Header file of U(S)ART module.
  *
  * NOTE: This file is not a full-fledged U(S)ART driver, but contains only some of
  * 	  the functions that are needed for the current project.
  ******************************************************************************
  */

/* The U(S)ART setting is based on the choice of a set of pins. Also sending and receiving data via U(S)ART always uses DMA.
 * The stream and channel numbers for a particular U(S)ART are listed below.
 *
 * The set of pins.
 * 		  	  |    pinsPack_1	|    pinsPack_2   |
 * U(S)ARTx___|___TX___|___RX___|___TX___|___RX___|
 * 		      |		   |		|		 |		  |
 * USART1	  |  PA9   |  PA10  |  PB6   |  PB7   |
 * USART2	  |	 PA2   |  PA3	|  PD5   |  PD6	  |
 * USART3	  |	 PB10  |  PB11	|  PC10  |  PC11  |
 * UART4	  |	 PA0   |  PA1	|  PC10  |  PC11  |
 * UART5	  |	 PC12  |  PD2	|  ---   |  ---	  |
 * USART6	  |	 PC6   |  PC7	|  PG14  |  PG9   |
 * UART7	  |	 PE8   |  PE7   |  PF7	 |  PF6   |
 * UART8	  |  PE1   |  PE0   |  ---   |  ---   |
 *
 * DMA streams and channels.
 *     	      |		            	    |
 * U(S)ARTx___|_DMAx Stream x Channel x_|
 * 		      |			                |
 * USART1_TX  |      DMA2 St.7 Ch.4     |
 * USART1_RX  |      DMA2 St.2 Ch.4     |
 * 		      |						    |
 * USART2_TX  |      DMA1 St.6 Ch.4     |
 * USART2_RX  |      DMA1 St.5 Ch.4     |
 * 		      |						    |
 * USART3_TX  |      DMA1 St.3 Ch.4     |
 * USART3_RX  |      DMA1 St.1 Ch.4     |
 * 		      |							|
 * UART4_TX   |      DMA1 St.4 Ch.4     |
 * UART4_RX   |      DMA1 St.2 Ch.4     |
 * 		      |						    |
 * UART5_TX   | 	 DMA1 St.7 Ch.4     |
 * UART5_RX   |      DMA1 St.0 Ch.4     |
 * 		      |						    |
 * USART6_TX  | 	 DMA2 St.6 Ch.5     |
 * USART6_RX  | 	 DMA2 St.1 Ch.5     |
 *			  |							|
 * UART7_TX	  | 	 DMA1 St.1 Ch.5     |
 * UART7_RX	  | 	 DMA1 St.3 Ch.5     |
 *			  |							|
 * UART8_TX   | 	 DMA1 St.0 Ch.5     |
 * UART8_RX   | 	 DMA1 St.6 Ch.5     |
 *
 */

//---------------------------------------------------------------------------
// Define to prevent recursive inclusion
//---------------------------------------------------------------------------
#ifndef __USH_STM32F4XX_USART_H
#define __USH_STM32F4XX_USART_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "stm32f4xx.h"
#include "ush_stm32f4xx_misc.h"

//---------------------------------------------------------------------------
// Structures and enumerations
//---------------------------------------------------------------------------

/**
 * @brief USART pinsPack enumeration to select pins combination for U(S)ART
 */
typedef enum
{
	USART_PINSPACK_1	= 0x00U,	/* pinsPack1 selected */
	USART_PINSPACK_2				/* pinsPack2 selected */
} USH_USART_pinsPack;

/**
 * @brief USART pinsPack enumeration to select a mode for U(S)ART
 */
typedef enum
{
	USART_MODE_RX		= 0x04U,	/* only RX selected */
	USART_MODE_TX		= 0x08U,	/* only TX selected */
	USART_MODE_RX_TX 	= 0x0CU		/* RX and TX selected */
} USH_USART_mode;

/**
  * @brief UART initialization structure definition
  */
typedef struct
{
	USART_TypeDef* USARTx;			/* A pointer to U(S)ART peripheral to be used where x is between 1 to 8 */

	USH_USART_pinsPack PinsPack;	/* U(S)ART pinsPack enumeration to select pins combination for U(S)ART.
								   	   This parameter can be a value of @ref USH_USART_pinsPack */

	uint32_t BaudRate;				/* The special value which will be used to configure the UART communication baud rate.
								   	   The baud rate is computed using the following formula:
								   	   	   - IntegerDivider = ((PCLKx) / (8 * (OVR8 + 1) * (BaudRate)))
								   	   	   - FractionalDivider = ((IntegerDivider - ((uint32_t)IntegerDivider)) * 8 * (OVR8+1)) + 0.5
	                                     	 Where OVR8 is the "oversampling by 8 mode" configuration bit in the CR1 register. */

	USH_USART_mode Mode;			/* U(S)ART modes selection. This parameter can be a value of @ref USH_USART_mode */

} USH_USART_initTypeDef;

/**
 * @brief U(S)ART flags enumeration
 */
typedef enum
{
	USART_FLAG_PE		= USART_SR_PE,		/* Parity error flag */
	USART_FLAG_FE		= USART_SR_FE,		/* Framing error flag */
	USART_FLAG_NE		= USART_SR_NE,		/* Noise detected flag */
	USART_FLAG_ORE		= USART_SR_ORE,		/* Overrun error */
	USART_FLAG_IDLE		= USART_SR_IDLE, 	/* IDLE line detected */
	USART_FLAG_RXNE		= USART_SR_RXNE,	/* Read data register not empty flag */
	USART_FLAG_TC		= USART_SR_TC,		/* Transmission complete flag */
	USART_FLAG_TXE		= USART_SR_TXE,		/* Transmit data register empty */
	USART_FLAG_LBD		= USART_SR_LBD,		/* LIN break detection flag */
	USART_FLAG_CTS		= USART_SR_CTS,		/* CTS flag */
	USART_FLAG_ALL		= 0x360U			/* CTS + LBD + TC + RXNE flags */
} USH_USART_flags;

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------

/**
 * @brief	Enable U(S)ART
 */
#define USH_USART_ENABLE(HANDLE)			((HANDLE)->CR1 |= USART_CR1_UE);

/**
 * @brief	Disable U(S)ART
 */
#define USH_USART_DISABLE(HANDLE)			((HANDLE)->CR1 &= ~USART_CR1_UE);

#define IS_USART_ALL_INSTANCE(INSTANCE)  	(((INSTANCE) == USART1)	|| \
										 	 ((INSTANCE) == USART2)	|| \
											 ((INSTANCE) == USART3)	|| \
											 ((INSTANCE) == UART4)	|| \
											 ((INSTANCE) == UART5)	|| \
											 ((INSTANCE) == USART6)	|| \
											 ((INSTANCE) == UART7)  || \
											 ((INSTANCE) == UART8))

#define IS_USART_PINSPACK(PINSPACK)			(((PINSPACK) == USART_PINSPACK_1) || \
											 ((PINSPACK) == USART_PINSPACK_2))

#define IS_USART_BAUDRATE(BAUDRATE)			(((BAUDRATE) <= 10500000U))

#define IS_USART_MODE(MODE)					(((MODE) == USART_MODE_RX) || \
											 ((MODE) == USART_MODE_TX) || \
											 ((MODE) == USART_MODE_RX_TX))

#define IS_USART_MESSAGE_SIZE(SIZE)			(((SIZE) != 0x00U))

#define IS_USART_CLEAR_FLAGS(FLAG)			(((FLAG) == USART_FLAG_PE)   || \
											 ((FLAG) == USART_FLAG_FE)   || \
											 ((FLAG) == USART_FLAG_NE)   || \
											 ((FLAG) == USART_FLAG_ORE)  || \
											 ((FLAG) == USART_FLAG_IDLE) || \
											 ((FLAG) == USART_FLAG_RXNE) || \
											 ((FLAG) == USART_FLAG_TC)   || \
											 ((FLAG) == USART_FLAG_TXE)  || \
											 ((FLAG) == USART_FLAG_LBD)  || \
											 ((FLAG) == USART_FLAG_CTS)  || \
											 ((FLAG) == USART_FLAG_ALL))

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

/**
 * @brief 	This function initializes the U(S)ART peripheral according to the specified parameters in the USH_USART_initTypeDef.
 * @note 	This function has default settings:
 *			- Asynchronous mode;
 *			- Word Length - 8 bits;
 *			- Parity - None;
 *			- Stop bits - 1;
 *			- Hardware flow control - None;
 *			- Over sampling - 16 samples.
 * @param 	initStructure - A pointer to a USH_USART_initTypeDef structure that contains the configuration information
 * 							for the specified U(S)ART peripheral.
 * @retval	None.
 */
void USART_init(USH_USART_initTypeDef *initStructure);

/**
 * @brief	This function receives an amount of data in DMA mode till either the expected number of data is received or an IDLE event occurs.
 * @param 	usart - A pointer to U(S)ART peripheral to be used where x is between 1 to 8.
 * @param 	data - The data to be transmitted.
 * @param 	size - The data transfer size.
 * @return	The periphery status.
 */
USH_peripheryStatus USART_receiveToIdleDMA(USART_TypeDef* usart, uint8_t* data, uint16_t size);

/**
 * @brief	This function transmits data using DMA.
 * @param 	usart - A pointer to U(S)ART peripheral to be used where x is between 1 to 8.
 * @param 	data - The data to be transmitted.
 * @param 	size - The data transfer size.
 * @retval	The periphery status.
 */
USH_peripheryStatus USART_transmitDMA(USART_TypeDef* usart, uint8_t* data, uint16_t size);

/**
 * @brief 	This function clears U(S)ART flags.
 * @param 	usart - A pointer to U(S)ART peripheral to be used where x is between 1 to 8.
 * @param 	flags - U(S)ART flags. This parameter can be a value of @ref USH_USART_flags.
 * @retval	None.
 */
void USART_clearFlags(USART_TypeDef* usart, USH_USART_flags flags);

/**
 * @brief 	This function returns a pointer to DMA stream depending on the received pointer to U(S)ART.
 * @param 	usart - A pointer to U(S)ART peripheral to be used where x is between 1 to 8.
 * @param 	mode - U(S)ART modes selection. This parameter can be a value of @ref USH_USART_mode. if mode is USART_MODE_RX_TX than the function
 * 				   will use mode like USART_MODE_TX.
 * @return	A pointer to DMA stream.
 */
DMA_Stream_TypeDef* USART_getDmaStream(USART_TypeDef* usart, USH_USART_mode mode);

/**
 * @brief 	This function handles U(S)ART interrupt request.
 * @param 	initStructure - A pointer to a USH_USART_initTypeDef structure that contains the configuration information
 * 							for the specified U(S)ART peripheral.
 * @retval	None.
 */
void USART_IRQHandler(USH_USART_initTypeDef *initStructure);

//---------------------------------------------------------------------------
// DMA interrupt user callbacks
//---------------------------------------------------------------------------
__WEAK void USART_idleCallback(USART_TypeDef* usart);

#endif /* __USH_STM32F4XX_USART_H */
