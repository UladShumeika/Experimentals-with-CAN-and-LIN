/**
  ******************************************************************************
  * @file    ush_stm32f4xx_misc.h
  * @author  Ulad Shumeika
  * @version v1.1
  * @date    27-February-2023
  * @brief   Header file of miscellaneous module.
  *
  *
  *
  *	@Major changes v1.1
  *		- added the ability to configure a preemption priority group;
  *		- added PWR section
  *
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Define to prevent recursive inclusion
//---------------------------------------------------------------------------
#ifndef __USH_STM32F4XX_MISC_H
#define __USH_STM32F4XX_MISC_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "stm32f4xx.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define MIN_PRIORITY	(15U)

//---------------------------------------------------------------------------
// Structures and enumerations
//---------------------------------------------------------------------------

/**
 * @brief Voltage regulator scaling enumeration
 */
typedef enum
{
	PWR_VOLTAGE_SCALE_1			= PWR_CR_VOS,		/* Regulator voltage output Scale 1 mode.
													   If overdrive mode OFF then system frequency up to 168 MHz.
													   If overdrive mode ON then system frequency up to 180 MHz. */

	PWR_VOLTAGE_SCALE_2			= PWR_CR_VOS_1,		/* Regulator voltage output Scale 2 mode.
													   If overdrive mode OFF then system frequency up to 144 MHz.
													   If overdrive mode ON then system frequency up to 168 MHz. */

	PWR_VOLTAGE_SCALE_3			= PWR_CR_VOS_0		/* Regulator voltage output Scale 3 mode.
													   System frequency up to 120 MHz. */
} USH_PWR_voltageScaling;

/**
 * @brief Preemption priority group enumeration
 */
typedef enum
{
	NVIC_PRIORITYGROUP_0	= 0x00000007U,		/* 0 bits for pre-emption priority
	 	 	 	 	 	 	 	 	 	 	 	   4 bits for subpriority */

	NVIC_PRIORITYGROUP_1	= 0x00000006U,		/* 1 bits for pre-emption priority
												   3 bits for subpriority */

	NVIC_PRIORITYGROUP_2	= 0x00000005U,		/* 2 bits for pre-emption priority
												   2 bits for subpriority */

	NVIC_PRIORITYGROUP_3	= 0x00000004U,		/* 3 bits for pre-emption priority
												   1 bits for subpriority */

	NVIC_PRIORITYGROUP_4	= 0x00000003U,		/* 4 bits for pre-emption priority
												   0 bits for subpriority */
} USH_NVIC_priorityGroup;

/**
 * @brief Flash latency enumeration
 */
typedef enum
{
	FLASH_LATENCY_0,		/* Flash zero latency cycle */
	FLASH_LATENCY_1,		/* Flash one latency cycle */
	FLASH_LATENCY_2,		/* Flash two latency cycle */
	FLASH_LATENCY_3,		/* Flash three latency cycle */
	FLASH_LATENCY_4,		/* Flash four latency cycle */
	FLASH_LATENCY_5,		/* Flash five latency cycle */
	FLASH_LATENCY_6,		/* Flash six latency cycle */
	FLASH_LATENCY_7,		/* Flash seven latency cycle */
	FLASH_LATENCY_8,		/* Flash eight latency cycle */
	FLASH_LATENCY_9,		/* Flash nine latency cycle */
	FLASH_LATENCY_10,		/* Flash ten latency cycle */
	FLASH_LATENCY_11,		/* Flash eleven latency cycle */
	FLASH_LATENCY_12,		/* Flash twelve latency cycle */
	FLASH_LATENCY_13,		/* Flash thirteen latency cycle */
	FLASH_LATENCY_14,		/* Flash fourteen latency cycle */
	FLASH_LATENCY_15,		/* Flash fifteen latency cycle */
} USH_FLASH_latency;

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define IS_MISC_PWR_VOLTAGE_SCALING(SCALE)			   (((SCALE) == PWR_VOLTAGE_SCALE_1)  || \
														((SCALE) == PWR_VOLTAGE_SCALE_2)  || \
														((SCALE) == PWR_VOLTAGE_SCALE_3))

#define IS_MISC_NVIC_PRIORITY_GROUP(GROUP) 			   (((GROUP) == NVIC_PRIORITYGROUP_0) || \
                                       	   	   	 	 	((GROUP) == NVIC_PRIORITYGROUP_1) || \
												 	 	((GROUP) == NVIC_PRIORITYGROUP_2) || \
												 	 	((GROUP) == NVIC_PRIORITYGROUP_3) || \
												 	 	((GROUP) == NVIC_PRIORITYGROUP_4))

#define IS_MISC_NVIC_PREEMPTION_PRIORITY(PRIORITY)  	((PRIORITY) < 16U)

#define IS_MISC_NVIC_SUB_PRIORITY(PRIORITY)  			((PRIORITY) < 16U)

#define IS_MISC_NVIC_DEVICE_IRQ(IRQ)                	((IRQ) >= (IRQn_Type)0x00U)

#define IS_MISC_FLASH_LATENCY(LATENCY)					(((LATENCY) == FLASH_LATENCY_0) || \
														 ((LATENCY) == FLASH_LATENCY_1) || \
														 ((LATENCY) == FLASH_LATENCY_2) || \
														 ((LATENCY) == FLASH_LATENCY_3) || \
														 ((LATENCY) == FLASH_LATENCY_4) || \
														 ((LATENCY) == FLASH_LATENCY_5) || \
														 ((LATENCY) == FLASH_LATENCY_6) || \
														 ((LATENCY) == FLASH_LATENCY_7) || \
														 ((LATENCY) == FLASH_LATENCY_8) || \
														 ((LATENCY) == FLASH_LATENCY_9) || \
														 ((LATENCY) == FLASH_LATENCY_10) || \
														 ((LATENCY) == FLASH_LATENCY_11) || \
														 ((LATENCY) == FLASH_LATENCY_12) || \
														 ((LATENCY) == FLASH_LATENCY_13) || \
														 ((LATENCY) == FLASH_LATENCY_14) || \
														 ((LATENCY) == FLASH_LATENCY_15))

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// The section of power controller
//---------------------------------------------------------------------------

/**
 * @brief 	This function configures the main internal regulator output voltage.
 * @param 	voltageScaling - specifies the regulator output voltage to achieve
 * 							 a tradeoff between performance and power consumption
 * 							 when the device does not operate at the maximum frequency
 * 							 (refer to the datasheets for more details).
 * @retval 	None.
 */
void MISC_PWR_mainRegulatorModeConfig(USH_PWR_voltageScaling voltageScaling);

//---------------------------------------------------------------------------
// The section of timeout timer
//---------------------------------------------------------------------------

/**
 * @brief 	This function sets up TIM14 timer to check for timeout.
 * @retval	None.
 */
void MISC_timeoutTimer(void);

/**
 * @brief 	This function increments a variable "timeoutTicks".
 * @retval	None.
 */
void MISC_timeoutTimerIncTick(void);

/**
 * @brief 	This function returns "timeoutTicks" variable value.
 * @retval	None.
 */
uint32_t MISC_timeoutGetTick(void);

//---------------------------------------------------------------------------
// The section of NVIC
//---------------------------------------------------------------------------

/**
  * @brief  This function sets the priority grouping field (preemption priority and subpriority)
  *         using the required unlock sequence.
  * @param  priorityGroup - The priority grouping bits length.
  * @note   When the NVIC_PriorityGroup_0 is selected, IRQ preemption is no more possible.
  *         The pending IRQ priority will be managed only by the subpriority.
  * @retval None.
  */
void MISC_NVIC_setPriorityGrouping(USH_NVIC_priorityGroup priorityGroup);

/**
  * @brief  This function sets the priority of an interrupt.
  * @param  IRQn External interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f4xxxx.h))
  * @param  PreemptPriority - The preemption priority for the IRQn channel.
  *         This parameter can be a value between 0 and 15.
  *         A lower priority value indicates a higher priority.
  * @param  SubPriority - The subpriority level for the IRQ channel.
  *         This parameter can be a value between 0 and 15.
  *         A lower priority value indicates a higher priority.
  * @retval	None.
  */
void MISC_NVIC_SetPriority(IRQn_Type IRQn, uint32_t preemptPriority, uint32_t subPriority);

/**
  * @brief  This function enables a device specific interrupt in the NVIC interrupt controller.
  * @note   To configure interrupts priority correctly, the NVIC_PriorityGroupConfig()
  *         function should be called before.
  * @param  IRQn - The external interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f4xxxx.h))
  * @retval None.
  */
void MISC_NVIC_EnableIRQ(IRQn_Type IRQn);

/**
  * @brief  This function disables a device specific interrupt in the NVIC interrupt controller.
  * @param  IRQn - The external interrupt number.
  *         This parameter can be an enumerator of IRQn_Type enumeration
  *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32f4xxxx.h))
  * @retval None.
  */
void MISC_NVIC_DisableIRQ(IRQn_Type IRQn);

//---------------------------------------------------------------------------
// The section of FLASH memory
//---------------------------------------------------------------------------

/**
  * @brief  This function enables or disables the prefetch buffer.
  * @param  newState - A new state of the prefetch buffer.
  *          		   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void MISC_FLASH_prefetchBufferCmd(FunctionalState newState);

/**
  * @brief  This function enables or disables the instruction cache feature.
  * @param  newState - A new state of the instruction cache.
  *          		   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void MISC_FLASH_instructionCacheCmd(FunctionalState newState);

/**
  * @brief  This function enables or disables the data cache feature.
  * @param  newState - A new state of the data cache.
  *          		   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void MISC_FLASH_dataCacheCmd(FunctionalState newState);

/**
  * @brief  This function sets the code latency value.
  * @param  flashLatency - specifies the FLASH Latency value.
  *          			   This parameter can be a value of @ref USH_FLASH_latency.
  *
  * @note For STM32F405xx/407xx, STM32F415xx/417xx, STM32F401xx/411xE/STM32F412xG and STM32F413_423xx devices
  *       this parameter can be a value between FLASH_LATENCY_0 and FLASH_LATENCY_7.
  *
  * @note For STM32F42xxx/43xxx devices this parameter can be a value between
  *       FLASH_LATENCY_0 and FLASH_LATENCY_15.
  *
  * @retval None
  */
void MISC_FLASH_setLatency(USH_FLASH_latency flashLatency);

#endif /* __USH_STM32F4XX_MISC_H */
