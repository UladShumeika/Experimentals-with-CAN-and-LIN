/*
 * The pet project - Experimentals with CAN and LIN buses
 *
 * 	Created on: 21 March 2023
 *      Author: Ulad Shumeika
 *
 * This project was developed to study CAN and LIN buses,
 * as well as J1939 protocol.
 *
 */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "main.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define PWR_OVERDRIVE_TIMEOUT		(1000U)

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------
static void initMicrocontroller(void);
static USH_peripheryStatus initSystemClock(void);
static USH_peripheryStatus initSysTick(void);

//---------------------------------------------------------------------------
// Main function
//---------------------------------------------------------------------------

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// Initialize flash, cashes and timeout timer
	initMicrocontroller();

	// Initialize system clock
	initSystemClock();

	// Initialize sysTick timer
	initSysTick();

	// Call init function for freertos objects (in freertos.c)
	freeRtosInit();

	// Start scheduler
	osKernelStart();

    // Loop forever
	for(;;);
}

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
  * @brief This function is used to initialize system clock
  * @retval ErrorStatus Status
  */
static USH_peripheryStatus initSystemClock(void)
{
	USH_peripheryStatus status = STATUS_OK;
	USH_RCC_PLL_settingsTypeDef pllInitStructure = {0};
	USH_RCC_clocksInitTypeDef clksInitStructure = {0};

	// Configure the main internal regulator output voltage
	MISC_PWR_mainRegulatorModeConfig(PWR_VOLTAGE_SCALE_1);

	// Enable HSE oscillator
	status = RCC_initHSE();

	if(status == STATUS_OK)
	{
		// Configure PLL
		pllInitStructure.PLL_source 	= RCC_PLLSOURCE_HSE;
		pllInitStructure.PLLM 			= 4U;
		pllInitStructure.PLLN 			= 168U;
		pllInitStructure.PLLP 			= 2U;
		pllInitStructure.PLLQ 			= 4U;
		status = RCC_initPLL(&pllInitStructure);
	}

	if(status == STATUS_OK)
	{
		// Configure FLASH LATENCY
		MISC_FLASH_setLatency(FLASH_LATENCY_5);
		if((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_LATENCY_5) status = STATUS_ERROR;
	}

	if(status == STATUS_OK)
	{
		// Configure SYSCLK, HCLK and PCLKs
		clksInitStructure.SYSCLK_source		= RCC_SYSCLKSOURCE_PLL;
		clksInitStructure.HCLK_divider  	= RCC_SYSCLK_DIVIDER_1;
		clksInitStructure.APB1_divider  	= RCC_HCLK_DIVIDER_4;
		clksInitStructure.APB2_divider 		= RCC_HCLK_DIVIDER_2;
		status = RCC_initClocks(&clksInitStructure);
	}

	if(status == STATUS_OK)
	{
		// Update the global variable SystemCoreClock
		SystemCoreClockUpdate();

		// Update timeout timer
		MISC_timeoutTimerInit();
	}

	return status;
}

/**
  * @brief	This function is used to initialize SysTick. The SysTick is configured
  * 		to have 1ms time base with a dedicated Tick interrupt priority.
  * @retval	The peripheral status.
  */
static USH_peripheryStatus initSysTick(void)
{
	USH_peripheryStatus status = STATUS_OK;

	// Configure the SysTick to have interrupt in 1ms time basis
	if(SysTick_Config(SystemCoreClock / SYS_TICK_1MS) != 0U)
	{
		status = STATUS_ERROR;
	}

	return status;
}

/**
  * @brief 	This function is used to initialize the main nodes of the microcontroller to run.
  * 		It performs the following:
  * 			Configure Flash prefetch, Instruction cache, Data cache;
  * 			Set NVIC Group Priority to 4;
  * 			Initialize timeout timer;
  * @retval None.
  */
static void initMicrocontroller(void)
{
	// Configure Flash prefetch, Instruction cache, Data cache
	MISC_FLASH_prefetchBufferCmd(ENABLE);
	MISC_FLASH_instructionCacheCmd(ENABLE);
	MISC_FLASH_dataCacheCmd(ENABLE);

	// Set NVIC Group Priority to 4
	MISC_NVIC_setPriorityGrouping(NVIC_PRIORITYGROUP_4);

	// Initialize timeout timer
	MISC_timeoutTimerInit();
}
