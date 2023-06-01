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
// Configuration section
//---------------------------------------------------------------------------

#if defined(STM32F429xx)
	#define	RCC_PLLSOURCE					(RCC_PLLSOURCE_HSE)
	#define RCC_PLLM						(4U)
	#define RCC_PLLN						(180U)
	#define RCC_PLLP						(2U)
	#define RCC_PLLQ						(4U)

#elif defined(STM32F407xx)
	#define	RCC_PLLSOURCE					(RCC_PLLSOURCE_HSE)
	#define RCC_PLLM						(4U)
	#define RCC_PLLN						(168U)
	#define RCC_PLLP						(2U)
	#define RCC_PLLQ						(4U)

#endif

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define PWR_OVERDRIVE_TIMEOUT				(1000U)

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------
static void initMicrocontroller(void);
static uint32_t initSystemClock(void);
static uint32_t initSysTick(void);

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
static uint32_t initSystemClock(void)
{
	uint32_t status = 0U;
	USH_RCC_PLL_settingsTypeDef pllInitStructure = {0};
	USH_RCC_clocksInitTypeDef clksInitStructure = {0};

#if defined(STM32F429xx)
	uint32_t ticksStart = 0;
#endif

	// Configure the main internal regulator output voltage
	MISC_PWR_mainRegulatorModeConfig(PWR_VOLTAGE_SCALE_1);

	// Enable HSE oscillator
	status = RCC_initHSE();

	if(status == 0U)
	{
		// Configure PLL
		pllInitStructure.PLL_source 	= RCC_PLLSOURCE;
		pllInitStructure.PLLM 			= RCC_PLLM;
		pllInitStructure.PLLN 			= RCC_PLLN;
		pllInitStructure.PLLP 			= RCC_PLLP;
		pllInitStructure.PLLQ 			= RCC_PLLQ;
		status = RCC_initPLL(&pllInitStructure);
	}

#if defined(STM32F429xx)
	if(status == PRJ_STATUS_OK)
	{
		// Activate the Over-Drive mode
		PWR->CR |= PWR_CR_ODEN;

		// Wait till the Over-Drive mode is enabled
		ticksStart = MISC_timeoutGetTick();
		while(!MISC_PWR_getFlagStatus(PWR_FLAG_ODRDY))
		{
			if((MISC_timeoutGetTick() - ticksStart) > PWR_OVERDRIVE_TIMEOUT)
			{
				status = PRJ_STATUS_TIMEOUT;
				break;
			}
		}
	}

	if(status == PRJ_STATUS_OK)
	{
		// Activate the Over-Drive switching
		PWR->CR |= PWR_CR_ODSWEN;

		// Wait till the Over-Drive switching is enabled
		while(!MISC_PWR_getFlagStatus(PWR_FLAG_ODSWRDY))
		{
			if((MISC_timeoutGetTick() - ticksStart) > PWR_OVERDRIVE_TIMEOUT)
			{
				status = PRJ_STATUS_TIMEOUT;
				break;
			}
		}
	}
#endif

	if(status == PRJ_STATUS_OK)
	{
		// Configure FLASH LATENCY
		MISC_FLASH_setLatency(FLASH_LATENCY_5);
		if((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_LATENCY_5) status = PRJ_STATUS_ERROR;
	}

	if(status == PRJ_STATUS_OK)
	{
		// Configure SYSCLK, HCLK and PCLKs
		clksInitStructure.SYSCLK_source		= RCC_SYSCLKSOURCE_PLL;
		clksInitStructure.HCLK_divider  	= RCC_SYSCLK_DIVIDER_1;
		clksInitStructure.APB1_divider  	= RCC_HCLK_DIVIDER_4;
		clksInitStructure.APB2_divider 		= RCC_HCLK_DIVIDER_2;
		status = RCC_initClocks(&clksInitStructure);
	}

	if(status == PRJ_STATUS_OK)
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
static uint32_t initSysTick(void)
{
	uint32_t status = PRJ_STATUS_OK;

	// Configure the SysTick to have interrupt in 1ms time basis
	if(SysTick_Config(SystemCoreClock / SYS_TICK_1MS) != 0U)
	{
		status = PRJ_STATUS_ERROR;
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
