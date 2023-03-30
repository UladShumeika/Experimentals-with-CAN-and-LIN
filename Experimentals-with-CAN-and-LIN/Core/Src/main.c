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
#define SYS_TICK_PRIORITY			(15U)
#define SYS_TICK_1MS				(1000U)
#define PWR_OVERDRIVE_TIMEOUT		(1000U)

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------
static void initMicrocontroller(void);
static USH_peripheryStatus initSysTick(uint32_t tickPriority);
static USH_peripheryStatus initSystemClock(void);

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

	// Initialize sysTick timer
	initSysTick(SYS_TICK_PRIORITY);

	// Initialize system clock
	initSystemClock();

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
	USH_RCC_oscInitTypeDef oscInitStructure;

	uint32_t ticksStart = 0;

	// Configure the main internal regulator output voltage
	MISC_PWR_mainRegulatorModeConfig(PWR_VOLTAGE_SCALE_1);

	// Configure the External High Speed oscillator (HSE) and the main PLL
	oscInitStructure.OscillatorTypes = RCC_OSCILLATORTYPE_HSE;
	oscInitStructure.HSE_state = RCC_HSE_ON;
	oscInitStructure.PLL.PLL_source = RCC_PLLSOURCE_HSE;
	oscInitStructure.PLL.PLL_state = RCC_PLL_ON;
	oscInitStructure.PLL.PLLM = 4;
	oscInitStructure.PLL.PLLN = 180;
	oscInitStructure.PLL.PLLP = 2;
	oscInitStructure.PLL.PLLQ = 4;
	RCC_oscInit(&oscInitStructure);

	// Activate the Over-Drive mode
	PWR->CR |= PWR_CR_ODEN;

	// Wait till the Over-Drive mode is enabled
	ticksStart = MISC_timeoutGetTick();
	while(!MISC_PWR_getFlagStatus(PWR_FLAG_ODRDY))
	{
		if((MISC_timeoutGetTick() - ticksStart) > PWR_OVERDRIVE_TIMEOUT)
		{
			return STATUS_TIMEOUT;
		}
	}

	// Activate the Over-Drive switching
	PWR->CR |= PWR_CR_ODSWEN;

	// Wait till the Over-Drive switching is enabled
	while(!MISC_PWR_getFlagStatus(PWR_FLAG_ODSWRDY))
	{
		if((MISC_timeoutGetTick() - ticksStart) > PWR_OVERDRIVE_TIMEOUT)
		{
			return STATUS_TIMEOUT;
		}
	}

	// Configure FLASH LATENCY
	MISC_FLASH_setLatency(FLASH_LATENCY_5);
	if((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_LATENCY_5) return STATUS_ERROR;

	// Configure HCLK
//	RCC_PCLK1Config(RCC_HCLK_Div16); // Set the highest APBx dividers in order to ensure that it doesn't go
//	RCC_PCLK2Config(RCC_HCLK_Div16); // through a non-spec phase whatever we decrease or increase HCLK
//	RCC_HCLKConfig(RCC_SYSCLK_Div1);
//
//	// Configure SYSCLK
//	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
//	while(RCC_GetSYSCLKSource() != RCC_CFGR_SWS_PLL);
//
//	// Configure PCLK1
//	RCC_PCLK1Config(RCC_HCLK_Div4);
//
//	// Configure PCLK2
//	RCC_PCLK2Config(RCC_HCLK_Div2);
//
//	// Update the global variable SystemCoreClock
//	SystemCoreClockUpdate();
//
//	// Update SysTick with new SystemCoreClock
//	initSysTick(SYS_TICK_PRIORITY);
//
//	return SUCCESS;
}

/**
  * @brief	This function is used to initialize SysTick. The SysTick is configured
  * 		to have 1ms time base with a dedicated Tick interrupt priority
  * @param 	tickPriority Tick interrupt priority
  * @retval	ErrorStatus Status
  */
static USH_peripheryStatus initSysTick(uint32_t tickPriority)
{
	// Configure the SysTick to have interrupt in 1ms time basis
	if(SysTick_Config(SystemCoreClock / SYS_TICK_1MS) > 0U)
	{
		return STATUS_ERROR;
	}

	// Configure the SysTick IRQ priority
	if(tickPriority < (1UL << __NVIC_PRIO_BITS))
	{
		NVIC_SetPriority(SysTick_IRQn, tickPriority);
	} else
		{
			return STATUS_ERROR;
		}

	return STATUS_OK;
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
