//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "heartbeat.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define PORT_LED_HEARTBEAT				GPIOD
#define PIN_LED_HEARTBEAT				GPIO_PIN_15
#define BLICK_DELAY_HEARTBEAT			(1000U)				// ms

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------
static void HEARTBEAT_gpioInit(void);

//---------------------------------------------------------------------------
// Descriptions of FreeRTOS elements
//---------------------------------------------------------------------------
static osThreadId heartbeatHandle;

//---------------------------------------------------------------------------
// FreeRTOS's threads
//---------------------------------------------------------------------------

/**
  * @brief 	Function implementing the heartbeat thread.
  * @param	argument - Not used.
  * @retval	None.
  */
void heartbeatTask(void const *argument)
{
	HEARTBEAT_gpioInit();

	// Infinite loop
	for(;;)
	{
		GPIO_toggleBits(PORT_LED_HEARTBEAT, PIN_LED_HEARTBEAT);
		osDelay(BLICK_DELAY_HEARTBEAT);
	}
}

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
  * @brief  GPIO configuration for heartbeat module.
  * @param  None.
  * @retval None.
  */
static void HEARTBEAT_gpioInit(void)
{
	USH_GPIO_initTypeDef gpioInitStructure = {0};

	// GPIOG clock enable
	__RCC_GPIOD_CLOCK_ENABLE();

	// Configure GPIO pins : PIN_LED_HEARTBEAT
	gpioInitStructure.GPIOx				= PORT_LED_HEARTBEAT;
	gpioInitStructure.Pin				= PIN_LED_HEARTBEAT;
	gpioInitStructure.Mode				= GPIO_MODE_OUTPUT_PP;
	gpioInitStructure.Pull				= GPIO_PULLDOWN;
	gpioInitStructure.Speed				= GPIO_SPEED_VERY_HIGH;
	GPIO_init(&gpioInitStructure);

	// Configure GPIO pin Output Level
	GPIO_resetBits(PORT_LED_HEARTBEAT, PIN_LED_HEARTBEAT);
}

/**
  * @brief  FreeRTOS initialization for heartbeat module.
  * @param  None.
  * @retval None.
  */
void HEARTBEAT_freeRtosInit(void)
{
	// Create the thread(s)
	// definition and creation of HeartbeatTask
	osThreadDef(Heartbeat, heartbeatTask, osPriorityLow, 0, 128);
	heartbeatHandle = osThreadCreate(osThread(Heartbeat), NULL);
}
