//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "bxCAN.h"

//---------------------------------------------------------------------------
// Descriptions of FreeRTOS elements
//---------------------------------------------------------------------------
static osThreadId sendMessagesHandle;
static osThreadId receiveMessagesHandle;

//---------------------------------------------------------------------------
// FreeRTOS's threads
//---------------------------------------------------------------------------

/**
  * @brief 	Function implementing the sending messages thread.
  * @param	argument - Not used.
  * @retval	None.
  */
void bxCAN_sendMessages(void const *argument)
{
	// Infinite loop
	for(;;)
	{
		osDelay(1);
	}
}

/**
  * @brief 	Function implementing the receiving messages thread.
  * @param	argument - Not used.
  * @retval	None.
  */
void bxCAN_receiveMessages(void const *argument)
{
	// Infinite loop
	for(;;)
	{
		osDelay(1);
	}
}

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
  * @brief  FreeRTOS initialization for bxCAN module.
  * @param  None.
  * @retval None.
  */
void bxCAN_freeRtosInit(void)
{
	// Create the thread(s)
	// definition and creation of the sending messages thread
	osThreadDef(sendMessages, bxCAN_sendMessages, osPriorityLow, 0, 128);
	sendMessagesHandle = osThreadCreate(osThread(sendMessages), NULL);

	// definition and creation of the receiving messages thread
	osThreadDef(receiveMessages, bxCAN_receiveMessages, osPriorityLow, 0, 128);
	receiveMessagesHandle = osThreadCreate(osThread(receiveMessages), NULL);
}
