//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "bxCAN.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define USE_CAN						(CAN1)

//---------------------------------------------------------------------------
// Descriptions of FreeRTOS elements
//---------------------------------------------------------------------------
static osThreadId sendMessagesHandle;
static osThreadId receiveMessagesHandle;

//---------------------------------------------------------------------------
// Structure definitions
//---------------------------------------------------------------------------
USH_CAN_settingsTypeDef canInit = {0};

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------
static void bxCAN_CAN1_init(void);

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
	bxCAN_CAN1_init();

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
  * @brief  CAN1 configuration for bxCAN module.
  * @param  None.
  * @retval None.
  */
static void bxCAN_CAN1_init(void)
{
	canInit.CANx						= USE_CAN;
	canInit.Timings.BaudratePrescaler 	= 5U;
	canInit.Timings.TimeSegment1 		= CAN_TS1_TQ15;
	canInit.Timings.TimeSegment2		= CAN_TS2_TQ2;
	canInit.Timings.ResynchJumpWidth	= CAN_SJW_TQ1;
	canInit.Mode						= CAN_MODE_NORMAL;
	canInit.AutoBusOff					= ENABLE;
	canInit.AutoWakeUp					= DISABLE;
	canInit.AutoRetransmission			= ENABLE;
	canInit.ReceiveFifoLocked			= DISABLE;
	canInit.TransmitFifoPriority		= ENABLE;
	CAN_init(&canInit);
}

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
