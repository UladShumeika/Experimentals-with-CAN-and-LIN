//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "bxCAN.h"
#include <string.h>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define USE_CAN							(CAN1)

// CAN1 interrupt priorities
#define CAN1_TX_PREEMPPRIORITY			(5U)
#define CAN1_TX_SUBPRIORITY				(0U)

#define CAN1_RX0_PREEMPPRIORITY			(5U)
#define CAN1_RX0_SUBPRIORITY			(0U)

//---------------------------------------------------------------------------
// Descriptions of FreeRTOS elements
//---------------------------------------------------------------------------
static osThreadId sendMessagesHandle;
static osThreadId receiveMessagesHandle;

//---------------------------------------------------------------------------
// Structure definitions
//---------------------------------------------------------------------------
USH_CAN_settingsTypeDef canInit = {0};
USH_CAN_filterTypeDef filterConfig = {0};

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
	CAN_TxHeaderTypeDef txMessage = {0};
	const char data[] = "Hello!";

	bxCAN_CAN1_init();

	txMessage.StdId		= 0x0000U;
	txMessage.IDE		= CAN_ID_STD;
	txMessage.RTR		= CAN_RTR_DATA;
	txMessage.DLC		= strlen(data);

	// Infinite loop
	for(;;)
	{
		CAN_addTxMessage(USE_CAN, &txMessage, (uint8_t*)data);
		osDelay(100);
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

	filterConfig.FilterIdHigh			= 0x0000U;
	filterConfig.FilterIdLow			= 0x0000U;
	filterConfig.FilterMaskIdHigh		= 0x0000U;
	filterConfig.FilterMaskIdLow		= 0x0000U;
	filterConfig.FilterFIFOAssignment	= CAN_FILTER_FIFO_0;
	filterConfig.FilterBank				= 0U;
	filterConfig.FilterMode				= CAN_FILTER_MODE_IDMASK;
	filterConfig.FilterScale			= CAN_FILTERSCALE_32BIT;
	filterConfig.FilterActivation		= CAN_FILTER_ENABLE;
	CAN_filtersConfig(USE_CAN, &filterConfig);

	CAN_enable(USE_CAN);

	CAN_interruptConfig(USE_CAN, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING), ENABLE);
}

/**
 * @brief  	This function is used to initialize CAN modules global interrupts.
 * @note	This function should not be modified, when global interrupts of CAN modules are required,
 * 			this function must be implemented in the user file.
 * @retval	None.
 */
void CAN_initGlobalInterrupts(void)
{
	MISC_NVIC_SetPriority(CAN1_TX_IRQn, CAN1_TX_PREEMPPRIORITY, CAN1_TX_SUBPRIORITY);
	MISC_NVIC_EnableIRQ(CAN1_TX_IRQn);

	MISC_NVIC_SetPriority(CAN1_RX0_IRQn, CAN1_RX0_PREEMPPRIORITY, CAN1_RX0_SUBPRIORITY);
	MISC_NVIC_EnableIRQ(CAN1_RX0_IRQn);
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