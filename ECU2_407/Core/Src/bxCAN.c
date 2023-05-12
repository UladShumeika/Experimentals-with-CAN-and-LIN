//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "bxCAN.h"
#include <limits.h>
#include <string.h>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define USE_CAN							(CAN2)
#define USE_CAN_FIFO					(CAN_FILTER_FIFO_0)

// CAN1 interrupt priorities
#define CAN2_TX_PREEMPPRIORITY			(5U)
#define CAN2_TX_SUBPRIORITY				(0U)

#define CAN2_RX0_PREEMPPRIORITY			(5U)
#define CAN2_RX0_SUBPRIORITY			(0U)

//---------------------------------------------------------------------------
// Descriptions of FreeRTOS elements
//---------------------------------------------------------------------------
static osThreadId sendMessagesHandle;
static osThreadId receiveMessagesHandle;

//---------------------------------------------------------------------------
// Structure definitions
//---------------------------------------------------------------------------
USH_CAN_settingsTypeDef canInit = {0};
static USH_CAN_filterTypeDef filterConfig = {0};

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------
static void bxCAN_init(void);

//---------------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------------
static J1939_states J1939_state = J1939_STATE_UNINIT;

//---------------------------------------------------------------------------
// FreeRTOS's threads
//---------------------------------------------------------------------------

/**
  * @brief 	Function implementing the receiving messages thread.
  * @param	argument - Not used.
  * @retval	None.
  */
void bxCAN_receiveMessages(void const *argument)
{
	bxCAN_init();

	// Infinite loop
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if(J1939_state != J1939_STATE_UNINIT)
		{
		}
	}
}

/**
  * @brief 	Function implementing the sending messages thread.
  * @param	argument - Not used.
  * @retval	None.
  */
void bxCAN_sendMessages(void const *argument)
{
	uint32_t notifiedValue;

	// Infinite loop
	for(;;)
	{
		xTaskNotifyWait(0, ULONG_MAX, &notifiedValue, osWaitForever);

		switch(notifiedValue)
		{
			case J1939_NOTIFICATION_TP_CM_Abort:
				// send Abort
				break;

			case J1939_NOTIFICATION_TP_CM_BAM:
				// send BAM
				break;

			case J1939_NOTIFICATION_TP_CM_CTS:
				// send CTS
				break;

			case J1939_NOTIFICATION_TP_CM_RTS:
				// send RTS
				break;

			case J1939_NOTIFICATION_TP_CM_EndOfMsgACK:
				// send END
				break;

			default:
				break;
		}
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
static void bxCAN_init(void)
{
	canInit.CANx						= USE_CAN;
	canInit.Timings.BaudratePrescaler 	= 12U;
	canInit.Timings.TimeSegment1 		= CAN_TS1_TQ11;
	canInit.Timings.TimeSegment2		= CAN_TS2_TQ2;
	canInit.Timings.ResynchJumpWidth	= CAN_SJW_TQ1;
	canInit.Mode						= CAN_MODE_NORMAL;
	canInit.AutoBusOff					= ENABLE;
	canInit.AutoWakeUp					= DISABLE;
	canInit.AutoRetransmission			= ENABLE;
	canInit.ReceiveFifoLocked			= DISABLE;
	canInit.TransmitFifoPriority		= DISABLE;
	CAN_init(&canInit);

	filterConfig.FilterIdHigh			= 0x0000U;
	filterConfig.FilterIdLow			= 0x0000U;
	filterConfig.FilterMaskIdHigh		= 0x0000U;
	filterConfig.FilterMaskIdLow		= 0x0000U;
	filterConfig.FilterFIFOAssignment	= USE_CAN_FIFO;
	filterConfig.FilterBank				= 15U;
	filterConfig.FilterMode				= CAN_FILTER_MODE_IDMASK;
	filterConfig.FilterScale			= CAN_FILTERSCALE_32BIT;
	filterConfig.FilterActivation		= CAN_FILTER_ENABLE;
	filterConfig.SlaveStartFilterBank	= 7U;
	CAN_filtersConfig(USE_CAN, &filterConfig);

	CAN_enable(USE_CAN);

	CAN_interruptEnable(USE_CAN, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING));
}

/**
 * @brief  	This function is used to initialize CAN modules global interrupts.
 * @note	This function should not be modified, when global interrupts of CAN modules are required,
 * 			this function must be implemented in the user file.
 * @retval	None.
 */
void CAN_initGlobalInterrupts(void)
{
	MISC_NVIC_setPriority(CAN2_TX_IRQn, CAN2_TX_PREEMPPRIORITY, CAN2_TX_SUBPRIORITY);
	MISC_NVIC_enableIRQ(CAN2_TX_IRQn);

	MISC_NVIC_setPriority(CAN2_RX0_IRQn, CAN2_RX0_PREEMPPRIORITY, CAN2_RX0_SUBPRIORITY);
	MISC_NVIC_enableIRQ(CAN2_RX0_IRQn);
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

//---------------------------------------------------------------------------
// Callback functions
//---------------------------------------------------------------------------

/**
  * @brief  FIFO 0 message pending callback.
  * @note	This function should not be modified, when the callback is needed,
  * 		the CAN_rxFifo0MsgPendingCallback could be implemented in the user file.
  * @param  can - A pointer to CAN peripheral to be used where x is 1 or 2.
  * @retval None.
  */
void CAN_rxFifo0MsgPendingCallback(CAN_TypeDef* can)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(can == USE_CAN)
	{
		CAN_interruptDisable(USE_CAN, CAN_IT_RX_FIFO0_MSG_PENDING);

		vTaskNotifyGiveFromISR(receiveMessagesHandle, &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/**
  * @brief  TX mailbox 0 complete callback.
  * @note	This function should not be modified, when the callback is needed,
  * 		the CAN_txMailbox0CompleteCallback could be implemented in the user file.
  * @param  can - A pointer to CAN peripheral to be used where x is 1 or 2.
  * @retval None.
  */
void CAN_txMailbox0CompleteCallback(CAN_TypeDef* can)
{

}
