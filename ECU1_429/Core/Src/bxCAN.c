//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "bxCAN.h"
#include "SAE_J1939_21_Transport_Layer.h"
#include <limits.h>

//---------------------------------------------------------------------------
// Configuration section
//---------------------------------------------------------------------------

/* NOTE: When changing the CAN bus configuration, it is necessary to check
 * 		 the configuration of the bus(including filters) and global interrupts.
 */

#if defined(STM32F429xx)
	#define CURRENT_ECU_ADDRESS					(101U)

	#define USE_CAN								(CAN1)
	#define USE_CAN_FIFO						(CAN_FILTER_FIFO_0)

	#define CAN_BAUDRATE_PRESCALER				(10U)			// Bit rate 250 kbit/s at PCLK1 = 45 MHz
	#define CAN_TIME_SEGMENT_1					(CAN_TS1_TQ15)
	#define CAN_TIME_SEGMENT_2					(CAN_TS2_TQ2)
	#define CAN_RESYCH_JUMP_WIDTH				(CAN_SJW_TQ1)

	#define CAN_TX_PREEMPPRIORITY				(5U)
	#define CAN_TX_SUBPRIORITY					(0U)

	#define CAN_RX0_PREEMPPRIORITY				(5U)
	#define CAN_RX0_SUBPRIORITY					(0U)

#elif defined(STM32F407xx)
	#define CURRENT_ECU_ADDRESS					(202U)

	#define USE_CAN								(CAN2)
	#define USE_CAN_FIFO						(CAN_FILTER_FIFO_0)

	#define CAN_BAUDRATE_PRESCALER				(12U)			// Bit rate 250 kbit/s at PCLK1 = 42 MHz
	#define CAN_TIME_SEGMENT_1					(CAN_TS1_TQ11)
	#define CAN_TIME_SEGMENT_2					(CAN_TS2_TQ2)
	#define CAN_RESYCH_JUMP_WIDTH				(CAN_SJW_TQ1)

	#define CAN_TX_PREEMPPRIORITY				(5U)
	#define CAN_TX_SUBPRIORITY					(0U)

	#define CAN_RX0_PREEMPPRIORITY				(5U)
	#define CAN_RX0_SUBPRIORITY					(0U)

#endif

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define J1939_MESSAGE_PACKET_FREQ				(125U) // from 50 to 200 ms
#define J1939_MESSAGE_TIMEOUT					(750U)

#define J1939_BROADCAST_ADDRESS					(255U)

#define J1939_CONNECTION_MANAGEMENT				(0xECU)
#define J1939_DATA_TRANSFER						(0xEBU)

//---------------------------------------------------------------------------
// Descriptions of FreeRTOS elements
//---------------------------------------------------------------------------
static osThreadId sendMessagesHandle;
static osThreadId receiveMessagesHandle;
static osSemaphoreId receiveMessagesSemHandle;
static osTimerId timeoutTimerHandle;

//---------------------------------------------------------------------------
// Structure definitions
//---------------------------------------------------------------------------
USH_CAN_settingsTypeDef canInit = {0};
USH_CAN_filterTypeDef filterConfig = {0};

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
		osSemaphoreWait(receiveMessagesSemHandle, osWaitForever);

		if(J1939_state != J1939_STATE_UNINIT)
		{
			J1939_messagesProcessing();
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

/**
  * @brief 	Function implementing the timer callback
  * @param	argument - Used to get a pointer to the state of the J1939 protocol.
  * @retval	None.
  */
void timeoutTimer_Callback(void const *argument)
{
	J1939_states result = *(J1939_states*)pvTimerGetTimerID((TimerHandle_t)argument);

	switch(result)
	{
		case J1939_STATE_TP_RECEIVING_BROADCAST:
		case J1939_STATE_TP_RECEIVING_PEER_TO_PEER:
			xTaskNotify(sendMessagesHandle, (uint32_t)J1939_NOTIFICATION_TP_CM_Abort, eSetBits);
			break;

		case J1939_STATE_TP_SENDING_BROADCAST:
		case J1939_STATE_TP_SENDING_PEER_TO_PEER:
			xTaskNotify(sendMessagesHandle, (uint32_t)J1939_NOTIFICATION_TP_CM_Abort, eSetBits);
			break;

		default:
			break;
	}
}

//---------------------------------------------------------------------------
// Other functions
//---------------------------------------------------------------------------

/**
 * @brief 	This function is used to processing J1939 messages.
 * @param 	rxMessage - A pointer to the receiving message's data.
 * @param	data - A pointer to the receiving data.
 * @retval	None.
 */
void J1939_messagesProcessing(void)
{
	J1939_status status = J1939_STATUS_OK;
	USH_CAN_rxHeaderTypeDef rxMessage = {0};
	uint8_t data[8] = {0};
	uint8_t pduFormat = 0;

	CAN_getRxMessage(USE_CAN, USE_CAN_FIFO, &rxMessage, data);

	pduFormat = (uint8_t)(rxMessage.ExtId >> 16U);
//	uint8_t pages = (uint8_t)(rxMessage->ExtId >> 24U) & J1939_PRIORITY_MASK;
//	uint8_t destinAddress = (uint8_t)(rxMessage->ExtId >> 8U);
//	uint8_t sourceAddress = (uint8_t)rxMessage->ExtId;

	if(J1939_state != J1939_STATE_UNINIT)
	{
		if(pduFormat == 0xEA)
		{
			status = J1939_readTP_connectionManagement(data);
			if(status == J1939_STATUS_OK)
			{
				// Start timeout timer
				osTimerStart(timeoutTimerHandle, J1939_MESSAGE_TIMEOUT);
			}

		} else if(pduFormat == 0xEB)
		{
			// Stop timeout timer
			osTimerStop(timeoutTimerHandle);

			// Read the receiving data
			status = J1939_readTP_dataTransfer(data);

			// If the receiving data isn't finished, start timeout timer
			if(status == J1939_STATUS_DATA_CONTINUE) osTimerStart(timeoutTimerHandle, J1939_MESSAGE_TIMEOUT);
		}
	}

	CAN_interruptEnable(USE_CAN, CAN_IT_RX_FIFO0_MSG_PENDING);
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
	canInit.Timings.BaudratePrescaler 	= CAN_BAUDRATE_PRESCALER;
	canInit.Timings.TimeSegment1 		= CAN_TIME_SEGMENT_1;
	canInit.Timings.TimeSegment2		= CAN_TIME_SEGMENT_2;
	canInit.Timings.ResynchJumpWidth	= CAN_RESYCH_JUMP_WIDTH;
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
	filterConfig.FilterBank				= 0U;
	filterConfig.FilterMode				= CAN_FILTER_MODE_IDMASK;
	filterConfig.FilterScale			= CAN_FILTERSCALE_32BIT;
	filterConfig.FilterActivation		= CAN_FILTER_ENABLE;
	CAN_filtersConfig(USE_CAN, &filterConfig);

	CAN_enable(USE_CAN);

	CAN_interruptEnable(USE_CAN, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING));

	J1939_state = J1939_STATE_NORMAL;
}

/**
 * @brief  	This function is used to initialize CAN modules global interrupts.
 * @note	This function should not be modified, when global interrupts of CAN modules are required,
 * 			this function must be implemented in the user file.
 * @retval	None.
 */
void CAN_initGlobalInterrupts(void)
{
	MISC_NVIC_setPriority(CAN1_TX_IRQn, CAN_TX_PREEMPPRIORITY, CAN_TX_SUBPRIORITY);
	MISC_NVIC_enableIRQ(CAN1_TX_IRQn);

	MISC_NVIC_setPriority(CAN1_RX0_IRQn, CAN_RX0_PREEMPPRIORITY, CAN_RX0_SUBPRIORITY);
	MISC_NVIC_enableIRQ(CAN1_RX0_IRQn);
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

	// Create the semaphore(s)
	// definition and creation of the receive messages semaphore
	osSemaphoreDef(receiveMessagesSem);
	receiveMessagesSemHandle = osSemaphoreCreate(osSemaphore(receiveMessagesSem), 1);

	// Create the timer(s)
	// definition and creation of the timeout timer for J1939 TP messages
	osTimerDef(Timeout, timeoutTimer_Callback);
	timeoutTimerHandle = osTimerCreate(osTimer(Timeout), osTimerOnce, (void *)&J1939_state);
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
	if(can == USE_CAN)
	{
		CAN_interruptDisable(USE_CAN, CAN_IT_RX_FIFO0_MSG_PENDING);

		osSemaphoreRelease(receiveMessagesSemHandle);
	}
}
