//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "bxCAN.h"
#include "SAE_J1939_21_Transport_Layer.h"
#include "SAE_J1939_81_Network_Management_Layer.h"
#include <limits.h>
#include <string.h>

//---------------------------------------------------------------------------
// Configuration section
//---------------------------------------------------------------------------

/* NOTE: When changing the CAN bus configuration, it is necessary to check
 * 		 the configuration of the bus(including filters) and global interrupts.
 */

#if defined(STM32F429xx)
	#define CURRENT_ECU_ADDRESS					(101U)
	#define ECU2_ADDRESS						(202U)

	#define USE_CAN								(CAN1)
	#define USE_CAN_FIFO						(CAN_FILTER_FIFO_0)
	#define USE_FILTER_BANK						(0U)

	#define CAN_BAUDRATE_PRESCALER				(10U)			// Bit rate 250 kbit/s at PCLK1 = 45 MHz
	#define CAN_TIME_SEGMENT_1					(CAN_TS1_TQ15)
	#define CAN_TIME_SEGMENT_2					(CAN_TS2_TQ2)
	#define CAN_RESYCH_JUMP_WIDTH				(CAN_SJW_TQ1)

	#define CAN_TX_IRQn							(CAN1_TX_IRQn)
	#define CAN_RX0_IRQn						(CAN1_RX0_IRQn)

	#define CAN_TX_PREEMPPRIORITY				(5U)
	#define CAN_TX_SUBPRIORITY					(0U)

	#define CAN_RX0_PREEMPPRIORITY				(5U)
	#define CAN_RX0_SUBPRIORITY					(0U)

#elif defined(STM32F407xx)
	#define CURRENT_ECU_ADDRESS					(202U)
	#define ECU1_ADDRESS						(101U)

	#define USE_CAN								(CAN2)
	#define USE_CAN_FIFO						(CAN_FILTER_FIFO_0)
	#define USE_FILTER_BANK						(15U)

	#define CAN_BAUDRATE_PRESCALER				(12U)			// Bit rate 250 kbit/s at PCLK1 = 42 MHz
	#define CAN_TIME_SEGMENT_1					(CAN_TS1_TQ11)
	#define CAN_TIME_SEGMENT_2					(CAN_TS2_TQ2)
	#define CAN_RESYCH_JUMP_WIDTH				(CAN_SJW_TQ1)

	#define CAN_TX_IRQn							(CAN2_TX_IRQn)
	#define CAN_RX0_IRQn						(CAN2_RX0_IRQn)

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
static osThreadId applicationHandle;
static osTimerId timeoutTimerHandle;
osMessageQId fromCanToApplicationHandle;
osPoolId J1939_messageStructureHandle;

//---------------------------------------------------------------------------
// Structure definitions
//---------------------------------------------------------------------------
USH_CAN_settingsTypeDef canInit = {0};
static USH_CAN_filterTypeDef filterConfig = {0};

//---------------------------------------------------------------------------
// Static function prototypes
//---------------------------------------------------------------------------
static void bxCAN_init(void);
static void J1939_messagesProcessing(void);

//---------------------------------------------------------------------------
// Structure definitions
//---------------------------------------------------------------------------
static dataStructure txData = {0};

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
	J1939_setCurrentECUAddress(CURRENT_ECU_ADDRESS);

	// Infinite loop
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

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
	J1939_status status = J1939_STATUS_DATA_CONTINUE;

	// Infinite loop
	for(;;)
	{
		xTaskNotifyWait(0, ULONG_MAX, &notifiedValue, portMAX_DELAY);

//		switch(notifiedValue)
//		{
//			case J1939_NOTIFICATION_TP_DATA_TRANSFER:
//				if(J1939_STATE_TP_SENDING_BROADCAST)
//				{
//					status = J1939_sendTP_dataTransfer(J1939_BROADCAST_ADDRESS);
//				} else
//				{
//					//J1939_sendTP_dataTransfer(destinationAddress);
//				}
//
//				if(status == J1939_STATUS_DATA_CONTINUE)
//				{
//					osTimerStart(timeoutTimerHandle, J1939_MESSAGE_PACKET_FREQ);
//				} else
//				{
//					J1939_state = J1939_STATE_NORMAL;
//					J1939_cleanTPstructures();
//				}
//				break;
//
//			case J1939_NOTIFICATION_TP_CM_Abort:
//				// send Abort
//				break;
//
//			case J1939_NOTIFICATION_TP_CM_BAM:
//				J1939_sendTP_connectionManagement(J1939_BROADCAST_ADDRESS);
//				osTimerStart(timeoutTimerHandle, J1939_MESSAGE_PACKET_FREQ);
//				break;
//
//			case J1939_NOTIFICATION_TP_CM_CTS:
//				// send CTS
//				break;
//
//			case J1939_NOTIFICATION_TP_CM_RTS:
//				// send RTS
//				break;
//
//			case J1939_NOTIFICATION_TP_CM_EndOfMsgACK:
//				// send END
//				break;
//
//			case J1939_NOTIFICATION_NORMAL:
//				CAN_addTxMessage(USE_CAN, &txData.txMessage, txData.data);
//				break;
//
//			default:
//				break;
//		}
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
}

/**
  * @brief 	Function implementing the application thread.
  * @param	argument - Not used.
  * @retval	None.
  */
void applicationTask(void const *argument)
{
	osEvent evt;
	J1939_message *message;

#if defined(STM32F407xx)

	uint8_t testData[] = "This is test message for TP..";
	uint16_t dataSize = strlen((char*)testData);
	uint32_t PGN = 0xFEFE;

	J1939_sendMessage(testData, dataSize, J1939_BROADCAST_ADDRESS, PGN);
//	J1939_sendMessage(testData, dataSize, ECU1_ADDRESS, PGN);
#endif

	for(;;)
	{
		evt = osMessageGet(fromCanToApplicationHandle, osWaitForever);

		if(evt.status == osEventMessage)
		{
			message = evt.value.p;
			(void)message;
		}
	}
}

//---------------------------------------------------------------------------
// Other functions
//---------------------------------------------------------------------------

/**
 * @brief	This function is used to send message via J1939 protocol.
 * @param 	data - A pointer to the sending data.
 * @param 	dataSize - A size of the sending data.
 * @param 	destinationAddress - A destination address(255 for broadcast).
 */
void J1939_sendMessage(uint8_t* data, uint16_t dataSize, uint8_t destinationAddress, uint32_t PGN)
{
	if(dataSize > 8)
	{
		if(destinationAddress == J1939_BROADCAST_ADDRESS)
		{
//			J1939_fillTPstructures(data, dataSize, PGN, J1939_CONTROL_BYTE_TP_CM_BAM);
//			J1939_state = J1939_STATE_TP_SENDING_BROADCAST;

			xTaskNotify(sendMessagesHandle, J1939_NOTIFICATION_BAM, eSetBits);
		} else	// peer-to-peer connection
		{
//			J1939_fillTPstructures(data, dataSize, PGN, J1939_CONTROL_BYTE_TP_CM_RTS);
//			J1939_state = J1939_STATE_TP_SENDING_PEER_TO_PEER;

			xTaskNotify(sendMessagesHandle, J1939_NOTIFICATION_RTS, eSetBits);
		}
	} else
	{
		txData.data = data;
		txData.txMessage.StdId = 0x0001;
		txData.txMessage.IDE = CAN_ID_STD;
		txData.txMessage.RTR = CAN_RTR_DATA;
		txData.txMessage.DLC = dataSize;

		xTaskNotify(sendMessagesHandle, J1939_NOTIFICATION_NORMAL, eSetBits);
	}
}

/**
 * @brief 	This function is used to processing J1939 messages.
 * @param 	rxMessage - A pointer to the receiving message's data.
 * @param	data - A pointer to the receiving data.
 * @retval	None.
 */
void J1939_messagesProcessing(void)
{
//	J1939_status status = J1939_STATUS_OK;
	USH_CAN_rxHeaderTypeDef rxMessage = {0};
	uint8_t data[8] = {0};
	uint8_t pduFormat = 0, destinAddress = 0;

	CAN_getRxMessage(USE_CAN, USE_CAN_FIFO, &rxMessage, data);

	pduFormat = (uint8_t)(rxMessage.ExtId >> 16U);
//	uint8_t pages = (uint8_t)(rxMessage->ExtId >> 24U) & J1939_PRIORITY_MASK;
	destinAddress = (uint8_t)(rxMessage.ExtId >> 8U);
//	uint8_t sourceAddress = (uint8_t)rxMessage->ExtId;

	// Process message when the CAN bus and the J1939 protocol is initialized
	if(J1939_state != J1939_STATE_UNINIT)
	{
		// Process message when destination address is J1939_BROADCAST_ADDRESS or CURRENT_ECU_ADDRESS
		if((destinAddress == J1939_BROADCAST_ADDRESS) || (destinAddress == CURRENT_ECU_ADDRESS))
		{
			if(pduFormat == J1939_CONNECTION_MANAGEMENT)
			{
//				status = J1939_readTP_connectionManagement(data);
//				if(status == J1939_STATUS_OK)
//				{
//					// Start timeout timer
//					osTimerStart(timeoutTimerHandle, J1939_MESSAGE_TIMEOUT);
//				} else if((status == J1939_STATUS_DATA_ABORT) && ((J1939_state == J1939_STATE_TP_SENDING_BROADCAST) || \
//						                                          (J1939_state == J1939_STATE_TP_SENDING_PEER_TO_PEER)))
//				{
//					J1939_state = J1939_STATE_NORMAL;
//					J1939_cleanTPstructures();
//				}

			} else if(pduFormat == J1939_DATA_TRANSFER)
			{
				// Stop timeout timer
				osTimerStop(timeoutTimerHandle);

				// Read the receiving data
//				status = J1939_readTP_dataTransfer(data);

				// If the receiving data isn't finished, start timeout timer
//				if(status == J1939_STATUS_DATA_CONTINUE) osTimerStart(timeoutTimerHandle, J1939_MESSAGE_TIMEOUT);
			}
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
	filterConfig.FilterBank				= USE_FILTER_BANK;
	filterConfig.FilterMode				= CAN_FILTER_MODE_IDMASK;
	filterConfig.FilterScale			= CAN_FILTERSCALE_32BIT;
	filterConfig.FilterActivation		= CAN_FILTER_ENABLE;
	filterConfig.SlaveStartFilterBank	= 7U;
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
	MISC_NVIC_setPriority(CAN_TX_IRQn, CAN_TX_PREEMPPRIORITY, CAN_TX_SUBPRIORITY);
	MISC_NVIC_enableIRQ(CAN_TX_IRQn);

	MISC_NVIC_setPriority(CAN_RX0_IRQn, CAN_RX0_PREEMPPRIORITY, CAN_RX0_SUBPRIORITY);
	MISC_NVIC_enableIRQ(CAN_RX0_IRQn);
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
	osThreadDef(receiveMessages, bxCAN_receiveMessages, osPriorityBelowNormal, 0, 128);
	receiveMessagesHandle = osThreadCreate(osThread(receiveMessages), NULL);

	// definition and creation of the application thread
	osThreadDef(application, applicationTask, osPriorityLow, 0, 128);
	applicationHandle = osThreadCreate(osThread(application), NULL);

	// Create the timer(s)
	// definition and creation of the timeout timer for J1939 TP messages
	osTimerDef(Timeout, timeoutTimer_Callback);
	timeoutTimerHandle = osTimerCreate(osTimer(Timeout), osTimerOnce, (void *)&J1939_state);

	// Create the queue(s)
	// definition and creating of the queue for sending data from J1939 protocol to the application.
	osMessageQDef(fromCanToApplication, 1, J1939_message);
	fromCanToApplicationHandle = osMessageCreate(osMessageQ(fromCanToApplication), NULL);

	// Create the memory pool(s)
	// definition and creating of messageStructHandle
	osPoolDef(messagePool, 1, J1939_message);
	J1939_messageStructureHandle = osPoolCreate(osPool(messagePool));
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
