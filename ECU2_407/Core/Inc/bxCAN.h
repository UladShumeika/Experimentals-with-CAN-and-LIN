//---------------------------------------------------------------------------
// Define to prevent recursive inclusion
//---------------------------------------------------------------------------
#ifndef __CAN_H
#define __CAN_H

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "main.h"

//---------------------------------------------------------------------------
// Structures and enumerations
//---------------------------------------------------------------------------

/**
 * @brief SAE J1939 protocol states enumeration.
 */
typedef enum
{
	J1939_STATE_UNINIT,							/* CAN bus and J1939 protocol isn't initialized	*/
	J1939_STATE_NORMAL,							/* For sending CAN message <= 8 bytes */
	J1939_STATE_TP_RECEIVING_BROADCAST,			/* Transport protocol connection is in broadcast session (receiving) */
	J1939_STATE_TP_SENDING_BROADCAST,			/* Transport protocol connection is in broadcast session (sending) */
	J1939_STATE_TP_RECEIVING_PEER_TO_PEER,		/* Transport protocol connection is in peer-to-peer session (receiving) */
	J1939_STATE_TP_SENDING_PEER_TO_PEER			/* Transport protocol connection is in peer-to-peer session (receiving) */
} J1939_states;

/**
 * @brief SAE J1939 notification types enumeration.
 */
typedef enum
{
	J1939_NOTIFICATION_TP_DATA_TRANSFER,		/* For sending data transfer packages */
	J1939_NOTIFICATION_TP_CM_Abort,				/* For sending Abort packages */
	J1939_NOTIFICATION_TP_CM_BAM,				/* For sending BAM packages */
	J1939_NOTIFICATION_TP_CM_EndOfMsgACK,		/* For sending EndOfMsgACK packages */
	J1939_NOTIFICATION_TP_CM_CTS,				/* For sending CTS packages */
	J1939_NOTIFICATION_TP_CM_RTS,				/* For sending RTS packages */
	J1939_NOTIFICATION_NORMAL					/* For sending packages <= 8 bytes */
} J1939_notificationsTypes;

/**
 * @brief The structure for storing messages which is or less 8 bytes.
 */
typedef struct
{
	uint8_t* data;							/* A pointer to the sending data */
	USH_CAN_txHeaderTypeDef txMessage;		/* A structure for CAN frame settings */
} dataStructure;

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

/* Initialization */
void bxCAN_freeRtosInit(void);

/* FreeRTOS tasks */
void bxCAN_receiveMessages(void const *argument);
void bxCAN_sendMessages(void const *argument);
void timeoutTimer_Callback(void const *argument);

/* Driver functions */
void J1939_sendMessage(uint8_t* data, uint16_t dataSize, uint8_t destinationAddress, uint32_t PGN);
void applicationTask(void const *argument);

#endif /* __CAN_H */
