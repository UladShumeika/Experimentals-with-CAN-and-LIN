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
	J1939_NOTIFICATION_TP_CM_Abort,
	J1939_NOTIFICATION_TP_CM_BAM,
	J1939_NOTIFICATION_TP_CM_EndOfMsgACK,
	J1939_NOTIFICATION_TP_CM_CTS,
	J1939_NOTIFICATION_TP_CM_RTS
} J1939_notificationsTypes;

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------

/* Initialization */
void bxCAN_freeRtosInit(void);

void bxCAN_sendMessages(void const *argument);
void bxCAN_receiveMessages(void const *argument);
void timeoutTimer_Callback(void const *argument);
void J1939_messagesProcessing(void);
/* Driver functions */

#endif /* __CAN_H */
