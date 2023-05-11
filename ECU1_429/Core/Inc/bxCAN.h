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
	J1939_STATE_UNINIT,
	J1939_STATE_NORMAL,
	J1939_STATE_TP_RECEIVING_BROADCAST,
	J1939_STATE_TP_SENDING_BROADCAST,
	J1939_STATE_TP_RECEIVING_PEER_TO_PEER,
	J1939_STATE_TP_SENDING_PEER_TO_PEER
} J1939_states;

//---------------------------------------------------------------------------
// External function prototypes
//---------------------------------------------------------------------------
void bxCAN_freeRtosInit(void);
void bxCAN_init(void);
void bxCAN_sendMessages(void const *argument);
void bxCAN_receiveMessages(void const *argument);
void timeoutTimer_Callback(void const *argument);
void J1939_messagesProcessing(void);

#endif /* __CAN_H */
