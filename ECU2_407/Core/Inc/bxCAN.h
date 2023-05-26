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
 * @brief SAE J1939 notification types enumeration.
 */
typedef enum
{
	J1939_NOTIFICATION_NORMAL,		/* For sending <= 8 bytes messages */
	J1939_NOTIFICATION_BAM,			/* For sending BAM messages */
	J1939_NOTIFICATION_RTS,			/* For sending RTS messages */
	J1939_NOTIFICATION_CTS,			/* For sending CTS messages */
	J1939_NOTIFICATION_EOM,			/* For sending EndOfMsgACK messages */
	J1939_NOTIFICATION_ABORT,		/* For sending Abort messages */
	J1939_NOTIFICATION_DATA,		/* For sending data transfer messages */
} J1939_notificationsTypes;

/**
 * @brief The structure for storing messages which is or less 8 bytes.
 */
typedef struct
{
	uint8_t* data;							/* A pointer to the sending data */
	USH_CAN_txHeaderTypeDef txMessage;		/* A structure for CAN frame settings */
} dataStructure;

/**
 * @brief J1939 data message structure.
 */
typedef struct
{
	uint8_t *message;						/* The pointer to received message */
	uint16_t sizeMessage;					/* The size of received message */
} J1939_message;

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
