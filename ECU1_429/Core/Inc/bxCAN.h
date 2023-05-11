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
// External function prototypes
//---------------------------------------------------------------------------
void bxCAN_freeRtosInit(void);
void bxCAN_sendMessages(void const *argument);
void bxCAN_receiveMessages(void const *argument);
void timeoutTimer_Callback(void const *argument);

#endif /* __CAN_H */
