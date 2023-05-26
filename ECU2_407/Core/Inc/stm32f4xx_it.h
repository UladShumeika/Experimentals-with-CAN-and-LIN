//---------------------------------------------------------------------------
// Define to prevent recursive inclusion
//---------------------------------------------------------------------------
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

//---------------------------------------------------------------------------
// Exported functions prototypes
//---------------------------------------------------------------------------
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void CAN2_TX_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);

#endif /* __STM32F4xx_IT_H */
