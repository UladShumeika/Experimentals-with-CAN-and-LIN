//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

//---------------------------------------------------------------------------
// Functions prototypes
//---------------------------------------------------------------------------

// GetIdleTaskMemory prototype (linked to static allocation support)
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

// GetTimerTaskMemory prototype (linked to static allocation support)
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

//---------------------------------------------------------------------------
// Static variables
//---------------------------------------------------------------------------
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

//---------------------------------------------------------------------------
// FreeRTOS functions
//---------------------------------------------------------------------------

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

//---------------------------------------------------------------------------
// Initialization functions
//---------------------------------------------------------------------------

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void freeRtosInit(void)
{
#ifdef HEARTBEAT
	HEARTBEAT_freeRtosInit();
#endif

#ifdef CAN
	bxCAN_freeRtosInit();
#endif

#ifdef EEPROM
	prj_eeprom_freertos_init();
#endif
}
