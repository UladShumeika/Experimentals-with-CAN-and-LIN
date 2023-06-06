/**
  ******************************************************************************
  * @file    eeprom.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    31 May 2023
  *
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "eeprom.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------
#define PRJ_EEPROM_DMA_TX_PREEMPTION_PRIORITY					(5U)
#define PRJ_EEPROM_DMA_TX_SUBPRIORITY							(0U)

#define PRJ_EEPROM_DMA_RX_PREEMPTION_PRIORITY					(5U)
#define PRJ_EEPROM_DMA_RX_SUBPRIORITY							(0U)

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
static osThreadId m_eeprom_read_write_memory_handle = {0};

static prj_dma_handler_t m_dma_tx = {0};
static prj_dma_handler_t m_dma_rx = {0};
static prj_i2c_init_t m_i2c_init = {0};

//---------------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// Static functions declaration
//---------------------------------------------------------------------------
static void eeprom_read_write_memory_task(void const *p_argument);

static uint32_t eeprom_init_auxiliary_peripherals(void);
static uint32_t eeprom_init_gpio(void);
static uint32_t eeprom_init_dma(void);
static uint32_t eeprom_init_i2c(void);

//---------------------------------------------------------------------------
// API
//---------------------------------------------------------------------------

/*!
 * @brief Initialize FreeRTOS objects.
 *
 * This function is used to initialize FreeRTOS objects (threads, queues, sems and etc.).
 *
 * @return None.
 */
void prj_eeprom_freertos_init(void)
{
	// Create the thread(s)
	// definition and creation of the read/write eeprom thread
	osThreadDef(read_write_eeprom, eeprom_read_write_memory_task, osPriorityLow, 0, 128);
	m_eeprom_read_write_memory_handle = osThreadCreate(osThread(read_write_eeprom), NULL);
}

//---------------------------------------------------------------------------
// STATIC
//---------------------------------------------------------------------------

/*!
 * @brief Function implementing the read/write eeprom thread.
 *
 * @param	p_argument - Not used.
 *
 * @return	None.
 */
static void eeprom_read_write_memory_task(void const *p_argument)
{
	/* Infinite loop */
	for(;;)
	{

	}
}

/*!
 * @brief Initialize auxiliary peripherals for EEPROM memory.
 *
 * This function is used to initialize peripherals for work with EEPROM memory.
 * (GPIO, DMA, I2C and interrupts)
 *
 * @return @ref PRJ_STATUS_OK if GPIO initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
static uint32_t eeprom_init_auxiliary_peripherals(void)
{
	uint32_t status = PRJ_STATUS_OK;

	/* -------------------------- GPIO configuration -------------------------- */

	status = eeprom_init_gpio();

	/* -------------------------- DMA configuration --------------------------- */

	if(status == PRJ_STATUS_OK)
	{
		status = eeprom_init_dma();
	}
	else
	{
		; /* DO NOTHING */
	}

	/* -------------------------- I2C configuration --------------------------- */

	if(status == PRJ_STATUS_OK)
	{
		status = eeprom_init_i2c();
	}
	else
	{
		; /* DO NOTHING */
	}

	return status;
}

/*!
 * @brief Initialize GPIO peripherals for EEPROM memory.
 *
 * @return @ref PRJ_STATUS_OK if GPIO initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
static uint32_t eeprom_init_gpio(void)
{
	uint32_t status = PRJ_STATUS_OK;
	USH_GPIO_initTypeDef gpio_init = {0};

	/* Enable GPIOF clock */
	__RCC_GPIOF_CLOCK_ENABLE();

	/* I2C2 GPIO pins configuration
	   PF0	  ------> I2C_SDA
	   PF1    ------> I2C_SCL */
	gpio_init.GPIOx 		= GPIOF;
	gpio_init.Pin 			= (GPIO_PIN_0 | GPIO_PIN_1);
	gpio_init.Mode			= GPIO_MODE_ALTERNATE_OD;
	gpio_init.Pull			= GPIO_NOPULL;
	gpio_init.Speed			= GPIO_SPEED_VERY_HIGH;
	gpio_init.Alternate 	= GPIO_AF4_I2C2;
	status = GPIO_init(&gpio_init);

	return status;
}

/*!
 * @brief Initialize DMA peripherals for EEPROM memory.
 *
 * @return @ref PRJ_STATUS_OK if DMA initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
static uint32_t eeprom_init_dma(void)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Enable DMA1 clock */
	__RCC_DMA1_CLOCK_ENABLE();

	/* DMA tx interrupt init */
	MISC_NVIC_setPriority(DMA1_Stream7_IRQn, PRJ_EEPROM_DMA_TX_PREEMPTION_PRIORITY, PRJ_EEPROM_DMA_TX_SUBPRIORITY);
	MISC_NVIC_enableIRQ(DMA1_Stream7_IRQn);

	/* DMA tx init */
	m_dma_tx.p_dma_stream						= DMA1_Stream7;
	m_dma_tx.dma_init.channel					= PRJ_DMA_CHANNEL_7;
	m_dma_tx.dma_init.direction					= PRJ_DMA_MEMORY_TO_PERIPH;
	m_dma_tx.dma_init.periph_inc				= PRJ_DMA_PINC_DISABLE;
	m_dma_tx.dma_init.mem_inc					= PRJ_DMA_MINC_ENABLE;
	m_dma_tx.dma_init.periph_data_alignment		= PRJ_DMA_PERIPH_SIZE_BYTE;
	m_dma_tx.dma_init.mem_data_alignment		= PRJ_DMA_MEMORY_SIZE_BYTE;
	m_dma_tx.dma_init.mode						= PRJ_DMA_NORMAL_MODE;
	m_dma_tx.dma_init.priority					= PRJ_DMA_PRIORITY_LOW;
	m_dma_tx.dma_init.fifo_mode					= PRJ_DMA_FIFO_MODE_DISABLE;
	status = prj_dma_init(&m_dma_tx);

	/* DMA rx interrupt init */
	MISC_NVIC_setPriority(DMA1_Stream2_IRQn, PRJ_EEPROM_DMA_RX_PREEMPTION_PRIORITY, PRJ_EEPROM_DMA_RX_SUBPRIORITY);
	MISC_NVIC_enableIRQ(DMA1_Stream2_IRQn);

	/* DMA rx init */
	m_dma_rx.p_dma_stream						= DMA1_Stream2;
	m_dma_rx.dma_init.channel					= PRJ_DMA_CHANNEL_7;
	m_dma_rx.dma_init.direction					= PRJ_DMA_MEMORY_TO_PERIPH;
	m_dma_rx.dma_init.periph_inc				= PRJ_DMA_PINC_ENABLE;
	m_dma_rx.dma_init.mem_inc					= PRJ_DMA_MINC_DISABLE;
	m_dma_rx.dma_init.periph_data_alignment		= PRJ_DMA_PERIPH_SIZE_BYTE;
	m_dma_rx.dma_init.mem_data_alignment		= PRJ_DMA_MEMORY_SIZE_BYTE;
	m_dma_rx.dma_init.mode						= PRJ_DMA_NORMAL_MODE;
	m_dma_rx.dma_init.priority					= PRJ_DMA_PRIORITY_LOW;
	m_dma_rx.dma_init.fifo_mode					= PRJ_DMA_FIFO_MODE_DISABLE;
	status = prj_dma_init(&m_dma_rx);

	return status;
}

/*!
 * @brief Initialize I2C peripherals for EEPROM memory.
 *
 * @return @ref PRJ_STATUS_OK if I2C initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
static uint32_t eeprom_init_i2c(void)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Enable I2C2 clock */
	__RCC_I2C2_CLOCK_ENABLE();

	/* I2C init */
	m_i2c_init.p_i2c				= I2C2;
	m_i2c_init.clock_speed			= 400000U;
	m_i2c_init.duty_cycle			= PRJ_I2C_DUTYCYCLE_2;
	m_i2c_init.own_address_1		= 0U;
	m_i2c_init.addressing_mode		= PRJ_I2C_ADDRESSING_MODE_7BIT;
	m_i2c_init.dual_address_mode 	= PRJ_I2C_DUAL_ADDRESS_DISABLE;
	m_i2c_init.own_address_2		= 0U;
	m_i2c_init.general_call_mode	= PRJ_I2C_GENERAL_CALL_DISABLE;
	m_i2c_init.nostretch_mode		= PRJ_I2C_NOSTRETCH_DISABLE;
	status = prj_i2c_init(&m_i2c_init);

	return status;
}
