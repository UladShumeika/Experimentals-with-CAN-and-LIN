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
#include "24lc256.h"
#include "string.h"

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------

/* NOTE: When changing the I2C configuration, it is necessary to check
         the configuration of gpio, dma and global interrupts. */

#if defined(STM32F429xx)
	#define PRJ_EEPROM_MSG_SIZE										(PRJ_24LC256_MSG_SIZE)

	#define PRJ_EEPROM_I2C_USE										(PRJ_24LC256_I2C_USED)
	#define PRJ_EEPROM_I2C_CLOCK_SPEED								(PRJ_24LC256_I2C_CLOCK_SPEED)

	#define PRJ_EEPROM_I2C_DEVICE_ADDRESS							(0x50U)
	#define PRJ_EEPROM_I2C_DEVICE_ADDRESS_SIZE						(PRJ_I2C_ADDRESSING_MODE_7BIT)

	#define PRJ_EEPROM_WP_GPIO_PORT									PRJ_24LC256_WP_PORT
	#define PRJ_EEPROM_WP_GPIO_PIN									PRJ_24LC256_WP_PIN

	#define PRJ_EEPROM_I2C_CLOCK_ENABLE								__RCC_I2C2_CLOCK_ENABLE
	#define PRJ_EEPROM_DMA_CLOCK_ENABLE								__RCC_DMA1_CLOCK_ENABLE

	#define PRJ_EEPROM_DMA_STREAM_TX								DMA1_Stream7
	#define PRJ_EEPROM_DMA_CHANNEL_TX								PRJ_DMA_CHANNEL_7

	#define PRJ_EEPROM_DMA_STREAM_RX								DMA1_Stream2
	#define PRJ_EEPROM_DMA_CHANNEL_RX								PRJ_DMA_CHANNEL_7

	#define PRJ_EEPROM_DMA_STREAM_TX_IRQN							DMA1_Stream7_IRQn
	#define PRJ_EEPROM_DMA_STREAM_RX_IRQN							DMA1_Stream2_IRQn
	#define PRJ_EEPROM_I2C_EV_IRQN									I2C2_EV_IRQn
	#define PRJ_EEPROM_I2C_ER_IRQN									I2C2_ER_IRQn

	#define PRJ_EEPROM_DMA_TX_PREEMPTION_PRIORITY					(5U)
	#define PRJ_EEPROM_DMA_TX_SUBPRIORITY							(1U)

	#define PRJ_EEPROM_DMA_RX_PREEMPTION_PRIORITY					(5U)
	#define PRJ_EEPROM_DMA_RX_SUBPRIORITY							(0U)

	#define PRJ_EEPROM_I2C_EV_PREEMPTION_PRIORITY					(5U)
	#define PRJ_EEPROM_I2C_EV_SUBPRIORITY							(2U)

	#define PRJ_EEPROM_I2C_ER_PREEMPTION_PRIORITY					(5U)
	#define PRJ_EEPROM_I2C_ER_SUBPRIORITY							(1U)

#endif

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
static osThreadId m_eeprom_read_write_memory_handle = {0};

static prj_i2c_init_t m_i2c_init = {0};
static prj_dma_handler_t m_dma_tx = {0};
static prj_dma_handler_t m_dma_rx = {0};

//---------------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------------
static uint8_t data_write[] = "Hello, my eeprom, again. How are you? Tell me something about you! Test message! 123456789()";
static uint8_t data_read[PRJ_EEPROM_MSG_SIZE] = {0};

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
	/* Create the thread(s) */
	/* definition and creation of the read/write eeprom thread */
	osThreadDef(read_write_eeprom, eeprom_read_write_memory_task, osPriorityLow, 0, 128);
	m_eeprom_read_write_memory_handle = osThreadCreate(osThread(read_write_eeprom), NULL);
}

/*!
 * @brief Handle i2c event interrupt.
 */
void prj_eeprom_i2c_ev_irq_handler(void)
{
	prj_i2c_irq_handler(PRJ_EEPROM_I2C_USE);
}

/*!
 * @brief Handle i2c error interrupt.
 */
void prj_eeprom_i2c_er_irq_handler(void)
{
	prj_i2c_irq_handler(PRJ_EEPROM_I2C_USE);
}

/*!
 * @brief Handle dma stream rx global interrupt.
 */
void prj_eeprom_i2c_dma_rx_irq_handler(void)
{
	prj_dma_irq_handler(&m_dma_rx);
}

/*!
 * @brief Handle dma stream tx global interrupt.
 */
void prj_eeprom_i2c_dma_tx_irq_handler(void)
{
	prj_dma_irq_handler(&m_dma_tx);
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
	uint32_t status = PRJ_STATUS_OK;

	/* Initialize auxiliary peripherals */
	eeprom_init_auxiliary_peripherals();

	/* Initialize eeprom and its system buffers */
	status = prj_eeprom_24lc256_init(PRJ_EEPROM_I2C_DEVICE_ADDRESS);

	macro_prj_common_unused(status);

	/* Infinite loop */
	for(;;)
	{
		osDelay(1000);
	}
}

/*!
 * @brief Initialize auxiliary peripherals for eeprom memory.
 *
 * This function is used to initialize peripherals for work with EEPROM memory.
 * (gpio, dma, i2c and interrupts)
 *
 * @return @ref PRJ_STATUS_OK if gpio initialization was successful.
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
 * @brief Initialize gpio peripherals for EEPROM memory.
 *
 * @return @ref PRJ_STATUS_OK if gpio initialization was successful.
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
 * @brief Initialize dma peripherals for EEPROM memory.
 *
 * @return @ref PRJ_STATUS_OK if dma initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
static uint32_t eeprom_init_dma(void)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Enable DMA clock */
	PRJ_EEPROM_DMA_CLOCK_ENABLE();

	/* DMA tx interrupt init */
	MISC_NVIC_setPriority(PRJ_EEPROM_DMA_STREAM_TX_IRQN, PRJ_EEPROM_DMA_TX_PREEMPTION_PRIORITY, PRJ_EEPROM_DMA_TX_SUBPRIORITY);
	MISC_NVIC_enableIRQ(PRJ_EEPROM_DMA_STREAM_TX_IRQN);

	/* DMA tx init */
	m_dma_tx.p_dma_stream						= PRJ_EEPROM_DMA_STREAM_TX;
	m_dma_tx.dma_init.channel					= PRJ_EEPROM_DMA_CHANNEL_TX;
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
	MISC_NVIC_setPriority(PRJ_EEPROM_DMA_STREAM_RX_IRQN, PRJ_EEPROM_DMA_RX_PREEMPTION_PRIORITY, PRJ_EEPROM_DMA_RX_SUBPRIORITY);
	MISC_NVIC_enableIRQ(PRJ_EEPROM_DMA_STREAM_RX_IRQN);

	/* DMA rx init */
	m_dma_rx.p_dma_stream						= PRJ_EEPROM_DMA_STREAM_RX;
	m_dma_rx.dma_init.channel					= PRJ_EEPROM_DMA_CHANNEL_RX;
	m_dma_rx.dma_init.direction					= PRJ_DMA_PERIPH_TO_MEMORY;
	m_dma_rx.dma_init.periph_inc				= PRJ_DMA_PINC_DISABLE;
	m_dma_rx.dma_init.mem_inc					= PRJ_DMA_MINC_ENABLE;
	m_dma_rx.dma_init.periph_data_alignment		= PRJ_DMA_PERIPH_SIZE_BYTE;
	m_dma_rx.dma_init.mem_data_alignment		= PRJ_DMA_MEMORY_SIZE_BYTE;
	m_dma_rx.dma_init.mode						= PRJ_DMA_NORMAL_MODE;
	m_dma_rx.dma_init.priority					= PRJ_DMA_PRIORITY_LOW;
	m_dma_rx.dma_init.fifo_mode					= PRJ_DMA_FIFO_MODE_DISABLE;
	status = prj_dma_init(&m_dma_rx);

	/* Safe dma handlers' pointers */
	prj_eeprom_24lc256_dma_handlers_set(&m_dma_tx, &m_dma_rx);

	return status;
}

/*!
 * @brief Initialize i2c peripherals for EEPROM memory.
 *
 * @return @ref PRJ_STATUS_OK if i2c initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if there are problems with the input parameters.
 */
static uint32_t eeprom_init_i2c(void)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Enable I2C clock */
	PRJ_EEPROM_I2C_CLOCK_ENABLE();

	/* I2C init */
	m_i2c_init.p_i2c				= PRJ_EEPROM_I2C_USE;
	m_i2c_init.clock_speed			= PRJ_EEPROM_I2C_CLOCK_SPEED;
	m_i2c_init.duty_cycle			= PRJ_I2C_DUTYCYCLE_2;
	m_i2c_init.own_address_1		= 0U;
	m_i2c_init.addressing_mode		= PRJ_EEPROM_I2C_DEVICE_ADDRESS_SIZE;
	m_i2c_init.dual_address_mode 	= PRJ_I2C_DUAL_ADDRESS_DISABLE;
	m_i2c_init.own_address_2		= 0U;
	m_i2c_init.general_call_mode	= PRJ_I2C_GENERAL_CALL_DISABLE;
	m_i2c_init.nostretch_mode		= PRJ_I2C_NOSTRETCH_DISABLE;
	status = prj_i2c_init(&m_i2c_init);

	/* I2C EV interrupt init */
	MISC_NVIC_setPriority(PRJ_EEPROM_I2C_EV_IRQN, PRJ_EEPROM_I2C_EV_PREEMPTION_PRIORITY, PRJ_EEPROM_I2C_EV_SUBPRIORITY);
	MISC_NVIC_enableIRQ(PRJ_EEPROM_I2C_EV_IRQN);

	/* I2C ER interrupt init */
	MISC_NVIC_setPriority(PRJ_EEPROM_I2C_ER_IRQN, PRJ_EEPROM_I2C_ER_PREEMPTION_PRIORITY, PRJ_EEPROM_I2C_ER_SUBPRIORITY);
	MISC_NVIC_enableIRQ(PRJ_EEPROM_I2C_ER_IRQN);

	return status;
}
