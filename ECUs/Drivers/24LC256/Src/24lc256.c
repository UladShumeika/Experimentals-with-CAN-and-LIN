/**
  ******************************************************************************
  * @file    24lc256.c
  * @author  Ulad Shumeika
  * @version v1.0
  * @date    09 June 2023
  * @brief	 This file contains the implementation of functions for working with
  * 		 eeprom 24lc256.
  *
  ******************************************************************************
  */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "24lc256.h"
#include "ush_stm32f4xx_conf.h"
#include <stddef.h>
#include <string.h>

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------
#define PRJ_24LC256_PAGE_SIZE								(64U)
#define PRJ_24LC256_MAX_MEM_ADDRESS							(32767U)

/* Used to store data that is written only once and
 * does not change during the operation of the device (64 bytes) */
#define PRJ_24LC256_STATIC_DATA_SPACE_SIZE					(PRJ_24LC256_PAGE_SIZE)
#define PRJ_24LC256_STATIC_DATA_SPACE_BEGIN					(0x0000U)
#define PRJ_24LC256_STATIC_DATA_SPACE_END					(0x003FU)

/* The status buffer that stores a pointer to the index of the parameter buffer
 * that already stores a pointer to the actual data (32 bytes) */
#define PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_SIZE			(PRJ_24LC256_PAGE_SIZE / 2U / 2U)
#define PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_BEGIN			(0x0040U)
#define PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_END			(0x005FU)

/* The parameter buffer that stores a pointer to the actual data (32 bytes) */
#define PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_SIZE		(PRJ_24LC256_PAGE_SIZE / 2U / 2U)
#define PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_BEGIN		(0x0060U)
#define PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_END		(0x009FU)

/* Used to store the actual data */
#define PRJ_24LC256_DINAMIC_DATA_SPACE_SIZE					(32640U)
#define PRJ_24LC256_DINAMIC_DATA_SPACE_BEGIN				(0x0080U)
#define PRJ_24LC256_DINAMIC_DATA_SPACE_END					(PRJ_24LC256_MAX_MEM_ADDRESS)

#define PRJ_24LC256_MAX_NUM_RECORDS							(200U)  /* Used for status buffer */
#define PRJ_24LC256_TRIALS									(3U)
#define PRJ_24LC256_DELAY									(10U)

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
static prj_24lc256_dma_handlers_t m_dma_handlers = {0};
static prj_i2c_transmission_t m_i2c_tx = {0};
static prj_i2c_transmission_t m_i2c_rx = {0};

static prj_24lc256_system_t m_system_param = {0};

//---------------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------------
static uint16_t m_status_buffer[PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_SIZE] = {0};
static uint16_t m_parameter_buffer[PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_SIZE] = {0};
static uint8_t m_page_buffer[PRJ_24LC256_PAGE_SIZE] = {0};

//---------------------------------------------------------------------------
// Static functions declaration
//---------------------------------------------------------------------------
static void eeprom_24lc256_status_buffer_index_get(uint8_t* data, uint8_t data_size, prj_24lc256_system_t* system);

#if(PRJ_24LC256_WP_ENABLED == 1U)
	static void eeprom_24lc256_write_protection(uint32_t state);
#endif

//---------------------------------------------------------------------------
// API
//---------------------------------------------------------------------------

/*!
 * @brief Initialize 24lc256 memory.
 *
 * This function is used to capture status and parameter buffers.
 *
 * @param[in] dev_address	A target device address.
 *
 * @return @ref PRJ_STATUS_OK if initialization was successful.
 * @return @ref PRJ_STATUS_ERROR if the data was not read from memory.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_init(uint8_t dev_address)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Enable write protection */
#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
#endif

	/* Fill in the general parameters */
	m_i2c_rx.p_i2c				= PRJ_24LC256_I2C_USED;
	m_i2c_rx.dev_address		= dev_address;
	m_i2c_rx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;

	/* Capture status buffer from memory */
	m_i2c_rx.mem_address		= PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_BEGIN;
	m_i2c_rx.p_data				= (uint8_t*)m_status_buffer;
	m_i2c_rx.data_size 			= sizeof(m_status_buffer);
	status = prj_i2c_read_dma(&m_i2c_rx);

	/* Capture parameter buffer from memory */
	m_i2c_rx.mem_address		= PRJ_24LC256_DINAMIC_DATA_PARAMETER_SPACE_BEGIN;
	m_i2c_rx.p_data				= (uint8_t*)m_parameter_buffer;
	m_i2c_rx.data_size 			= sizeof(m_parameter_buffer);
	status = prj_i2c_read_dma(&m_i2c_rx);

	if(status == PRJ_STATUS_OK)
	{
		/* Search for the index of the status buffer that contains the maximum value */
		eeprom_24lc256_status_buffer_index_get(m_status_buffer,
											   PRJ_24LC256_DINAMIC_DATA_STATUS_SPACE_SIZE,
											   &m_system_param);
	}
	else
	{
		/* DO NOTHING */
	}

	return status;
}

/*!
 * @brief Set dma handlers' pointers in 24LC256 structure.
 *
 * This function is used to get and to safe dma handlers' pointers and to link dma handlers and i2c transmission structures.
 *
 * @param[in] p_dma_tx		A ponter to dma tx handler.
 * @param[in] p_dma_rx		A ponter to dma rx handler.
 *
 * @return None.
 */
void prj_eeprom_24lc256_dma_handlers_set(prj_dma_handler_t* p_dma_tx, prj_dma_handler_t* p_dma_rx)
{
	/* Set DMA handlers' pointers */
	m_dma_handlers.p_dma_tx = p_dma_tx;
	m_dma_handlers.p_dma_rx = p_dma_rx;

	/* Link DMA tx handler and i2c tx transmission */
	m_i2c_tx.p_dma										= m_dma_handlers.p_dma_tx;
	m_dma_handlers.p_dma_tx->p_controls_peripherals 	= (void*)&m_i2c_tx;

	/* Link DMA rx handler and i2c rx transmission */
	m_i2c_rx.p_dma										= m_dma_handlers.p_dma_rx;
	m_dma_handlers.p_dma_rx->p_controls_peripherals 	= (void*)&m_i2c_rx;
}

/*!
 * @brief Check device availability on the bus.
 *
 * This function is used to check the specified device availability several times.
 * The number of checks is defined in @ref PRJ_24LC256_TRIALS.
 *
 * @param[in] dev_address	A target device address.
 *
 * @return @ref PRJ_STATUS_OK if the device is available.
 * @return @ref PRJ_STATUS_ERROR if a null pointer to the peripheral is passed
 * 		   or the device is not detected.
 * @return @ref PRJ_STATUS_TIMEOUT if the timeout has passed.
 */
uint32_t prj_eeprom_24lc256_connect_test(uint8_t dev_address)
{
	uint32_t status = PRJ_STATUS_OK;

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
#endif

	status = prj_i2c_is_device_ready(PRJ_24LC256_I2C_USED, dev_address, PRJ_24LC256_TRIALS);

	return status;
}

/*!
 * @brief Erase the memory
 *
 * This function is used to erase all memory.
 *
 * @param[in] dev_address	A target device address.
 *
 * @return @ref PRJ_STATUS_OK if memory clearing was successful.
 * @return @ref PRJ_STATUS_ERROR if the device is not detected.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_erase_memory(uint8_t dev_address)
{
	uint32_t status = PRJ_STATUS_OK;
	uint16_t mem_address = 0x0000U;

	/* Fill in the page buffer */
	memset(m_page_buffer, 0xFFU, PRJ_24LC256_PAGE_SIZE);

	/* Fill in the i2c tx structure */
	m_i2c_tx.p_i2c				= PRJ_24LC256_I2C_USED;
	m_i2c_tx.dev_address		= dev_address;
	m_i2c_tx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;
	m_i2c_tx.p_data				= m_page_buffer;
	m_i2c_tx.data_size 			= PRJ_24LC256_PAGE_SIZE;

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_DISABLE);
#endif

	/* Erase every memory page */
	while((mem_address < PRJ_24LC256_MAX_MEM_ADDRESS) && (status == PRJ_STATUS_OK))
	{
		m_i2c_tx.mem_address = mem_address;
		mem_address = mem_address + PRJ_24LC256_PAGE_SIZE;

		status = prj_i2c_write_dma(&m_i2c_tx);

		MISC_timeoutDelay(PRJ_24LC256_DELAY);
	}

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
#endif

	return status;
}

/*!
 * @brief Read the data in the static data space.
 *
 * This function is used to read data in the static space.
 *
 * @param[in]  dev_address	A target device address.
 * @param[out] data			A pointer to the array in which to store the data.
 * @param[in]  data_size	Size of the array.
 *
 * @return @ref PRJ_STATUS_OK if memory reading was successful.
 * @return @ref PRJ_STATUS_ERROR if the device is not detected or the pointer is NULL or
 * 		   array size is larger than static data space.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_read_static(uint8_t dev_address, void* data, uint8_t data_size)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Check the pointer and data size */
	if((data == NULL) || (data_size >= PRJ_24LC256_STATIC_DATA_SPACE_SIZE))
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		/* DO NOTHING */
	}

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
#endif

	if(status == PRJ_STATUS_OK)
	{
		/* Fill i2c rx structure */
		m_i2c_rx.p_i2c				= PRJ_24LC256_I2C_USED;
		m_i2c_rx.dev_address		= dev_address;
		m_i2c_rx.mem_address		= PRJ_24LC256_STATIC_DATA_SPACE_BEGIN;
		m_i2c_rx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;
		m_i2c_rx.p_data				= (uint8_t*)data;
		m_i2c_rx.data_size 			= data_size;
		status = prj_i2c_read_dma(&m_i2c_rx);
	}
	else
	{
		/* DO NOTHING */
	}

	return status;
}

/*!
 * @brief Write the data in the static data space.
 *
 * This function is used to write data in the static space.
 *
 * @param[in] dev_address	A target device address.
 * @param[in] data			A pointer to the array in which to store the data.
 * @param[in] data_size	 	Size of the array.
 *
 * @return @ref PRJ_STATUS_OK if memory writing was successful.
 * @return @ref PRJ_STATUS_ERROR if the device is not detected or the pointer is NULL or
 * 		   array size is larger than static data space.
 * @return @ref PRJ_STATUS_TIMEOUT if a timeout is detected on any flag.
 */
uint32_t prj_eeprom_24lc256_write_static(uint8_t dev_address, void* data, uint8_t data_size)
{
	uint32_t status = PRJ_STATUS_OK;

	/* Check the pointer and data size */
	if((data == NULL) || (data_size >= PRJ_24LC256_STATIC_DATA_SPACE_SIZE))
	{
		status = PRJ_STATUS_ERROR;
	}
	else
	{
		/* DO NOTHING */
	}

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_DISABLE);
#endif

	if(status == PRJ_STATUS_OK)
	{
		/* Fill i2c tx structure */
		m_i2c_tx.p_i2c				= PRJ_24LC256_I2C_USED;
		m_i2c_tx.dev_address		= dev_address;
		m_i2c_tx.mem_address		= PRJ_24LC256_STATIC_DATA_SPACE_BEGIN;
		m_i2c_tx.mem_address_size	= PRJ_I2C_MEM_ADDRESS_SIZE_16BIT;
		m_i2c_tx.p_data				= (uint8_t*)data;
		m_i2c_tx.data_size 			= data_size;
		status = prj_i2c_write_dma(&m_i2c_tx);

		MISC_timeoutDelay(PRJ_24LC256_DELAY);
	}
	else
	{
		/* DO NOTHING */
	}

#if(PRJ_24LC256_WP_ENABLED == 1U)
	eeprom_24lc256_write_protection(PRJ_STATE_ENABLE);
#endif

	return status;
}

//---------------------------------------------------------------------------
// STATIC
//---------------------------------------------------------------------------

/*!
 * @brief Search for the index in the status buffer.
 *
 * This function is used to search for the index of the status buffer that contains the maximum value
 * and to safe system parameters.
 *
 * @param[in] 	data		A pointer to the status buffer.
 * @param[in]	data_size	The status buffer size.
 * @param[out]	system		A pointer to the structure in which the system parameters will be stored.
 *
 * @return None.
 */
static void eeprom_24lc256_status_buffer_index_get(uint8_t* data, uint8_t data_size, prj_24lc256_system_t* system)
{
	uint8_t index = 0;
	uint8_t max_value = data[0];

	if((data != NULL) && (system != NULL))
	{
		for(uint8_t i = 1; i < data_size; i++)
		{
			if(max_value < data[i])
			{
				max_value = data[i];
				index = i;
			}
			else
			{
				/* DO NOTHING */
			}
		}

		/* Safe data */
		system->buffer_index = index;
		system->record_number = max_value;
	}
	else
	{
		/* DO NOTHING */
	}
}

#if(PRJ_24LC256_WP_ENABLED == 1U)
/*!
 * @brief Enable or disable eeprom write protection.
 *
 * @param state[in] 	Enable or disable state.
 *
 * @return None.
 */
static void eeprom_24lc256_write_protection(uint32_t state)
{
	if(state == PRJ_STATE_ENABLE)
	{
		GPIO_writeBits(PRJ_24LC256_WP_PORT, PRJ_24LC256_WP_PIN, GPIO_PIN_SET);
	}
	else
	{
		GPIO_writeBits(PRJ_24LC256_WP_PORT, PRJ_24LC256_WP_PIN, GPIO_PIN_RESET);
	}
}
#endif
