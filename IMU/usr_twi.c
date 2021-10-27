/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *  KU Leuven - Technology Campus Gent,
 *  Gebroeders De Smetstraat 1,
 *  B-9000 Gent, Belgium
 *
 *         File: usr_twi.c
 *      Created: YYYY-MM-DD
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: I2C communication
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

#include "usr_twi.h"
#include "imu.h"

/* TWI instance. */
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

volatile bool twi_tx_done = false;
volatile bool twi_rx_done = false;

/* I2C Initialisation */
/**
 * @brief UART initialization.
 */
uint32_t twi_open (void)
{
    ret_code_t err_code;

	
    const nrf_drv_twi_config_t twi_imu_config = {
       .scl                = USR_TWI_SCL,		// PIN 27
       .sda                = USR_TWI_SDA,		// PIN 26
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_imu_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
		
		return NRF_SUCCESS;
}

uint32_t twi_close(void)
{
    nrf_drv_twi_disable(&m_twi);

    nrf_drv_twi_uninit(&m_twi);

    return NRF_SUCCESS;
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
	switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    twi_rx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    twi_rx_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
						NRF_LOG_INFO("Address NACK")
            APP_ERROR_CHECK(NRF_ERROR_INVALID_ADDR);
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
						NRF_LOG_INFO("Data NACK");
            APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
            break;
        default:
            break;
    }
}

/* Low level write I2C functionality */
ret_code_t i2c_write_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, const uint8_t* data, uint32_t len, bool stop)
{   
		ret_code_t err_code;
		
		const uint8_t buf_len = len+1; // Register address + number of bytes
    uint8_t tx_buf[buf_len];

    tx_buf[0] = sub_address;
    
    memcpy(tx_buf+1, data, len); // Shift the data to make place for subaddress
	
		// Open the I2C connection
		err_code = twi_open();
    APP_ERROR_CHECK(err_code);
	
		err_code = nrf_drv_twi_tx(twi_handle, address, tx_buf, buf_len, stop);  
		APP_ERROR_CHECK(err_code);
	
		do{
			idle_state_handle();
		}
		while(twi_tx_done == false);
		
		// Close the I2C connection
		err_code = twi_close();
    APP_ERROR_CHECK(err_code);
	
		twi_tx_done = false;
	
		return err_code;
}

/* Low level read I2C functionality */
ret_code_t i2c_read_bytes(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t * dest, uint8_t dest_count)
{   
		ret_code_t err_code;  
	
		uint8_t tx_buf[1];
		tx_buf[0] = sub_address;
	
		// Setting up transfer
    nrf_drv_twi_xfer_desc_t xfer_desc;
    xfer_desc.address = address;
    xfer_desc.type = NRF_DRV_TWI_XFER_TXRX;
    xfer_desc.primary_length = sizeof(tx_buf);
		xfer_desc.secondary_length = dest_count;
    xfer_desc.p_primary_buf = tx_buf;
		xfer_desc.p_secondary_buf = dest;
	
	
		// Open the I2C connection
		err_code = twi_open();
    APP_ERROR_CHECK(err_code);
	
		// Transferring
    err_code = nrf_drv_twi_xfer(&m_twi, &xfer_desc, NRF_DRV_TWI_FLAG_TX_NO_STOP);
		APP_ERROR_CHECK(err_code);
		
		do{
			idle_state_handle();
		}
		while(twi_rx_done == false);
			
		// Close the I2C connection
		err_code = twi_close();
    APP_ERROR_CHECK(err_code);

    twi_rx_done = false;
	
		return err_code;
}

/* Low level read I2C functionality */
ret_code_t i2c_read_bytes_dma(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t * dest, uint8_t dest_count)
{   
		ret_code_t err_code;  
	
		uint8_t tx_buf[1];
		tx_buf[0] = sub_address;
	
		// Setting up transfer
    nrf_drv_twi_xfer_desc_t xfer_desc;
    xfer_desc.address = address;
    xfer_desc.type = NRF_DRV_TWI_XFER_TXRX;
    xfer_desc.primary_length = sizeof(tx_buf);
		xfer_desc.secondary_length = dest_count;
    xfer_desc.p_primary_buf = tx_buf;
		xfer_desc.p_secondary_buf = dest;
	
		// Transferring
    err_code = nrf_drv_twi_xfer(&m_twi, &xfer_desc, NRF_DRV_TWI_FLAG_TX_NO_STOP);
		APP_ERROR_CHECK(err_code);
	
		return err_code;
}
