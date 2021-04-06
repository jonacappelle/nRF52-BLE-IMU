#include "usr_twi.h"


/* TWI instance. */
//#define MAX_PENDING_TRANSACTIONS    5
//NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;


volatile bool twi_tx_done = false;
volatile bool twi_rx_done = false;


extern void idle_state_handle(void);

/* I2C Initialisation */
/**
 * @brief UART initialization.
 */
uint32_t twi_open (void)
{
    ret_code_t err_code;

	
    const nrf_drv_twi_config_t twi_imu_config = {
       .scl                = ARDUINO_SCL_PIN,		// PIN 27
       .sda                = ARDUINO_SDA_PIN,		// PIN 26
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_imu_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
		
		// Use twi manager for scheduled transactions
//		err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
//    APP_ERROR_CHECK(err_code);

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
//										return 0;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    twi_tx_done = true;
//										return 0;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    twi_rx_done = true;
//										return 0;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    twi_rx_done = true;
//										return 0;
                    break;
                default:
//										return -1;
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
						NRF_LOG_INFO("Address NACK")
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
						NRF_LOG_INFO("Data NACK");
            break;
        default:
            break;
    }
//    switch (p_event->type)
//    {
//        case NRF_DRV_TWI_EVT_DONE:
//            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
//            {
//                //data_handler(data[0]);
//            }
//            m_xfer_done = true;
//            break;
//				case NRF_DRV_TWI_EVT_ADDRESS_NACK:
//						NRF_LOG_INFO("Address NACK");
//						break;
//				case NRF_DRV_TWI_EVT_DATA_NACK:
//						NRF_LOG_INFO("Data NACK");
//						break;
//        default:
//            break;
//    }
}


//// The TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
//// Hence we need to merge the MPU register address with the buffer and then transmit all as one transmission
//static void merge_register_and_data(uint8_t * new_buffer, uint8_t reg, uint8_t * p_data, uint32_t length)
//{
//    new_buffer[0] = reg;
//    memcpy((new_buffer + 1), p_data, length);
//}



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
			nrf_gpio_pin_clear(18);
//			nrf_delay_us(100);
			idle_state_handle();
			nrf_gpio_pin_set(18);
		}
		while(twi_tx_done == false);
		
		// Close the I2C connection
		err_code = twi_close();
    APP_ERROR_CHECK(err_code);
	
//		while (twi_tx_done == false);
		twi_tx_done = false;
	
		return err_code;
}

///* reading byte or bytes from register before writing: special function*/        
//ret_code_t i2c_write_forread_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address)
//{   
//		ret_code_t err_code;
//		
//		m_xfer_done = false;
//	
//		err_code = nrf_drv_twi_tx(twi_handle, address, &sub_address, sizeof(sub_address), true);  
//		APP_ERROR_CHECK(err_code);
//	
//		while (m_xfer_done == false);
//	
//		return err_code;
//}

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
			nrf_gpio_pin_clear(18);
//			nrf_delay_us(100);
			idle_state_handle();
			nrf_gpio_pin_set(18);
		}
		while(twi_rx_done == false);
			
		// Close the I2C connection
		err_code = twi_close();
    APP_ERROR_CHECK(err_code);

//		while(twi_rx_done == false);
    twi_rx_done = false;

	
//		// Write address to read from first
//		err_code = i2c_write_forread_byte(twi_handle, address, sub_address);
//		
//	
//		if (NRF_SUCCESS == err_code)
//		{			
//			m_xfer_done = false;
//		
//			// Now we can actually read the data
//			err_code = nrf_drv_twi_rx(twi_handle, address, dest, dest_count);
//			APP_ERROR_CHECK(err_code);
//	
//			while (m_xfer_done == false);
//		}
	
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


///**
// * @brief Initialize the TWI Master module with PPI triggered
// * R/W operations started by a counter module
// */
//static void twi_with_easy_dma_setup()
//{
//    // Disable the TWIM module while we reconfigure it
//    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
//    NRF_TWIM0->SHORTS = 0;
//    NVIC_DisableIRQ(SPI0_TWI0_IRQn);
//    NVIC_ClearPendingIRQ(SPI0_TWI0_IRQn);
//	
//	
//    // Also already defined in usr_gpio
//    // Configure a gpiote channel to generate an event on a polarity change from 
//    // low to high generated the MPU interrupt pin.
////    uint8_t gpiote_ch_mpu_int_event = 0;
////    NRF_GPIOTE->CONFIG[gpiote_ch_mpu_int_event] = ( (GPIOTE_CONFIG_MODE_Event   << GPIOTE_CONFIG_MODE_Pos) | 
////                                                    (MPU_INT_PIN                << GPIOTE_CONFIG_PSEL_Pos) | 
////                                                    (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos));
//    // Already defined by nrf_drv_twi
////    NRF_TWIM0->PSEL.SCL = MPU_TWI_SCL_PIN;
////    NRF_TWIM0->PSEL.SDA = MPU_TWI_SDA_PIN;
////    NRF_TWIM0->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400;
//    
//    // Load TWI TX buffer into TWI module. Set number of bytes to write pr transfer, max count, to one. 
//    // Disable the EasyDMA list functionality for TWI TX.
//    NRF_TWIM0->TXD.PTR = (uint32_t)&p_tx_buffer;
//    NRF_TWIM0->TXD.MAXCNT = 1;
//    NRF_TWIM0->TXD.LIST = TWIM_TXD_LIST_LIST_Disabled << TWIM_TXD_LIST_LIST_Pos;
//    
//    // Point to TWI RX buffer. Set number of bytes to read pr transfer, max count, to TWIM_RX_BUF_WIDTH. 
//    // Disable the EasyDMA list functionality for TWI TX
//    NRF_TWIM0->RXD.PTR = (uint32_t)&p_rx_buffer;
//    NRF_TWIM0->RXD.MAXCNT = TWIM_RX_BUF_WIDTH;
//    NRF_TWIM0->RXD.LIST = TWIM_RXD_LIST_LIST_ArrayList << TWIM_RXD_LIST_LIST_Pos;
//    
//    // Make sure that MPU address is set
//    NRF_TWIM0->ADDRESS = MPU_ADDRESS;
//    // Enable shortcuts that starts a read right after a write and sends a stop condition after last TWI read
//    NRF_TWIM0->SHORTS = (TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos) | 
//                        (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos);
//    
//    // Configure PPI channel
//    // Use MPU interrupt pin as event
//    // Start timer 0 on event to count number of transfers
//    // Also start TWI transfers on event
//    // Enable PPI channel
//    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[gpiote_ch_mpu_int_event];
//    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TIMER0->TASKS_COUNT;
//    NRF_PPI->FORK[0].TEP = (uint32_t)&NRF_TWIM0->TASKS_STARTTX;
//    
//    // Enable the TWIM module
//    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
//}

///**
// * @brief Start the transfers
// */
//void start_transfers(void)
//{
//    // Enable timer interrupt
//    NVIC_EnableIRQ(TIMER0_IRQn);
//    // Start counter
//    NRF_TIMER0->TASKS_START = 1;
//    // Enable the PPI channel tying MPU interrupt pin to TWIM module
//    NRF_PPI->CHEN = PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos;
//}