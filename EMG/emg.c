// Include header
#include "emg.h"

// SPI driver
#include "nrf_drv_spi.h"

#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"

// Error handling
#include "app_error.h"

#include <string.h>

//nRF LOG
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Include ADC Driver
#include "AD7768-1.h"



#define EMG_GND         4
#define EMG_VDD         3


#define SPI_SS_PIN      31
#define SPI_MISO_PIN    30
#define SPI_MOSI_PIN    29
#define SPI_SCK_PIN     28

#define SDI_IDLE_HIGH           0xFF       // Idle high during reading 
#define ACTIVE_LOW_FRAME        0 << 7
#define READ                    1 << 6
#define WRITE                   0 << 6

#define SPI_INSTANCE  1 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[2];           /**< TX buffer. */
static uint8_t       m_rx_buf[2];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */


/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
    NRF_LOG_INFO("SPI read data: %d %d", m_rx_buf[0], m_rx_buf[1]);
    NRF_LOG_FLUSH();
}

uint32_t spi_init()
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    spi_config.mode = NRF_DRV_SPI_MODE_3;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    NRF_LOG_INFO("SPI Initialized.");
}


void spi_transaction()
{
    // Reset rx buffer and transfer done flag
    memset(m_rx_buf, 0, m_length);
    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

}

void spi_write(uint8_t address, uint8_t value)
{
    m_tx_buf[0] = ACTIVE_LOW_FRAME | WRITE | address;
    m_tx_buf[1] = value;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 0));
}


#define AD7768_RD_FLAG_MSK(x)		(BIT(6) | ((x) & 0x3F))
#define AD7768_WR_FLAG_MSK(x)		((x) & 0x3F)

void spi_read(uint8_t address)
{
    m_tx_buf[0] = ACTIVE_LOW_FRAME | READ | address;
    m_tx_buf[1] = SDI_IDLE_HIGH;

    spi_xfer_done = false;
    while(spi_xfer_done)
    {
        __WFE();
    }

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 2, m_rx_buf, 2));
}

