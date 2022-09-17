
/**
 * @file       bsp.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-23
 * @author     Thuan Le
 * @brief      Board Support Package (BSP)
 * 
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_hw.h"

/* Private defines ---------------------------------------------------- */
#define TWI_INSTANCE           0
#define SPI_INSTANCE_1         1
#define SPI_INSTANCE_2         2

/* Private enumerate/structure ---------------------------------------- */
static nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);
static nrf_drv_spi_t m_spi_1 = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE_1);
static nrf_drv_spi_t m_spi_2 = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE_2);

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static volatile bool_t data_ready = BS_FALSE;

/* Private function prototypes ---------------------------------------- */
static void m_bsp_i2c_init(void);
static void m_bsp_gpio_init(void);
static void m_bsp_spi_1_init(void);
static void m_bsp_spi_2_init(void);

/* Function definitions ----------------------------------------------- */
void bsp_hw_init(void)
{
  m_bsp_i2c_init();
  m_bsp_gpio_init();
  m_bsp_spi_1_init();
  m_bsp_spi_2_init();
}

int bsp_i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t *p_data, uint32_t len)
{
  uint8_t buffer[10];

  memcpy(buffer, &reg_addr, 1);
  memcpy(buffer + 1, p_data, len);

  return nrf_drv_twi_tx(&m_twi, slave_addr, buffer, len + 1, false);
}

int bsp_i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *p_data, uint32_t len)
{
  nrf_drv_twi_tx(&m_twi, slave_addr, (uint8_t *)&reg_addr, 1, true);

  return nrf_drv_twi_rx(&m_twi, slave_addr, p_data, len);
}

void bsp_delay_ms(uint32_t ms)
{
  nrf_delay_ms(ms);
}

void bsp_gpio_write(uint8_t pin , uint8_t state)
{
  nrf_gpio_pin_write(pin, state);
}

base_status_t bsp_spi_1_transmit_receive(uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
  if (nrf_drv_spi_transfer(&m_spi_1, tx_data, len, rx_data, len) != NRF_SUCCESS)
    return BS_ERROR;
  
  return BS_OK;
}

base_status_t bsp_spi_2_transmit_receive(uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
  if (nrf_drv_spi_transfer(&m_spi_2, tx_data, len, rx_data, len) != NRF_SUCCESS)
    return BS_ERROR;
  
  return BS_OK;
}

/* Private function definitions ---------------------------------------- */
/**
 * @brief         I2C init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_i2c_init(void)
{
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config =
  {
    .scl                = IO_I2C1_SCL,
    .sda                = IO_I2C1_SDA,
    .frequency          = NRF_DRV_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    .clear_bus_init     = false
  };

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
}

void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
  data_ready = BS_TRUE;
}

/**
 * @brief         SPI init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_spi_1_init(void)
{
  ret_code_t err_code;

  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

  spi_config.mosi_pin  = IO_AFE_MOSI;
  spi_config.miso_pin  = IO_AFE_MISO;
  spi_config.sck_pin   = IO_AFE_SCLK;
  spi_config.mode      = NRF_DRV_SPI_MODE_1;
  spi_config.frequency = NRF_DRV_SPI_FREQ_1M;

  err_code = nrf_drv_spi_init(&m_spi_1, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         SPI init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_spi_2_init(void)
{
  ret_code_t err_code;

  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

  spi_config.mosi_pin  = IO_FLASH_MOSI;
  spi_config.miso_pin  = IO_FLASH_MISO;
  spi_config.sck_pin   = IO_FLASH_SCLK;
  spi_config.mode      = NRF_DRV_SPI_MODE_0;
  spi_config.frequency = NRF_DRV_SPI_FREQ_1M;

  err_code = nrf_drv_spi_init(&m_spi_2, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Gpio init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_gpio_init(void)
{
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  nrf_gpio_cfg_input(IO_AFE_DRDY, NRF_GPIO_PIN_PULLUP);

  // Output in setting
  nrf_gpio_cfg_output(IO_FLASH_CS);
  nrf_gpio_cfg_output(IO_AFE_CS);
  nrf_gpio_cfg_output(IO_AFE_RST);
  nrf_gpio_cfg_output(IO_AFE_START);

  bsp_gpio_write(IO_FLASH_CS, 1);
  bsp_gpio_write(IO_AFE_CS, 1);
  bsp_gpio_write(IO_AFE_RST, 1);
  bsp_gpio_write(IO_AFE_START, 0);
}

/* End of file -------------------------------------------------------- */
