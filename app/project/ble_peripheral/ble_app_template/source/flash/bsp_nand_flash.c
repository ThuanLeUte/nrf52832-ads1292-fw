/**
 * @file       bsp_nand_flash.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-09-05
 * @author     Thuan Le
 * @brief      Board Support Package for Nand Flash
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_nand_flash.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static w25n01_t m_w25n01;

/* Private function prototypes ---------------------------------------- */
#if (_CONFIG_ENABLE_TEST_FLASH)
void bsp_nand_flash_test(void);
#endif // _CONFIG_ENABLE_TEST_FLASH

/* Function definitions ----------------------------------------------- */
void bsp_nand_flash_init(void)
{
  m_w25n01.spi_transfer = bsp_spi_2_transmit_receive;
  m_w25n01.gpio_write   = bsp_gpio_write;

  w25n01_init(&m_w25n01);

#if (_CONFIG_ENABLE_TEST_FLASH)
  bsp_nand_flash_test();
#endif // _CONFIG_ENABLE_TEST_FLASH
}

void bsp_nand_flash_block_erase(uint32_t page_num)
{
  w25n01_block_erase(&m_w25n01, page_num);
  nrf_delay_ms(1);
}

void bsp_nand_flash_write(uint32_t page_num, uint8_t *buf, uint16_t len)
{
  w25n01_load_program_data(&m_w25n01, page_num, buf, len);
  nrf_delay_ms(1); // Delay at least 100us

  w25n01_program_execute(&m_w25n01, page_num);
  nrf_delay_ms(1);
}

void bsp_nand_flash_read(uint32_t page_num, uint8_t *buf, uint16_t len)
{
  w25n01_page_data_read(&m_w25n01, page_num);
  nrf_delay_ms(1); // Delay at least 100us

  w25n01_read_data(&m_w25n01, page_num, buf, len);
  nrf_delay_ms(1);
}

/* Private function definitions ---------------------------------------- */
#if (_CONFIG_ENABLE_TEST_FLASH)
void bsp_nand_flash_test(void)
{
#define FLASH_PAGE_SIZE_SUPPORT (FLASH_PAGE_SIZE - 1)

  uint32_t block_addr = 0;
  uint8_t w_buf[FLASH_PAGE_SIZE_SUPPORT];
  uint8_t r_buf[FLASH_PAGE_SIZE_SUPPORT];
  uint8_t w_value = 0;

  // Prepare the buffer
  for(uint16_t i = 0; i < FLASH_PAGE_SIZE_SUPPORT; i++)
  {
    if ((i % 100) == 0)
    {
      w_value++;
    }
    w_buf[i] = w_value;
    r_buf[i] = 0;
  }

  // Must erase the block first

  for (uint8_t i = 0; i < 1000; i++)
  {
    bsp_nand_flash_block_erase(block_addr);

    bsp_nand_flash_write(block_addr , w_buf, FLASH_PAGE_SIZE_SUPPORT);

    bsp_nand_flash_read(block_addr , r_buf, FLASH_PAGE_SIZE_SUPPORT);

    if (memcmp(r_buf, w_buf, sizeof(w_buf)) == 0)
    {
      NRF_LOG_INFO("Flash read and write success");
    }
    else
    {
      NRF_LOG_ERROR("Flash read and write error");
    }
    NRF_LOG_PROCESS();
  }

#undef FLASH_PAGE_SIZE_SUPPORT
}
#endif // _CONFIG_ENABLE_TEST_FLASH

/* End of file -------------------------------------------------------- */
