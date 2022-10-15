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

void bsp_nand_flash_block_erase(uint32_t page_addr)
{
  w25n01_block_erase(&m_w25n01, page_addr);
  bsp_delay_ms(1000);  // Delay at least 1ms
}

void bsp_nand_flash_write(uint32_t page_addr, uint8_t *buf, uint16_t len)
{
  w25n01_load_program_data(&m_w25n01, page_addr, buf, len);
  bsp_delay_ms(100); // Delay at least 100us

  w25n01_program_execute(&m_w25n01, page_addr);
  bsp_delay_ms(100);   // Delay at least 1ms
}

void bsp_nand_flash_read(uint32_t page_addr, uint8_t *buf, uint16_t len)
{
  w25n01_page_data_read(&m_w25n01, page_addr);
  bsp_delay_ms(100); // Delay at least 100us

  w25n01_read_data(&m_w25n01, page_addr, buf, len);
  bsp_delay_ms(100);   // Delay at least 1ms
}

/* Private function definitions ---------------------------------------- */
#if (_CONFIG_ENABLE_TEST_FLASH)
void bsp_nand_flash_test(void)
{
#define FLASH_PAGE_SIZE_SUPPORT (FLASH_PAGE_SIZE - 1)

  uint32_t block_addr = FLASH_BLOCK64_SIZE * 10;
  uint8_t w_buf[FLASH_PAGE_SIZE_SUPPORT];
  uint8_t r_buf[FLASH_PAGE_SIZE_SUPPORT];

  // Prepare the buffer
  for(uint16_t i = 0; i < FLASH_PAGE_SIZE_SUPPORT; i++)
  {
    w_buf[i] = i;
    r_buf[i] = 0;
  }

  // Must erase the block first
  bsp_nand_flash_block_erase(block_addr);

  bsp_nand_flash_read(block_addr, r_buf, FLASH_PAGE_SIZE_SUPPORT);

  bsp_nand_flash_write(block_addr, w_buf, FLASH_PAGE_SIZE_SUPPORT);

  bsp_nand_flash_read(block_addr, r_buf, FLASH_PAGE_SIZE_SUPPORT);

#undef FLASH_PAGE_SIZE_SUPPORT
}
#endif // _CONFIG_ENABLE_TEST_FLASH

/* End of file -------------------------------------------------------- */
