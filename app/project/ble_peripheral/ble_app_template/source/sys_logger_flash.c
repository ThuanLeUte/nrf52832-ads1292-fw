/**
 * @file       sys_logger_flash.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-24
 * @author     Thuan Le
 * @brief      System module to handle log data to flash
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "sys_logger_flash.h"

/* Private defines ---------------------------------------------------- */
#define LOGGER_META_DATA_START_ADDR   (0)
#define LOGGER_DATA_START_ADDR        (FLASH_BLOCK64_SIZE)  // First block is for saving meta data
#define NUMBER_OF_PAGE_EACH_BLOCK     (64)                  // 64 pages per block

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  uint16_t writer;
  uint16_t reader;
  uint8_t buf[FLASH_PAGE_SIZE];
}
logger_ram_t;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
logger_meta_data_t g_logger_meta_data;

/* Private variables -------------------------------------------------- */
static logger_ram_t logger_ram;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void sys_logger_flash_init(void);
{
  // Reset ram logger
  memset(&logger_ram, 0, sizeof(logger_ram));
 
  // Read the meta data
  bsp_nand_flash_read(LOGGER_META_DATA_START_ADDR, &g_logger_meta_data, sizeof(g_logger_meta_data));
}

void sys_logger_flash_write(logger_data_t *logger_data, uint16_t len);
{
#define BLOCK g_logger_meta_data.writer

  uint32_t block_addr;
  uint32_t page_addr;

  // Save data to the RAM buffer (2048 Bytes - 1 page)
  memcpy(&logger_ram.buf[logger_ram.writer], logger_data, len);
  logger_ram.writer += len;

  // Enough 1 page to write to flash
  if (logger_ram.writer >= FLASH_PAGE_SIZE)
  {
    // Write data to flash
    block_addr = LOGGER_DATA_START_ADDR + (BLOCK * FLASH_BLOCK64_SIZE);
    page_addr  = block_addr +  g_logger_meta_data.page_index[BLOCK] * FLASH_BLOCK64_SIZE;
    bsp_nand_flash_block_erase(block_addr);
    bsp_nand_flash_write(page_addr, logger_ram.buf, FLASH_PAGE_SIZE);

    // Save meta data to flash ---- {
    g_logger_meta_data.page_index[BLOCK]++;
    if (g_logger_meta_data.page_index[BLOCK] > NUMBER_OF_PAGE_EACH_BLOCK)
    {
      BLOCK++;
      g_logger_meta_data.page_index[BLOCK] = 0;

      if (BLOCK > FLASH_BLOCK64_COUNT)
      {
        BLOCK = 0;
        g_logger_meta_data.page_index[BLOCK] = 0;
      }
    }
    bsp_nand_flash_block_erase(LOGGER_META_DATA_START_ADDR);
    bsp_nand_flash_write(LOGGER_META_DATA_START_ADDR, &g_logger_meta_data, sizeof(g_logger_meta_data));
    //-------------------------------}

    // Reset ram logger
    memset(&logger_ram, 0, sizeof(logger_ram));
  }

#undef BLOCK
}

void sys_logger_flash_read(uint16_t block_id);
{
  uint32_t block_addr;
  uint16_t page_addr;
  uint8_t page_index = 0;
  uint8_t data_len = sizeof(logger_data_t);
  logger_data_t temp_data;

  block_addr = LOGGER_DATA_START_ADDR + (block_id * FLASH_BLOCK64_SIZE);

  while (1)
  {
    // Read the data of the page
    page_addr = block_addr + page_index * FLASH_BLOCK64_SIZE;
    bsp_nand_flash_read(page_addr, logger_ram.buf, FLASH_PAGE_SIZE);

    while (1)
    {
      // Print out the data of the page
      memcpy(&temp_data, &logger_ram.buf[logger_ram.reader], data_len);

      // Send the data via UART or BLE
      
      // Check reader index in page
      logger_ram.reader += data_len;
      if (logger_ram.reader > logger_ram.write)
      {
        // Finish read data of the page
        break;
      }
    }

    // Check page index in block
    page_index++;
    if (page_index > g_logger_meta_data.page_index[block_id])
    {
      // Finish read the data of the block
      break;
    }
  }
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
