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
#include "nrf_log.h"

/* Private defines ---------------------------------------------------- */
#define LOGGER_META_DATA_START_ADDR   (0)
#define LOGGER_DATA_START_ADDR        (FLASH_BLOCK64_SIZE)  // First block is for saving meta data
#define NUMBER_OF_PAGE_EACH_BLOCK     (64)                  // 64 pages per block

/* Private enumerate/structure ---------------------------------------- */
static struct
{
  uint16_t writer;
  uint16_t reader;
  uint8_t buf[FLASH_PAGE_SIZE];
}logger_ram;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
logger_meta_data_t g_logger_meta_data;

/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void logger_flash_save_meta_data(void);

/* Function definitions ----------------------------------------------- */
void sys_logger_flash_init(void)
{
  // Reset ram logger
  memset(&logger_ram, 0, sizeof(logger_ram));
 
  // Read the meta data
  bsp_nand_flash_read(LOGGER_META_DATA_START_ADDR, (uint8_t *)&g_logger_meta_data, sizeof(g_logger_meta_data));

  // Emty data --> Erase all
  if (g_logger_meta_data.block_writer == 0xFFFF)
  {
    NRF_LOG_INFO("Logger empty, delete meta data");

    memset(&g_logger_meta_data, 0, sizeof(g_logger_meta_data));
    logger_flash_save_meta_data();
  }
  else
  {

    // Reset ram logger
    memset(&logger_ram, 0, sizeof(logger_ram));

    // Print info of the block
    NRF_LOG_INFO("Logger meta data:");
    NRF_LOG_INFO("Block writer: %d", g_logger_meta_data.block_writer);
  }
}

void sys_logger_flash_write(void)
{
  logger_status_t logger_status;
  logger_data_t logger_data;
  uint16_t block_writer = 0;

  NRF_LOG_INFO("sys_logger_flash_write");

  while (1)
  {
    NRF_LOG_PROCESS();

    // Simulate logger data
    logger_data.ecg_value++;

    // Write data to block
    logger_status = sys_logger_flash_write_block(block_writer, (uint8_t *)&logger_data, sizeof(logger_data));

    // Check write logger status
    switch (logger_status)
    {
    case LOGGER_WRITE_BLOCK_ON_PROCESS:
      break;

    case LOGGER_WRITE_PAGE_FINISHED:
      NRF_LOG_INFO("LOGGER_WRITE_PAGE_FINISHED");
      logger_flash_save_meta_data();

      break;
      
    case LOGGER_WRITE_BLOCK_FINISHED:
      NRF_LOG_INFO("LOGGER_WRITE_BLOCK_FINISHED");
      logger_flash_save_meta_data();
      goto _LBL_END_;
      break;

    case LOGGER_WRITE_BLOCK_ERROR:
      goto _LBL_END_;

      break;
    default:
      break;
    }
  }

_LBL_END_:
  NRF_LOG_INFO("_LBL_END_");
}

void sys_logger_flash_read(void)
{
  logger_status_t logger_status;
  logger_data_t logger_data;
  uint16_t block_reader = 0;

  NRF_LOG_INFO("sys_logger_flash_read");

  while (1)
  {
    NRF_LOG_PROCESS();

    // Read data at block_reader
    logger_status = sys_logger_flash_read_block(block_reader, (uint8_t *)&logger_data, sizeof(logger_data));

#if (_CONFIG_ENABLE_DETAIL_LOG)
    // Send the data via UART or BLE
    NRF_LOG_INFO("++++++++++++++++++++++++++++++++++++");
    NRF_LOG_INFO("ECG data  : %d", logger_data.ecg_value);
    NRF_LOG_INFO("Acc X Axis: %d", logger_data.acc_data.x);
    NRF_LOG_INFO("Acc Y Axis: %d", logger_data.acc_data.y);
    NRF_LOG_INFO("Acc Z Axis: %d", logger_data.acc_data.z);
    NRF_LOG_INFO("-----------------------------------");
#endif

    // Check read logger status
    switch (logger_status)
    {
    case LOGGER_READ_BLOCK_ON_PROCESS:
      break;

    case LOGGER_READ_PAGE_FINISHED:
      NRF_LOG_INFO("LOGGER_READ_PAGE_FINISHED");
      break;

    case LOGGER_READ_BLOCK_FINISHED:
      NRF_LOG_INFO("LOGGER_READ_BLOCK_FINISHED");
      sys_logger_flash_erase_block(block_reader);
      goto _LBL_END_;
      break;

    case LOGGER_READ_BLOCK_ERROR:
      goto _LBL_END_;

      break;
    default:
      break;
    }
  }

_LBL_END_:
  NRF_LOG_INFO("_LBL_END_");
}

logger_status_t sys_logger_flash_write_block(uint16_t block_id, uint8_t *data, uint16_t len)
{
  uint32_t block_addr;
  uint32_t page_addr;

  // Save data to the RAM buffer (2048 Bytes - 1 page)
  memcpy(&logger_ram.buf[logger_ram.writer], data, len);
  logger_ram.writer += len;

  // Enough 1 page to write to flash
  if (logger_ram.writer >= FLASH_PAGE_SIZE)
  {
    // Prepare address
    block_addr = LOGGER_DATA_START_ADDR + (block_id * FLASH_BLOCK64_SIZE);
    page_addr  = block_addr +  g_logger_meta_data.page_writer[block_id] * FLASH_BLOCK64_SIZE;

    // Write data to flash
    bsp_nand_flash_block_erase(block_addr);
    bsp_nand_flash_write(page_addr, logger_ram.buf, FLASH_PAGE_SIZE);

    // Reset ram logger
    memset(&logger_ram, 0, sizeof(logger_ram));

    // Move on to the next page
    g_logger_meta_data.page_writer[block_id]++;

    // Write block finished
    if (g_logger_meta_data.page_writer[block_id] > NUMBER_OF_PAGE_EACH_BLOCK)
    {
      g_logger_meta_data.page_writer[block_id] = NUMBER_OF_PAGE_EACH_BLOCK;
      return LOGGER_WRITE_BLOCK_FINISHED;
    }

    return LOGGER_WRITE_PAGE_FINISHED;
  }

  return LOGGER_WRITE_BLOCK_ON_PROCESS;
}

logger_status_t sys_logger_flash_read_block(uint16_t block_id, uint8_t *data, uint16_t len)
{
  uint32_t block_addr;
  uint32_t page_addr;
  static uint8_t page_reader = 0;

  // Read the data of the page
  if (logger_ram.reader == 0) // Read page data at the first time
  {
    block_addr = LOGGER_DATA_START_ADDR + (block_id * FLASH_BLOCK64_SIZE);
    page_addr  = block_addr + page_reader * FLASH_BLOCK64_SIZE;
    bsp_nand_flash_read(page_addr, logger_ram.buf, FLASH_PAGE_SIZE);
  }

  // Copy the data of the page
  memcpy(data, &logger_ram.buf[logger_ram.reader], len);

  // Increase reader
  logger_ram.reader += len;

  // Read page finished
  if (logger_ram.reader > FLASH_PAGE_SIZE)
  {
    // Reset ram logger
    memset(&logger_ram, 0, sizeof(logger_ram));

    // Move on to the next page
    page_reader++;

    // Read block finished
    if (page_reader > g_logger_meta_data.page_writer[block_id])
    {
      page_reader = 0;
      return LOGGER_READ_BLOCK_FINISHED;
    }

    return LOGGER_READ_PAGE_FINISHED;
  }

  return LOGGER_READ_BLOCK_ON_PROCESS;
}

void sys_logger_flash_erase_block(uint16_t block_id)
{
  uint32_t block_addr = LOGGER_DATA_START_ADDR + (block_id * FLASH_BLOCK64_SIZE);;

  // Erase block
  bsp_nand_flash_block_erase(block_addr);

  // Save meta data to flash
  g_logger_meta_data.page_writer[block_id] = 0;
  logger_flash_save_meta_data();
}

/* Private function definitions ---------------------------------------- */
static void logger_flash_save_meta_data(void)
{
  bsp_nand_flash_block_erase(LOGGER_META_DATA_START_ADDR);
  bsp_nand_flash_write(LOGGER_META_DATA_START_ADDR, (uint8_t *)&g_logger_meta_data, sizeof(g_logger_meta_data));
}

/* End of file -------------------------------------------------------- */
