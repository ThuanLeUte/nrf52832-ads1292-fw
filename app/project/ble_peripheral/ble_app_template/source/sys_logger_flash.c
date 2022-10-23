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
#include "damos_ram.h"

/* Private defines ---------------------------------------------------- */
#define LOGGER_META_DATA_START_BLOCK   (FLASH_BLOCK64_COUNT - 1) // Block at the end of the flash
#define LOGGER_DATA_START_ADDR         (0)
#define NUMBER_OF_PAGE_EACH_BLOCK      (64)                      // 64 pages per block
#define MAX_FLASH_PAGE_SIZE_SUPPORT    (FLASH_PAGE_SIZE - 1)

/* Private enumerate/structure ---------------------------------------- */
static struct
{
  uint16_t writer;
  uint16_t reader;
  uint8_t buf[MAX_FLASH_PAGE_SIZE_SUPPORT];
  uint8_t buf_read[MAX_FLASH_PAGE_SIZE_SUPPORT];
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

  sys_logger_flash_erase_block(LOGGER_META_DATA_START_BLOCK);

  // Read the meta data
  bsp_nand_flash_read(LOGGER_META_DATA_START_BLOCK, (uint8_t *)&g_logger_meta_data, sizeof(g_logger_meta_data));

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
#define PATIENT g_logger_meta_data.patient_info[g_logger_meta_data.logger_id]

  logger_status_t logger_status;
  logger_data_t logger_data;

  // Erase block before write data
  sys_logger_flash_erase_block(g_logger_meta_data.block_writer);

  // Save meta data
  PATIENT.block_start = g_logger_meta_data.block_writer;
  PATIENT.block_stop  = g_logger_meta_data.block_writer;
  PATIENT.page_writer = PATIENT.block_start * NUMBER_OF_PAGE_EACH_BLOCK;
  PATIENT.is_logged   = 1;

  NRF_LOG_INFO("Flash write start at logger ID: %d", g_logger_meta_data.logger_id);
  NRF_LOG_INFO("Block start : %d", PATIENT.block_start);
  NRF_LOG_INFO("Block stop  : %d", PATIENT.block_stop);
  NRF_LOG_INFO("Pager writer: %d", PATIENT.page_writer);
  NRF_LOG_INFO("Pager reader: %d", PATIENT.page_reader);
  NRF_LOG_INFO("Is logged   : %d", PATIENT.is_logged);

  logger_data.ecg_data.heart_rate = 0;

  // Reset ram logger
  memset(&logger_ram, 0, sizeof(logger_ram));

  while (1)
  {
    NRF_LOG_PROCESS();

#if (_CONFIG_ENABLE_SIMULATE_ECG_DATA)
    // Simulate logger data
    if (logger_data.ecg_data.heart_rate++ >= 250)
      logger_data.ecg_data.heart_rate = 0;
#else
    bsp_afe_get_ecg(&logger_data.ecg_data);
    // bsp_gyro_accel_get(&logger_data.acc_data, NULL);
#endif // _CONFIG_ENABLE_SIMULATE_ECG_DATA

    // Write data to block
    logger_status = sys_logger_flash_write_block(g_logger_meta_data.block_writer, PATIENT.page_writer, (uint8_t *)&logger_data, sizeof(logger_data));

    // Force to stop the record
    if (g_device.record.start_write == false)
    {
      // Move to the next block
      g_logger_meta_data.block_writer++;

      goto _LBL_END_;
    }

    // Check write logger status
    switch (logger_status)
    {
    case LOGGER_WRITE_BLOCK_ON_PROCESS:
      break;

    case LOGGER_WRITE_PAGE_FINISHED:
    {
      NRF_LOG_INFO("LOGGER_WRITE_PAGE_FINISHED: %d", PATIENT.page_writer);

      // Move on to the next page
      PATIENT.page_writer++;

      logger_flash_save_meta_data();
    }
    break;

    case LOGGER_WRITE_BLOCK_FINISHED:
    {
      NRF_LOG_INFO("LOGGER_WRITE_PAGE_FINISHED: %d", PATIENT.page_writer);
      NRF_LOG_WARNING("LOGGER_WRITE_BLOCK_FINISHED: %d", g_logger_meta_data.block_writer);

      // Move to the next block
      g_logger_meta_data.block_writer++;
      PATIENT.page_writer++;

      // Reset page writer
      PATIENT.block_stop  = g_logger_meta_data.block_writer;

      // Erase block before write data
      sys_logger_flash_erase_block(g_logger_meta_data.block_writer);

      logger_flash_save_meta_data();
    }
    break;

    case LOGGER_WRITE_BLOCK_ERROR:
      NRF_LOG_INFO("LOGGER_WRITE_BLOCK_ERROR");
      return;
      break;

    default:
      break;
    }
  }

_LBL_END_:
  NRF_LOG_INFO("_LBL_END_");

  NRF_LOG_INFO("Flash write stop at logger ID: %d", g_logger_meta_data.logger_id);
  NRF_LOG_INFO("Block start : %d", PATIENT.block_start);
  NRF_LOG_INFO("Block stop  : %d", PATIENT.block_stop);
  NRF_LOG_INFO("Page  writer: %d", PATIENT.page_writer);
  NRF_LOG_INFO("Page  reader: %d", PATIENT.page_reader);
  NRF_LOG_INFO("Is logged   : %d", PATIENT.is_logged);

  // Check logger id
  if (g_logger_meta_data.logger_id++ > MAX_RECORD_SUPPORTED)
    g_logger_meta_data.logger_id = 0;

  logger_flash_save_meta_data();

  g_device.record.start_write = false;
  g_device.led_blink_enable   = false;

#undef PATIENT
}

void sys_logger_flash_read(uint8_t logger_id)
{
#define PATIENT g_logger_meta_data.patient_info[logger_id]

  logger_status_t logger_status;
  logger_data_t logger_data;
  uint16_t block_reader = PATIENT.block_start;

  NRF_LOG_INFO("Current logger ID: %d", g_logger_meta_data.logger_id);
  NRF_LOG_INFO("Flash read start at logger ID: %d", logger_id);
  NRF_LOG_INFO("Block start : %d", PATIENT.block_start);
  NRF_LOG_INFO("Block stop  : %d", PATIENT.block_stop);
  NRF_LOG_INFO("Pager writer: %d", PATIENT.page_writer);
  NRF_LOG_INFO("Pager reader: %d", PATIENT.page_reader);
  NRF_LOG_INFO("Is logged   : %d", PATIENT.is_logged);

  if (PATIENT.is_logged == 0)
  {
    NRF_LOG_WARNING("Patient with ID: %d has no data", logger_id);
    return;
  }

  // Save meta data
  PATIENT.page_reader = block_reader * NUMBER_OF_PAGE_EACH_BLOCK;

  while (1)
  {
    NRF_LOG_PROCESS();

    // Read data at block_reader
    logger_status = sys_logger_flash_read_block(block_reader, PATIENT.page_reader, (uint8_t *)&logger_data, sizeof(logger_data));

#if (_CONFIG_ENABLE_DETAIL_LOG)
    // Send the data via UART or BLE
    NRF_LOG_INFO("ECG heart rate  : %d", logger_data.ecg_data.heart_rate);
    nrf_delay_ms(10);
#endif

    // Force to stop read the record
    if (g_device.record.start_read == false)
    {
      goto _LBL_END_;
    }

    // Check read logger status
    switch (logger_status)
    {
    case LOGGER_READ_BLOCK_ON_PROCESS:
      break;

    case LOGGER_READ_PAGE_FINISHED:
    {
      NRF_LOG_INFO("LOGGER_READ_PAGE_FINISHED: %d", PATIENT.page_reader);

      // Move on to the next page
      PATIENT.page_reader++;

      // Check block reader and page reader
      if ((block_reader == PATIENT.block_stop) && (PATIENT.page_reader == PATIENT.page_writer))
      {
        NRF_LOG_INFO("Finish read all data in record: %d", logger_id);
        goto _LBL_END_;
      }
    }
    break;

    case LOGGER_READ_BLOCK_FINISHED:
    {
      NRF_LOG_INFO("LOGGER_READ_PAGE_FINISHED: %d", PATIENT.page_reader);
      NRF_LOG_WARNING("LOGGER_READ_BLOCK_FINISHED: %d", block_reader);

      // Move to the next block
      block_reader++;
      PATIENT.page_reader++;

      // Check block reader
      if (block_reader > PATIENT.block_stop)
      {
        NRF_LOG_INFO("Finish read all data in record: %d", logger_id);
        goto _LBL_END_;
      }
    }
    break;

    case LOGGER_READ_BLOCK_ERROR:
      NRF_LOG_INFO("LOGGER_READ_BLOCK_ERROR");
      goto _LBL_END_;
      break;

    default:
      break;
    }
  }

_LBL_END_:
  NRF_LOG_INFO("_LBL_END_");
  g_device.record.start_read = false;
  g_device.led_blink_enable  = false;

#undef PATIENT
}

void sys_logger_flash_erase_all_record(void)
{
  NRF_LOG_WARNING("Erase all records");

  if (g_device.record.start_read || g_device.record.start_write)
  {
    NRF_LOG_WARNING("Device is on the writing or reading process, can not erase all records");
    return;
  }

  // Erase meta data
  memset(&g_logger_meta_data, 0, sizeof(g_logger_meta_data));
  logger_flash_save_meta_data();
}

void sys_logger_flash_start_writing_record(void)
{
  g_device.record.start_write = true;
  g_device.record.start_read  = false;
  g_device.led_blink_enable   = true;
}

void sys_logger_flash_start_reading_record(uint8_t record_id)
{
  g_device.record.id_read     = record_id;
  g_device.record.start_read  = true;
  g_device.record.start_write = false;
  g_device.led_blink_enable   = true;
}

void sys_logger_flash_stop_record(void)
{
  g_device.record.start_read  = false;
  g_device.record.start_write = false;
  g_device.led_blink_enable   = false;
}

bool sys_logger_flash_is_reading_record(void)
{
  if ((g_device.record.start_read == true) && (g_device.record.start_write == false))
    return true;

  return false;
}

bool sys_logger_flash_is_writing_record(void)
{
  if ((g_device.record.start_read == false) && (g_device.record.start_write == true))
    return true;

  return false;
}

logger_status_t sys_logger_flash_write_block(uint16_t block_id, uint32_t page_id, uint8_t *data, uint16_t len)
{
  if (page_id >= FLASH_PAGE_COUNT)
    return LOGGER_WRITE_BLOCK_ERROR;

  // Save data to the RAM buffer (2048 Bytes - 1 page)
  memcpy(&logger_ram.buf[logger_ram.writer], data, len);
  logger_ram.writer += len;

  // Enough 1 page to write to flash
  if (logger_ram.writer >= MAX_FLASH_PAGE_SIZE_SUPPORT)
  {
    // Write data to flash
    bsp_nand_flash_write(page_id, logger_ram.buf, MAX_FLASH_PAGE_SIZE_SUPPORT);
    memset(logger_ram.buf, 0, MAX_FLASH_PAGE_SIZE_SUPPORT);

    // Reset ram logger
    memset(&logger_ram, 0, sizeof(logger_ram));

    // Write block finished
    if ((page_id % (NUMBER_OF_PAGE_EACH_BLOCK - 1) == 0) && (page_id != 0))
      return LOGGER_WRITE_BLOCK_FINISHED;

    return LOGGER_WRITE_PAGE_FINISHED;
  }

  return LOGGER_WRITE_BLOCK_ON_PROCESS;
}

logger_status_t sys_logger_flash_read_block(uint16_t block_id, uint32_t page_id, uint8_t *data, uint16_t len)
{
  if (page_id >= FLASH_PAGE_COUNT)
    return LOGGER_READ_BLOCK_ERROR;

  // Read the data of the page
  if (logger_ram.reader == 0) // Read page data at the first time
  {
    bsp_nand_flash_read(page_id, logger_ram.buf, MAX_FLASH_PAGE_SIZE_SUPPORT);
  }

  // Copy the data of the page
  memcpy(data, &logger_ram.buf[logger_ram.reader], len);

  // Increase reader
  logger_ram.reader += len;

  // Read page finished
  if (logger_ram.reader > MAX_FLASH_PAGE_SIZE_SUPPORT)
  {
    // Reset ram logger
    memset(&logger_ram, 0, sizeof(logger_ram));

    // Read block finished
    if ((page_id % (NUMBER_OF_PAGE_EACH_BLOCK - 1) == 0) && (page_id != 0))
      return LOGGER_READ_BLOCK_FINISHED;

    return LOGGER_READ_PAGE_FINISHED;
  }

  return LOGGER_READ_BLOCK_ON_PROCESS;
}

void sys_logger_flash_erase_block(uint16_t block_id)
{
  uint32_t page_num = block_id * NUMBER_OF_PAGE_EACH_BLOCK;

  // Erase block
  bsp_nand_flash_block_erase(page_num);
}

/* Private function definitions ---------------------------------------- */
static void logger_flash_save_meta_data(void)
{
  bsp_nand_flash_block_erase(LOGGER_META_DATA_START_BLOCK);
  bsp_nand_flash_write(LOGGER_META_DATA_START_BLOCK, (uint8_t *)&g_logger_meta_data, sizeof(g_logger_meta_data));
}

/* End of file -------------------------------------------------------- */
