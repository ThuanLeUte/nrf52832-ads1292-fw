/**
 * @file       sys_logger_flash.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-24
 * @author     Thuan Le
 * @brief      System module to handle log data to flash
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _SYS_LOGGER_FLASH_H
#define _SYS_LOGGER_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "device_config.h"
#include "bsp_nand_flash.h"
#include "bsp_imu.h"
#include "bsp_afe.h"

/* Public defines ----------------------------------------------------- */
#define MAX_RECORD_SUPPORTED  (200)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
   LOGGER_WRITE_BLOCK_FINISHED
  ,LOGGER_WRITE_BLOCK_ON_PROCESS
  ,LOGGER_WRITE_PAGE_FINISHED
  ,LOGGER_WRITE_BLOCK_ERROR

  ,LOGGER_READ_BLOCK_FINISHED
  ,LOGGER_READ_BLOCK_ON_PROCESS
  ,LOGGER_READ_PAGE_FINISHED
  ,LOGGER_READ_BLOCK_ERROR
}
logger_status_t;

typedef struct
{
  uint16_t block_start;                   // (0 -> 1024) Blocks
  uint16_t block_stop;                    // (0 -> 1024) Blocks
  uint16_t page_writer;                    // (0 -> 64) Pages
  uint16_t page_reader;                    // (0 -> 64) Pages
  uint8_t is_logged;                      // (0 -> 64) Pages
}
logger_patient_info_t;

typedef struct
{
  uint16_t block_writer;                    // (0 -> 1024) One block
  uint8_t logger_id;
  logger_patient_info_t patient_info[MAX_RECORD_SUPPORTED];
}
logger_meta_data_t;

typedef struct 
{
  ecg_data_t ecg_data;
  mpu9250_scaled_data_t acc_data;
}
logger_data_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern logger_meta_data_t g_logger_meta_data;

/* Public function prototypes ----------------------------------------- */
void sys_logger_flash_init(void);
void sys_logger_flash_write(void);
void sys_logger_flash_read(uint8_t logger_id);
void sys_logger_flash_erase_all_record(void);
void sys_logger_flash_start_writing_record(void);
void sys_logger_flash_start_reading_record(uint8_t record_id);
void sys_logger_flash_stop_record(void);
bool sys_logger_flash_is_reading_record(void);
bool sys_logger_flash_is_writing_record(void);

logger_status_t sys_logger_flash_write_block(uint16_t block_id, uint32_t page_id, uint8_t *data, uint16_t len);
logger_status_t sys_logger_flash_read_block(uint16_t block_id, uint32_t page_id, uint8_t *data, uint16_t len);
void sys_logger_flash_erase_block(uint16_t block_id);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // _SYS_LOGGER_FLASH_H

/* End of file -------------------------------------------------------- */
