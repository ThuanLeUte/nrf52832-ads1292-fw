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

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  uint16_t writer; // (0 -> 1024) One block
  uint8_t page_index[FLASH_BLOCK64_COUNT]; // (0 -> 64)
}
logger_meta_data_t;

typedef struct 
{
  int16_t ecg_value;
  mpu9250_scaled_data_t acc_data;
}
logger_data_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void sys_logger_flash_init(void);
void sys_logger_flash_write(logger_t *logger);
void sys_logger_flash_read(logger_t *logger);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // _SYS_LOGGER_FLASH_H

/* End of file -------------------------------------------------------- */
