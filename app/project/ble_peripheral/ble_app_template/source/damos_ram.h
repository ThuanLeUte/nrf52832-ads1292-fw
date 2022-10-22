/**
 * @file       damos_ram.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-24
 * @author     Thuan Le
 * @brief      Damos RAM
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef _DAMOS_RAM_H
#define _DAMOS_RAM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
   SYS_MODE_STREAM_DATA
  ,SYS_MODE_RECORD_DATA
}
system_mode_t;

typedef struct
{
  system_mode_t mode;
  struct
  {
    uint8_t id_read;
    bool start_write;
    bool start_read;
  }
  record;

  bool led_blink_enable;
  uint64_t sys_tick;
  bool is_device_on;
}
system_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
extern system_t g_device;

/* Public function prototypes ----------------------------------------- */

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // _DAMOS_RAM_H

/* End of file -------------------------------------------------------- */
