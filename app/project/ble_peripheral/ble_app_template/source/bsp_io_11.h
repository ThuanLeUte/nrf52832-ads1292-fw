/**
 * @file       bsp_io_11.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-23
 * @author     Thuan Le
 * @brief      Pin description for hardware version 1.1
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BSP_IO_11_H
#define __BSP_IO_11_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "device_config.h"

/* Public defines ----------------------------------------------------- */
                                    // Schematic
// PIN NAME PORT 0 ``````````````````````````````
#define IO_I2C1_SCL             (27)    // SCL signal pin
#define IO_I2C1_SDA             (26)    // SDA signal pin

#define IO_AFE_START            (30)    // AFE START
#define IO_AFE_RST              (19)    // AFE RST
#define IO_AFE_DRDY             (20)    // AFE DRDY
#define IO_AFE_MISO             (28)    // AFE MISO
#define IO_AFE_MOSI             (25)    // AFE MOSI
#define IO_AFE_SCLK             (29)    // AFE SCLK
#define IO_AFE_CS               (31)    // AFE CS

#if (_CONFIG_DEVICE_DEVKIT)
#define IO_FLASH_MISO           (12)
#define IO_FLASH_MOSI           (5)
#define IO_FLASH_SCLK           (2)
#define IO_FLASH_CS             (4)
#else
#define IO_FLASH_MISO           (11)
#define IO_FLASH_MOSI           (12)
#define IO_FLASH_SCLK           (10)
#define IO_FLASH_CS             (9)
#endif // _CONFIG_DEVICE_DEVKIT

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/* -------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __BSP_IO_11_H

/* End of file -------------------------------------------------------- */
