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

/* Public defines ----------------------------------------------------- */
                                    // Schematic
// PIN NAME PORT 0 ``````````````````````````````
#define IO_I2C1_SCL             (27)    // SCL signal pin
#define IO_I2C1_SDA             (26)    // SDA signal pin

/* GPIO */
#define PS_HOLD						24 	/* OUTPUT */
#define LDO_AVCC_EN					23	/* OUTPUT */
#define LED1						17	/* OUTPUT */
#define LED2						18	/* OUTPUT */
#define BLE_CONNECT_PIN				LED1

/* I2C */
#define I2C1_SCL_PIN				27 	
#define I2C1_SDA_PIN				26

/* SPI */
#define SPI0_SCK_PIN                29  // SPI clock GPIO pin number.
#define SPI0_MOSI_PIN               25  // SPI Master Out Slave In GPIO pin number.
#define SPI0_MISO_PIN               28  // SPI Master In Slave Out GPIO pin number.
#define SPI0_SS_PIN                 22  // SPI Slave Select GPIO pin number.

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
