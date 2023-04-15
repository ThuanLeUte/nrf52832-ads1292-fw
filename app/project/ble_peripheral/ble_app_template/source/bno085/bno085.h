/**
 * @file       bno085.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-03-24
 * @author     Thuan Le
 * @brief      BNO085 Driver
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BNO085_H
#define __BNO085_H

#ifdef __cplusplus
extern "C" {
#endif

#define _CONFIG_ARDUINO_PLATFORM  (0)

/* Includes ----------------------------------------------------------- */
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#if (_CONFIG_ARDUINO_PLATFORM)
#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#else

#endif // _CONFIG_ARDUINO_PLATFORM

/* Public defines ----------------------------------------------------- */
#define BNO08x_I2C_ADDR_DEFAULT (0x4A) // The default I2C address

/* Additional Activities not listed in SH-2 lib */
#define PAC_ON_STAIRS           (8)   // Activity code for being on stairs
#define PAC_OPTION_COUNT        (9)   // The number of current options for the activity classifier

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
bool bno085_init(void);
bool bno085_was_reset(void);
bool bno085_enable_report(sh2_SensorId_t sensor, uint32_t interval_us);
bool bno085_get_sensor_event(sh2_SensorValue_t *value);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __BNO085_H

/* End of file -------------------------------------------------------- */
