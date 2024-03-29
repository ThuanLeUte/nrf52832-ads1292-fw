
/**
 * @file       device_config.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-09-05
 * @author     Thuan Le
 * @brief      Device config
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __DEVICE_CONFIG_H
#define __DEVICE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

/* Public defines ----------------------------------------------------- */
#define _CONFIG_DEVICE_DEVKIT                (1)
#define _CONFIG_ENABLE_DETAIL_LOG            (1)
#define _CONFIG_ENABLE_TEST_FLASH            (0)
#define _CONFIG_ENABLE_SIMULATE_ECG_DATA     (1)

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __DEVICE_CONFIG_H

/* End of file -------------------------------------------------------- */

