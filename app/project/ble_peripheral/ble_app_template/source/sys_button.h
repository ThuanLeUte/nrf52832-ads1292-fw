/**
 * @file       sys_button.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2022-02-15
 * @author     Thuan Le
 * @brief      Sys
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __SYS_BUTTON_H
#define __SYS_BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ----------------------------------------------------------------- */
#include "one_button.h"
#include "bsp_hw.h"

/* Public defines ----------------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------------- */
/* Public Constants --------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Public macros ------------------------------------------------------------ */
/* Public APIs -------------------------------------------------------------- */
/**
 * @brief         Button init
 *
 * @param[in]     None
 *
 * @return        None
 */
void sys_button_init(void);

/**
 * @brief         Button callback
 *
 * @param[in]     pin    Pin
 * @param[in]     evt    Event
 *
 * @return        None
 */
void sys_button_callback(int pin, one_button_event_t evt);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C" {
#endif

#endif // __SYS_BUTTON_H

/* End of file ---------------------------------------------------------------- */
