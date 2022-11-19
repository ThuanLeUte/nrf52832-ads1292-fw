/**
* @file       one_button.h
* @copyright  Copyright (C) 2021 Hydratech. All rights reserved.
* @license    This project is released under the Hydratech License.
* @version    01.00.00
* @date       2020-08-13
* @author     Thuan Le
* @brief      One button library to handle button event
* @note       None
* @example    None
*/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __ONE_BUTTON_H
#define __ONE_BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

/* Public enumerate/structure ----------------------------------------------- */
/**
 * @brief One button event enum
 */
typedef enum
{
  OB_EVT_CLICK             = 0x01,
  OB_EVT_DOUBLE_CLICK      = 0x02,
  OB_EVT_PRESS_START       = 0x04,
  OB_EVT_LONG_PRESS_START  = 0x08,
  OB_EVT_DURING_LONG_PRESS = 0x10,
  OB_EVT_LONG_PRESS_STOP   = 0x20
}
one_button_event_t;

typedef void (*one_button_callback_t)(int pin, one_button_event_t evt);

/**
 * @brief One button config structure
 */
typedef struct
{
  int _pin;                         // Hardware pin number.
  int _evt_mask;                    // See @one_button_event_t
  one_button_callback_t _callback;  // Callback

  bool     _activeLow;
  uint32_t _debounceTicks;  // Number of ticks for debounce times.
  uint32_t _clickTicks;     // Number of ticks that have to pass by
                            // before a click is detected.
  uint32_t _pressTicks;     // Number of ticks that have to pass by
                            // before a long button press is detected

  int      (*fp_read_pin)    (int pin);
  uint32_t (*fp_get_tick_ms) (void);
}
one_button_cfg_t;

/**
 * @brief One button struct
 */
typedef struct
{
  one_button_cfg_t cfg;

  int  _buttonPressed;
  bool _isLongPressed;

  // These variables that hold information across the upcoming tick calls.
  // They are initialized once on program start and are updated every time the
  // tick function is called.
  int _state;
  uint32_t _startTime; // will be set in state 1
  uint32_t _stopTime;  // will be set in state 2
}
one_button_t;

/* Public Constants --------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Public macros ------------------------------------------------------------ */
/* Public APIs -------------------------------------------------------------- */
/**
 * @brief         One button init
 *
 * @param[in]     obj       Pointer to one button object
 * @param[in]     init_cfg  Pointer to init config
 *
 * @return        None
 */
void one_button_init(one_button_t *obj, one_button_cfg_t *init_cfg);

/**
 * @brief         Call this function every time the input level has changed.
 *                Using this function no digital input pin is checked
 *                because the current level is given by the parameter.
 *
 * @param[in]     obj       Pointer to one button object
 *
 * @return        None
 */
void one_button_tick(one_button_t *obj);

/**
 * @brief         Get the current number of ticks that the button has been held down for.
 *
 * @param[in]     obj       Pointer to one button object
 *
 * @return        None
 */
int one_button_get_pressed_ticks(one_button_t *obj);

/**
 * @brief         Reset the button state machine.
 *
 * @param[in]     obj       Pointer to one button object
 *
 * @return        None
 */
void one_button_reset(one_button_t *obj);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C" {
#endif

#endif // __ONE_BUTTON_H

/* End of file -------------------------------------------------------------- */
