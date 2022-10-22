/**
* @file       one_button.c
* @copyright  Copyright (C) 2021 Hydratech. All rights reserved.
* @license    This project is released under the Hydratech License.
* @version    01.00.00
* @date       2020-06-11
* @author     Khang Vo
* @brief      One button library to handle button event
* @note       None
* @example    None
*/
/* Includes ----------------------------------------------------------------- */
#include "one_button.h"

/* Private enumerate/structure ---------------------------------------------- */
/* Private Constants -------------------------------------------------------- */
#define LOW   (0)
#define HIGH  (1)

/* Private variables -------------------------------------------------------- */
/* Private macros ----------------------------------------------------------- */
#define JUMP_TO_CALLBACK(ob_obj, evt)               \
  do {                                              \
    if (ob_obj->cfg._evt_mask & evt)                \
    {                                               \
      ob_obj->cfg._callback(ob_obj->cfg._pin, evt); \
    }                                               \
  } while (0)

/* Private prototypes ------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
void one_button_init(one_button_t *obj, one_button_cfg_t *init_cfg)
{
  obj->cfg = *init_cfg;

  if (obj->cfg._activeLow)
  {
    // The button connects the input pin to GND when pressed.
    obj->_buttonPressed = LOW;
  }
  else
  {
    // The button connects the input pin to VCC when pressed.
    obj->_buttonPressed = HIGH;
  }
}

int one_button_get_pressed_ticks(one_button_t *obj)
{
  return obj->_stopTime - obj->_startTime;
}

void one_button_reset(one_button_t *obj)
{
  obj->_state         = 0;
  obj->_startTime     = 0;
  obj->_stopTime      = 0;
  obj->_isLongPressed = false;
}

void one_button_tick(one_button_t *obj)
{
  uint32_t now = obj->cfg.fp_get_tick_ms(); // Current (relative) time in msecs.
  bool active_lvl = (obj->cfg.fp_read_pin(obj->cfg._pin) == obj->_buttonPressed);

  // OneButton state machine
  if (obj->_state == 0)
  { 
    // Waiting for menu pin being pressed.
    if (active_lvl)
    {
      obj->_state     = 1;    // Step to state 1
      obj->_startTime = now;  // Remember starting time
    }
  }
  else if (obj->_state == 1)
  { 
    // Waiting for menu pin being released.
    if ((!active_lvl) && 
        ((uint32_t)(now - obj->_startTime) < obj->cfg._debounceTicks))
    {
      // Button was released to quickly so I assume some debouncing.
      // Go back to state 0 without calling a function.
      obj->_state = 0;
    }
    else if (!active_lvl)
    {
      obj->_state    = 2;    // Step to state 2
      obj->_stopTime = now;  // Remember stopping time
    }
    else if ((active_lvl) && 
             ((uint32_t)(now - obj->_startTime) > obj->cfg._pressTicks))
    {
      obj->_isLongPressed = true; // Keep track of long press state

      JUMP_TO_CALLBACK(obj, OB_EVT_LONG_PRESS_START);
      JUMP_TO_CALLBACK(obj, OB_EVT_DURING_LONG_PRESS);

      obj->_state    = 6;    // Step to state 6
      obj->_stopTime = now;  // Remember stopping time
    }
    else
    {
      // Button was pressed down. wait. Stay in this state.
      // If a pressStart event is registered, call it:
      JUMP_TO_CALLBACK(obj, OB_EVT_PRESS_START);
    }
  }
  else if (obj->_state == 2)
  {
    // Waiting for menu pin being pressed the second time or timeout.
    if (((obj->cfg._evt_mask & OB_EVT_DOUBLE_CLICK) == 0) &&
        (uint32_t)(now - obj->_startTime) < obj->cfg._clickTicks)
    {
      // This was only a single short click
      JUMP_TO_CALLBACK(obj, OB_EVT_CLICK);
      obj->_state = 0;          // Restart.
    }
    else if (((obj->cfg._evt_mask & OB_EVT_DOUBLE_CLICK) == 0) &&
        (uint32_t)(now - obj->_startTime) > obj->cfg._clickTicks)
    {
      // This was only a single short click
      obj->_state = 0;          // Restart.
    }
    else if ((active_lvl) &&
             ((uint32_t)(now - obj->_stopTime) > obj->cfg._debounceTicks))
    {
      obj->_state     = 3;    // Step to state 3
      obj->_startTime = now;  // Remember starting time
    }
  }
  else if (obj->_state == 3)
  {
    // Waiting for menu pin being released finally.
    // Stay here for at least obj->_debounceTicks because else we might end up in
    // State 1 if the button bounces for too long.
    if ((!active_lvl) &&
        ((uint32_t)(now - obj->_startTime) > obj->cfg._debounceTicks))
    {
      // This was a 2 click sequence.
      JUMP_TO_CALLBACK(obj, OB_EVT_DOUBLE_CLICK);
      obj->_state    = 0;    // Restart.
      obj->_stopTime = now;  // Remember stopping time
    }
  }
  else if (obj->_state == 6)
  {
    // Waiting for menu pin being release after long press.
    if (!active_lvl)
    {
      obj->_isLongPressed = false;  // Keep track of long press state
      JUMP_TO_CALLBACK(obj, OB_EVT_LONG_PRESS_STOP);
      obj->_state    = 0;    // Restart.
      obj->_stopTime = now;  // Remember stopping time
    }
    else
    {
      // Button is being long pressed
      obj->_isLongPressed = true;   // Keep track of long press state
      JUMP_TO_CALLBACK(obj, OB_EVT_DURING_LONG_PRESS);
    }
  }
}

/* Private implementations -------------------------------------------------- */
/* End of file -------------------------------------------------------------- */
