/**
* @file       sys_button.c
* @copyright  Copyright (C) 2020 ITR VN. All rights reserved.
* @license    This project is released under the ITR License.
* @version    01.00.00
* @date       2020-06-11
* @author     Thuan Le
* @brief      System module to handle button events
* @note       None
* @example    None
*/
/* Includes ----------------------------------------------------------------- */
#include "sys_button.h"
#include "nrfx_systick.h"
#include "damos_ram.h"
#include "sys_logger_flash.h"

/* Private enumerate/structure ---------------------------------------------- */
/* Private Constants -------------------------------------------------------- */
#define BUTTON_NUM            (1)
#define BTN_DEBOUNCE_TIME     (50)       // Unit: ms
#define BTN_CLICK_TIME        (1000)     // Unit: ms
#define BTN_TASK_INTERVAL     APP_TIMER_TICKS(50)       // Unit: ms
#define SYSTICK_TASK_INTERVAL APP_TIMER_TICKS(1)        // Unit: ms

/* Private variables -------------------------------------------------------- */
static one_button_t ob[BUTTON_NUM];
APP_TIMER_DEF(m_button_timer_id);
APP_TIMER_DEF(m_sys_tick_timer_id);

/* Private prototypes ------------------------------------------------------- */
static void m_sys_button_task(void *param);
static void m_sys_systick_task(void *param);

int m_read_pin(int pin)
{
  return nrf_gpio_pin_read(pin);
}

uint32_t m_nrf_systick_get(void)
{
  return g_device.sys_tick;
}
 
/* Private macros ----------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
void sys_button_init(void)
{
  uint32_t    pin_arr[]        = { IO_SW1};
  uint32_t    press_time_arr[] = { 2000 };

  one_button_cfg_t ob_cfg;
  ob_cfg._evt_mask      = (OB_EVT_CLICK | OB_EVT_LONG_PRESS_START);
  ob_cfg._activeLow     = false;
  ob_cfg._callback      = sys_button_callback;
  ob_cfg._debounceTicks = BTN_DEBOUNCE_TIME;
  ob_cfg._clickTicks    = BTN_CLICK_TIME;
  ob_cfg.fp_read_pin    = m_read_pin;
  ob_cfg.fp_get_tick_ms = m_nrf_systick_get;

  for (int i = 0; i < BUTTON_NUM; i++)
  {
    ob_cfg._pin        = pin_arr[i];
    ob_cfg._pressTicks = press_time_arr[i];
    one_button_init(&ob[i], &ob_cfg);
    one_button_reset(&ob[i]);
  }

  app_timer_create(&m_button_timer_id,
                   APP_TIMER_MODE_REPEATED,
                   m_sys_button_task);

  app_timer_create(&m_sys_tick_timer_id,
                   APP_TIMER_MODE_REPEATED,
                   m_sys_systick_task);

  app_timer_start(m_button_timer_id, BTN_TASK_INTERVAL, NULL);
  app_timer_start(m_sys_tick_timer_id, SYSTICK_TASK_INTERVAL, NULL);
}

void sys_button_callback(int pin, one_button_event_t evt)
{
  if (OB_EVT_CLICK == evt)
  {
    static bool record_start = false;

    NRF_LOG_RAW_INFO("Button event click \n");
    if (record_start)
    {
      sys_logger_flash_stop_record();
      record_start = false;
    }
    else
    {
      sys_logger_flash_start_writing_record();
      record_start = true;
    }
  }
  else if (OB_EVT_LONG_PRESS_START == evt)
  {
    NRF_LOG_RAW_INFO("Button long press start \n");
    if (g_device.is_device_on)
    {
      // Power off device
      bsp_power_on_device(false);
      g_device.is_device_on = false;
    }
    else
    {
      // Power on device
      bsp_power_on_device(true);
      g_device.is_device_on = true;
    }
  }
}

/* Private functions -------------------------------------------------------- */
/**
 * @brief         Button task
 *
 * @param[in]     param   Pointer to param
 *
 * @return        None
 */
static void m_sys_button_task(void *param)
{
  for (int i = 0; i < BUTTON_NUM; i++)
  {
    one_button_tick(&ob[i]);
  }
}

/**
 * @brief         Button task
 *
 * @param[in]     param   Pointer to param
 *
 * @return        None
 */
static void m_sys_systick_task(void *param)
{
  g_device.sys_tick++;
}

/* Private implementations -------------------------------------------------- */
/* End of file -------------------------------------------------------------- */
