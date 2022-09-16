/**
 * @file       platform.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-09-05
 * @author     Thuan Le
 * @brief      Platform
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "platform.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
bool platform_read_pin(uint8_t pin)
{
  return nrf_gpio_pin_read(pin);
}

void platform_write_pin(uint8_t pin, bool enable)
{
  bsp_gpio_write(pin, enable);
}

void platform_delay(uint32_t ms)
{
  bsp_delay_ms(ms);
}

uint8_t platform_spi_transfer(uint8_t data)
{
  static uint8_t receive_data;

  bsp_spi_transmit_receive(&data, &receive_data, 1);

  return receive_data;
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */

