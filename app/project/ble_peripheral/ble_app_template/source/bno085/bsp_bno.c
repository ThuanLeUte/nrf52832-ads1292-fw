/**
 * @file       bsp_bno.C
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-03-24
 * @author     Thuan Le
 * @brief      Board support package for BNO
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_bno.h"
#include"bsp_hw.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static sh2_Hal_t m_bno085; ///< The struct representing the SH2 Hardware Abstraction Layer
static sh2_SensorValue_t *_sensor_value = NULL;

/* Private function prototypes ---------------------------------------- */
static int bsp_bno_hal_i2c_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us);
static int bsp_bno_hal_i2c_write(sh2_Hal_t *self, uint8_t *buf, unsigned len);
static uint32_t bsp_bno_hal_get_time_us(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);

/* Function definitions ----------------------------------------------- */
base_status_t bsp_bno_init(void)
{
  int status;
  sh2_ProductIds_t prodIds;

  // Init
  m_bno085.open      = NULL;
  m_bno085.close     = NULL;
  m_bno085.read      = bsp_bno_hal_i2c_read;
  m_bno085.write     = bsp_bno_hal_i2c_write;
  m_bno085.getTimeUs = bsp_bno_hal_get_time_us;

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&m_bno085, hal_callback, NULL);
  if (status != SH2_OK)
  {
    return BS_ERROR;
  }

  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK)
  {
    return BS_ERROR;
  }

  // Register sensor listener
  sh2_setSensorCallback(sensorHandler, NULL);

  return BS_OK;
}

/* Private function definitions ---------------------------------------- */
static int bsp_bno_hal_i2c_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us)
{
}

static int bsp_bno_hal_i2c_write(sh2_Hal_t *self, uint8_t *buf, unsigned len)
{

}

static uint32_t bsp_bno_hal_get_time_us(sh2_Hal_t *self)
{

}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent)
{

}

static void sensorHandler(void *cookie, sh2_SensorEvent_t *event)
{
  int rc;

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK)
  {
    _sensor_value->timestamp = 0;
    return;
  }
}

/* End of file -------------------------------------------------------- */
