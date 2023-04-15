

/**
 * @file       bno085.c
 * @copyright  Copyright (C) 2020 Peerbridge. All rights reserved.
 * @license    This project is released under the Peerbridge License.
 * @version    1.0.0
 * @date       2021-09-05
 * @author     Adrew Iosue
 * @brief      BNO085 Driver
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#define _CONFIG_ARDUINO_PLATFORM  (0)

#if (_CONFIG_ARDUINO_PLATFORM)
#include "Arduino.h"
#include <Wire.h>
#else
#include "bsp_hw.h"
#endif // _CONFIG_ARDUINO_PLATFORM

#include "bno085.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static sh2_ProductIds_t prod_id; // The product IDs returned by the sensor
static sh2_Hal_t hal;            // The struct representing the SH2 Hardware Abstraction Layer

static sh2_SensorValue_t *sensor_value = NULL;
static bool reset_occurred = false;

#if (_CONFIG_ARDUINO_PLATFORM)
static Adafruit_I2CDevice *i2c_dev = NULL; // Pointer to I2C bus interface
#endif

/* Private function prototypes ---------------------------------------- */
#if (_CONFIG_ARDUINO_PLATFORM)
static bool bno085_begin_i2c(uint8_t i2c_addr, TwoWire *wire, int32_t sensor_id);
#else
static bool bno085_begin_i2c(void);
#endif // _CONFIG_ARDUINO_PLATFORM

static int  bno085_i2c_write(sh2_Hal_t *self, uint8_t *data, unsigned len);
static int  bno085_i2c_read(sh2_Hal_t *self, uint8_t *data, unsigned len, uint32_t *t_us);
static void bno085_i2c_close(sh2_Hal_t *self);
static int  bno085_i2c_open(sh2_Hal_t *self);
static void bno085_sensor_handler(void *cookie, sh2_SensorEvent_t *event);

static uint32_t hal_get_time_us(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *event);

/* Function definitions ----------------------------------------------- */
bool bno085_init(void)
{
  int status;

#if (_CONFIG_ARDUINO_PLATFORM)
  bno085_begin_i2c(BNO08x_I2C_ADDR_DEFAULT, &Wire, 0);
#else
  bno085_begin_i2c();
#endif // _CONFIG_ARDUINO_PLATFORM

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&hal, hal_callback, NULL);
  if (status != SH2_OK)
  {
    return false;
  }

  memset(&prod_id, 0, sizeof(prod_id));

  // Check connection partially by getting the product id's
  status = sh2_getProdIds(&prod_id);
  if (status != SH2_OK)
  {
    return false;
  }

  // Register sensor listener
  sh2_setSensorCallback(bno085_sensor_handler, NULL);

  return true;
}

bool bno085_get_sensor_event(sh2_SensorValue_t *value)
{
  sensor_value = value;

  value->timestamp = 0;

  sh2_service();

  if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV)
  {
    return false;
  }

  return true;
}

bool bno085_enable_report(sh2_SensorId_t sensorId, uint32_t interval_us)
{
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK)
  {
    return false;
  }

  return true;
}

bool bno085_was_reset(void)
{
  bool x = reset_occurred;
  reset_occurred = false;

  return x;
}

/* Private function definitions --------------------------------------- */
#if (_CONFIG_ARDUINO_PLATFORM)
bool bno085_begin_i2c(uint8_t i2c_address, TwoWire *wire, int32_t sensor_id)
{
  if (i2c_dev)
  {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin())
  {
    Serial.println(F("I2C address not found"));
    return false;
  }

  hal.open = bno085_i2c_open;
  hal.close = bno085_i2c_close;
  hal.read = bno085_i2c_read;
  hal.write = bno085_i2c_write;
  hal.getTimeUs = hal_get_time_us;

  return true;
}
#else
static bool bno085_begin_i2c(void)
{
  hal.open = bno085_i2c_open;
  hal.close = bno085_i2c_close;
  hal.read = bno085_i2c_read;
  hal.write = bno085_i2c_write;
  hal.getTimeUs = hal_get_time_us;

  return true;
}
#endif // _CONFIG_ARDUINO_PLATFORM

static int bno085_i2c_open(sh2_Hal_t *self)
{
  uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  bool success = false;

  for (uint8_t attempts = 0; attempts < 5; attempts++)
  {
#if (_CONFIG_ARDUINO_PLATFORM)
    if (i2c_dev->write(softreset_pkt, 5))
#else
    if (bsp_i2c_write_bno(BNO08x_I2C_ADDR_DEFAULT, softreset_pkt, 5))
#endif // _CONFIG_ARDUINO_PLATFORM
    {
      success = true;
      break;
    }
    delay(30);
  }

  if (!success)
    return -1;
  
  delay(300);
  
  return 0;
}

static void bno085_i2c_close(sh2_Hal_t *self)
{
}

static int bno085_i2c_read(sh2_Hal_t *self, uint8_t *data, unsigned len, uint32_t *t_us)
{
  uint8_t header[4];
#if (_CONFIG_ARDUINO_PLATFORM)
  if (!i2c_dev->read(header, 4))
#else
  if (!bsp_i2c_read_bno(BNO08x_I2C_ADDR_DEFAULT, header, 4))
#endif // _CONFIG_ARDUINO_PLATFORM
  {
    return 0;
  }

  // Determine amount to read
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;

  // Unset the "continue" bit
  packet_size &= ~0x8000;

#if (_CONFIG_ARDUINO_PLATFORM)
  size_t i2c_buffer_max = i2c_dev->maxBufferSize();
#else
  size_t i2c_buffer_max = 255;
#endif // _CONFIG_ARDUINO_PLATFORM

  if (packet_size > len)
  {
    // packet wouldn't fit in our buffer
    return 0;
  }

  // the number of non-header bytes to read
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0)
  {
    if (first_read)
    {
      read_size = MIN(i2c_buffer_max, (size_t)cargo_remaining);
    }
    else
    {
      read_size = MIN(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

#if (_CONFIG_ARDUINO_PLATFORM)
    if (!i2c_dev->read(i2c_buffer, read_size))
#else
  if (!bsp_i2c_read_bno(BNO08x_I2C_ADDR_DEFAULT, i2c_buffer, read_size))
#endif // _CONFIG_ARDUINO_PLATFORM
    {
      return 0;
    }

    if (first_read)
    {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(data, i2c_buffer, cargo_read_amount);
      first_read = false;
    }
    else
    {
      // this is not the first read, so copy from 4 bytes after the beginning of
      // the i2c buffer to skip the header included with every new i2c read and
      // don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(data, i2c_buffer + 4, cargo_read_amount);
    }
    // advance our pointer by the amount of cargo read
    data += cargo_read_amount;

    // mark the cargo as received
    cargo_remaining -= cargo_read_amount;
  }

  return packet_size;
}

static int bno085_i2c_write(sh2_Hal_t *self, uint8_t *data, unsigned len)
{
#if (_CONFIG_ARDUINO_PLATFORM)
  size_t i2c_buffer_max = i2c_dev->maxBufferSize();
  uint16_t write_size = MIN(i2c_buffer_max, len);
  if (!i2c_dev->write(data, write_size))
#else
  size_t i2c_buffer_max = 255;
  uint16_t write_size = MIN(i2c_buffer_max, len);
  if (bsp_i2c_write_bno(BNO08x_I2C_ADDR_DEFAULT, data, len))
#endif // _CONFIG_ARDUINO_PLATFORM
  {
    return 0;
  }

  return write_size;
}

static uint32_t hal_get_time_us(sh2_Hal_t *self)
{
#if (_CONFIG_ARDUINO_PLATFORM)
  uint32_t t = millis() * 1000;
#else
  uint32_t t = g_systick * 1000;
#endif // _CONFIG_ARDUINO_PLATFORM
  return t;
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *event)
{
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (event->eventId == SH2_RESET)
  {
    reset_occurred = true;
  }
}

static void bno085_sensor_handler(void *cookie, sh2_SensorEvent_t *event)
{
  int rc;

  rc = sh2_decodeSensorEvent(sensor_value, event);
  if (rc != SH2_OK)
  {
    // Serial.println("BNO08x - Error decoding sensor event");
    sensor_value->timestamp = 0;
    return;
  }
}

/* End of file -------------------------------------------------------- */
