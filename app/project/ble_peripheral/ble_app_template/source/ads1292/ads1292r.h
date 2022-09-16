/**
 * @file       ads1292r.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-07-31
 * @author     Thuan Le
 * @brief      Driver support ADS1292R (Analog Front-End for Biopotential Measurements)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __ADS1292R_H
#define __ADS1292R_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "platform.h"

/* Public defines ----------------------------------------------------- */
// Register address
#define ADS1292_REG_ID          (0x00)
#define ADS1292_REG_CONFIG1     (0x01)
#define ADS1292_REG_CONFIG2     (0x02)
#define ADS1292_REG_LOFF        (0x03)
#define ADS1292_REG_CH1SET      (0x04)
#define ADS1292_REG_CH2SET      (0x05)
#define ADS1292_REG_RLDSENS     (0x06)
#define ADS1292_REG_LOFFSENS    (0x07)
#define ADS1292_REG_LOFFSTAT    (0x08)
#define ADS1292_REG_RESP1       (0x09)
#define ADS1292_REG_RESP2       (0x0A)

// Register commands
#define ADS1292_CMD_WAKEUP      (0x02) // Wake-up from standby mode
#define ADS1292_CMD_STANDBY     (0x04) // Enter Standby mode
#define ADS1292_CMD_RESET       (0x06) // Reset the device
#define ADS1292_CMD_START       (0x08) // Start and restart (synchronize) conversions
#define ADS1292_CMD_STOP        (0x0A) // Stop conversion
#define ADS1292_CMD_RDATAC      (0x10) // Enable Read Data Continuous mode (default mode at power-up)
#define ADS1292_CMD_SDATAC      (0x11) // Stop Read Data Continuous mode
#define ADS1292_CMD_RDATA       (0x12) // Read data by command; Supports multiple read back
#define ADS1292_CMD_RREG        (0x20) // (also = 00100000) is the first opcode that the address must be added to for ADS1292_CMD_RREG communication
#define ADS1292_CMD_WREG        (0x40) // 01000000 in binary (Datasheet, pg. 35)

#define CONFIG_SPI_MASTER_DUMMY (0xFF)

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief ADS1292 sensor data struct
 */
typedef struct
{
  volatile signed long daq_vals[8];
  bool lead_off_detected;
  signed long result_temp_resp;
}
ads1292_output_value_t;

/* Public function prototypes ----------------------------------------- */
base_status_t ads1292_get_ecg_and_respiration_sample(const int data_ready, const int chip_select, ads1292_output_value_t *data_sample);
base_status_t ads1292_init(const int chip_select, const int pwdn_pin, const int start_pin);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __ADS1292R_H

/* End of file -------------------------------------------------------- */

