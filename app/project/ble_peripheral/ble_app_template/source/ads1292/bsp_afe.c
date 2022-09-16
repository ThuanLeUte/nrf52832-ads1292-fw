/**
 * @file       bsp_afe.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-09-05
 * @author     Thuan Le
 * @brief      Board Support Package for AFE (ADS1292)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_afe.h"
#include "ecg_res_algo.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
volatile uint8_t global_heart_rate = 0;
volatile uint8_t global_respiration_rate = 0;

int16_t ecg_wave_buf, ecg_filter_out;
int16_t res_wave_buf, resp_filter_out;

/* Function definitions ----------------------------------------------- */
base_status_t bsp_afe_init(void)
{
  return ads1292_init(IO_AFE_CS, IO_AFE_RST, IO_AFE_START);
}

base_status_t bsp_afe_get_ecg(int16_t *ecg_data)
{
  ads1292_output_value_t ecg_values;

  base_status_t ret = ads1292_get_ecg_and_respiration_sample(IO_AFE_DRDY, IO_AFE_CS, &ecg_values);
  if (ret == BS_OK)
  {
    // Ignore the lower 8 bits out of 24bits
    ecg_wave_buf = (int16_t)(ecg_values.daq_vals[1] >> 8);
    res_wave_buf = (int16_t)(ecg_values.result_temp_resp >> 8);

    if (ecg_values.lead_off_detected == false)
    {
      // Filter out the line noise @40Hz cutoff 161 order
      ECG_ProcessCurrSample(&ecg_wave_buf, &ecg_filter_out);

      // Calculate
      QRS_Algorithm_Interface(ecg_filter_out, &global_heart_rate);
    }
    else
    {
      ecg_filter_out = 0;
      resp_filter_out = 0;
    }

    *ecg_data = ecg_filter_out;
  }

  return ret;
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
