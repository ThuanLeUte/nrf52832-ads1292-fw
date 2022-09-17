/**
 * @file       w25n01.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-04-09
 * @author     Thuan Le
 * @brief      SERIAL SLC NAND FLASH MEMORY
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __W25N01_H
#define __W25N01_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "bsp_hw.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief DS2728 sensor struct
 */
typedef struct 
{
  void (*gpio_write)(uint8_t pin , uint8_t state);
  base_status_t  (*spi_transfer)(uint8_t *tx_data, uint8_t *rx_data, uint16_t len);
}
w25n01_t;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief         W25N01 init
 *
 * @param[in]     me    Pointer to handle of W25N01 module
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t w25n01_init(w25n01_t *me);
base_status_t w25n01_block_erase(w25n01_t *me, uint32_t page_addr);
base_status_t w25n01_load_program_data(w25n01_t *me, uint16_t column_addr, uint8_t *p_data, uint32_t len);
base_status_t w25n01_program_execute(w25n01_t *me, uint32_t page_addr);
base_status_t w25n01_page_data_read(w25n01_t *me, uint32_t page_addr);
base_status_t w25n01_read_data(w25n01_t *me, uint16_t column_addr, uint8_t *p_data, uint32_t len);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __W25N01_H

/* End of file -------------------------------------------------------- */
