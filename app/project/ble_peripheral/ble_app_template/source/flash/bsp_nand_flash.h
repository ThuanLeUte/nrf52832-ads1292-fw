/**
 * @file       bsp_nand_flash.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-09-05
 * @author     Thuan Le
 * @brief      Board Support Package for Nand Flash
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BSP_NAND_FLASH_H
#define __BSP_NAND_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "w25n01.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void bsp_nand_flash_init(void);
void bsp_nand_flash_block_erase(uint32_t page_num);
void bsp_nand_flash_write(uint32_t page_num, uint8_t *buf, uint16_t len);
void bsp_nand_flash_read(uint32_t page_num, uint8_t *buf, uint16_t len);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __BSP_NAND_FLASH_H

/* End of file -------------------------------------------------------- */
