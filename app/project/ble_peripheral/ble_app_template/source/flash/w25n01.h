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

// /* Public defines ----------------------------------------------------- */
// The W25N01GV 1G-bit memory array is organized into 65,536 programmable pages of 2,048-bytes each.
// The entire page can be programmed at one time using the data from the 2,048-Byte internal buffer. Pages
// can be erased in groups of 64 (128KB block erase). The W25N01GV has 1,024 erasable blocks.

#define FLASH_SIZE (FLASH_PAGE_COUNT * FLASH_PAGE_SIZE) // Total device size in Bytes
#define FLASH_PAGE_COUNT    (0x10000) // Total device size in Pages (65536 Pages)
#define FLASH_PAGE_SIZE     (0x800)   // Total size of Pages (2048 Bytes)
#define FLASH_BLOCK64_COUNT (0x400)   // Total device size in Block64k (1024 Blocks)
#define FLASH_BLOCK64_SIZE  (0x20000) // Total size of Block64k (128KB = 64 pages)

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
 * @brief W25N01 init
 */
base_status_t w25n01_init(w25n01_t *me);

/**
 * @brief W25N01 erase block (64 pages = 128KB)
 * The 128KB Block Erase instruction sets all memory within a specified block (64-Pages, 128K-Bytes) to the
 * erased state of all 1s (FFh).
 */
base_status_t w25n01_block_erase(w25n01_t *me, uint32_t page_addr);

/**
 * @brief W25N01 load program data
 * The Program operation allows from one byte to 2,112 bytes (a page) of data to be programmed at previously
 * erased (FFh) memory locations. A Program operation involves two steps:
 * 1. Load the program data into the Data Buffer. 
 * 2. Issue “Program Execute” command to transfer the data from Data Buffer to the specified memory page.
 */
base_status_t w25n01_load_program_data(w25n01_t *me, uint16_t column_addr, uint8_t *p_data, uint32_t len);

/**
 * @brief W25N01 program execute
 * The Program Execute instruction is the second step of the Program operation. After the program data are
 * loaded into the 2,112-Byte Data Buffer (or 2,048 bytes when ECC is enabled), the Program Execute
 * instruction will program the Data Buffer content into the physical memory page that is specified in the
 * instruction
 */
base_status_t w25n01_program_execute(w25n01_t *me, uint32_t page_addr);

/**
 * @brief W25N01 page data read
 * The Page Data Read instruction will transfer the data of the specified memory page into the 2,112-Byte Data Buffer
 */
base_status_t w25n01_page_data_read(w25n01_t *me, uint32_t page_addr);

/**
 * @brief W25N01 read data
 * The Read Data instruction allows one or more data bytes to be sequentially read from the Data Buffer after
 * executing the Read Page Data instruction
 */
base_status_t w25n01_read_data(w25n01_t *me, uint16_t column_addr, uint8_t *p_data, uint32_t len);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __W25N01_H

/* End of file -------------------------------------------------------- */
