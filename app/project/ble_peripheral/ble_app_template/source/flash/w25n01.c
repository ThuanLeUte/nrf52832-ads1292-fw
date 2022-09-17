/**
 * @file       w25n01.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2022-08-18
 * @author     Thuan Le
 * @brief      SERIAL SLC NAND FLASH MEMORY
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "w25n01.h"

/* Private defines ---------------------------------------------------- */
#define W25N_JEDEC_ID                 (0x9F)
#define WINBOND_MAN_ID                (0xEF)
#define W25N01GV_DEV_ID               (0xAA21)

#define W25M_DIE_SELECT               (0xC2)

#define W25N_OP_RESET                 (0xFF)
#define W25N_OP_JEDEC_ID              (0x9F)
#define W25N_OP_READ_STATUS_REG       (0x05)
#define W25N_OP_WRITE_STATUS_REG      (0x01)
#define W25N_OP_WRITE_ENABLE          (0x06)
#define W25N_OP_WRITE_DISABLE         (0x04)
#define W25N_OP_BB_MANAGE             (0xA1)
#define W25N_OP_READ_BBM              (0xA5)
#define W25N_OP_LAST_ECC_FAIL         (0xA9)
#define W25N_OP_BLOCK_ERASE           (0xD8)
#define W25N_OP_PROG_DATA_LOAD        (0x02)
#define W25N_OP_RAND_PROG_DATA_LOAD   (0x84)
#define W25N_OP_PROG_EXECUTE          (0x10)
#define W25N_OP_PAGE_DATA_READ        (0x13)
#define W25N_OP_READ                  (0x03)
#define W25N_OP_FAST_READ             (0x0B)

#define W25N_REG_PROT                 (0xA0) // Protection register
#define W25N_REG_CONFIG               (0xB0) // Configuration register
#define W25N_REG_STAT                 (0xC0) // Status register

#define W25N01GV_MAX_PAGE             (65535)
#define W25N_MAX_COLUMN               (2112)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static base_status_t m_w25n01_transfer(w25n01_t *me, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);
static base_status_t m_w25n01_set_status_register(w25n01_t *me, uint8_t reg, uint8_t value);
static base_status_t m_w25n01_get_status_register(w25n01_t *me, uint8_t reg, uint8_t *value);
static base_status_t m_w25n01_write_enable(w25n01_t *me, bool enable);

/* Function definitions ----------------------------------------------- */
base_status_t w25n01_init(w25n01_t *me)
{
  ASSERT(me != NULL);
  ASSERT(me->gpio_write != NULL);
  ASSERT(me->spi_transfer != NULL);

  uint8_t jedec[5] = { W25N_JEDEC_ID, 0x00, 0x00, 0x00, 0x00 };

  CHECK_STATUS(m_w25n01_transfer(me, jedec, jedec, sizeof(jedec)));

  if (jedec[2] == WINBOND_MAN_ID)
  {
    if ((uint16_t)(jedec[3] << 8 | jedec[4]) == W25N01GV_DEV_ID)
    {
      CHECK_STATUS(m_w25n01_set_status_register(me, W25N_REG_PROT, 0x00));
      return BS_OK;
    }
  }

  return BS_ERROR;
}

base_status_t w25n01_block_erase(w25n01_t *me, uint32_t page_addr)
{
  ASSERT(page_addr <= W25N01GV_MAX_PAGE);
  
  uint8_t page_high  = (page_addr & 0xFF00) >> 8;
  uint8_t page_low   = page_addr;
  uint8_t cmd_buf[4] = { W25N_OP_BLOCK_ERASE, 0x00, page_high, page_low };

  CHECK_STATUS(m_w25n01_write_enable(me, true));
  CHECK_STATUS(m_w25n01_transfer(me, cmd_buf, NULL, sizeof(cmd_buf)));
  
  return BS_OK;
}

base_status_t w25n01_load_program_data(w25n01_t *me, uint16_t column_addr, uint8_t *p_data, uint32_t len)
{
  ASSERT(column_addr <= W25N_MAX_COLUMN);
  ASSERT(len <= W25N_MAX_COLUMN - column_addr);

  uint8_t column_high = (column_addr & 0xFF00) >> 8;
  uint8_t column_low  = column_addr & 0xFF;
  uint8_t cmd_buf[3]  = { W25N_OP_PROG_DATA_LOAD, column_high, column_low };

  CHECK_STATUS(m_w25n01_write_enable(me, true));

  // Write data
  me->gpio_write(IO_AFE_CS, 0);
  CHECK_STATUS(me->spi_transfer(cmd_buf, NULL, sizeof(cmd_buf)));
  CHECK_STATUS(me->spi_transfer(p_data, NULL, len));
  me->gpio_write(IO_AFE_CS, 1);

  return BS_OK;
}

base_status_t w25n01_program_execute(w25n01_t *me, uint32_t page_addr)
{
  ASSERT(page_addr <= W25N01GV_MAX_PAGE);

  uint8_t page_high  = (page_addr & 0xFF00) >> 8;
  uint8_t page_low   = page_addr;
  uint8_t cmd_buf[4] = { W25N_OP_PROG_EXECUTE, 0x00, page_high, page_low };

  CHECK_STATUS(m_w25n01_write_enable(me, true));
  CHECK_STATUS(m_w25n01_transfer(me, cmd_buf, NULL, sizeof(cmd_buf)));

  return BS_OK;
}

base_status_t w25n01_page_data_read(w25n01_t *me, uint32_t page_addr)
{
  ASSERT(page_addr <= W25N01GV_MAX_PAGE);

  uint8_t page_high  = (page_addr & 0xFF00) >> 8;
  uint8_t page_low   = page_addr;
  uint8_t cmd_buf[4] = { W25N_OP_PAGE_DATA_READ, 0x00, page_high, page_low };

  CHECK_STATUS(m_w25n01_transfer(me, cmd_buf, NULL, sizeof(cmd_buf)));

  return BS_OK;
}

base_status_t w25n01_read_data(w25n01_t *me, uint16_t column_addr, uint8_t *p_data, uint32_t len)
{
  ASSERT(column_addr <= W25N_MAX_COLUMN);
  ASSERT(len <= W25N_MAX_COLUMN - column_addr);

  uint8_t column_high  = (column_addr & 0xFF00) >> 8;
  uint8_t column_low   = column_addr;
  uint8_t cmd_buf[4] = { W25N_OP_READ, column_high, column_low, 0x00 };

  // Read data
  me->gpio_write(IO_AFE_CS, 0);
  CHECK_STATUS(me->spi_transfer(cmd_buf, NULL, sizeof(cmd_buf)));
  CHECK_STATUS(me->spi_transfer(p_data, p_data, len));
  me->gpio_write(IO_AFE_CS, 1);

  return BS_OK;
}

/* Private function definitions ---------------------------------------- */
static base_status_t m_w25n01_transfer(w25n01_t *me, uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
  base_status_t ret;

  me->gpio_write(IO_AFE_CS, 0);
  ret = me->spi_transfer(tx_data, rx_data, len);
  me->gpio_write(IO_AFE_CS, 1);

  return ret;
}

static base_status_t m_w25n01_set_status_register(w25n01_t *me, uint8_t reg, uint8_t value)
{
  uint8_t cmd_buf[3] = { W25N_OP_WRITE_STATUS_REG, reg, value };

  return m_w25n01_transfer(me, cmd_buf, NULL, sizeof(cmd_buf));
}

static base_status_t m_w25n01_get_status_register(w25n01_t *me, uint8_t reg, uint8_t *value)
{
  uint8_t cmd_buf[3] = { W25N_OP_READ_STATUS_REG, reg, 0x00 };

  CHECK_STATUS(m_w25n01_transfer(me, cmd_buf, cmd_buf, sizeof(cmd_buf)));
  *value = cmd_buf[2];

  return BS_OK;
}

static base_status_t m_w25n01_write_enable(w25n01_t *me, bool enable)
{
  uint8_t value;

  if (enable)
    value = W25N_OP_WRITE_ENABLE;
  else
    value = W25N_OP_WRITE_DISABLE;

  return m_w25n01_transfer(me, &value, NULL, 1);
}

/* End of file -------------------------------------------------------- */
