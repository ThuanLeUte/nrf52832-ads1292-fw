/**
 * @file       ble_setting.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      STS (BLE Setting Service)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "sdk_common.h"
#include "ble.h"
#include "ble_sts.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "sys_logger_flash.h"
#include "damos_ram.h"

/* Private defines ---------------------------------------------------- */
#define BLE_UUID_STS_START_STOP_RECORD_CHARACTERISTIC           (0x5235)
#define BLE_UUID_STS_GET_RECORD_CHARACTERISTIC                  (0x5236)
#define BLE_UUID_STS_CURRENT_RECORD_INDEX_CHARACTERISTIC        (0x5237)

#define STS_BASE_UUID                                                                                \
  {                                                                                                  \
    {                                                                                                \
      0x41, 0xEE, 0x68, 0x3A, 0x99, 0x0F, 0x0E, 0x72, 0x85, 0x49, 0x8D, 0xB3, 0x00, 0x00, 0x00, 0x00 \
    }                                                                                                \
  } /**< Used vendor specific UUID. */

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static const uint16_t BLE_UUID_CHAR[] = {
  BLE_UUID_STS_START_STOP_RECORD_CHARACTERISTIC,
  BLE_UUID_STS_GET_RECORD_CHARACTERISTIC,
  BLE_UUID_STS_CURRENT_RECORD_INDEX_CHARACTERISTIC
};

/* Private function prototypes ---------------------------------------- */
static void m_ble_sts_on_connect(ble_sts_t *p_sts, ble_evt_t const *p_ble_evt);
static void m_ble_sts_on_write(ble_sts_t *p_sts, ble_evt_t const *p_ble_evt);

static ret_code_t m_ble_sts_add_char(ble_sts_t *p_sts, const ble_sts_init_t *p_sts_init, ble_sts_charaterictic_t charac);
static ret_code_t m_ble_sts_send_notification(ble_gatts_hvx_params_t *const p_hvx_params, uint16_t conn_handle);

/* Function definitions ----------------------------------------------- */
uint32_t ble_sts_init(ble_sts_t *p_sts, ble_sts_init_t const *p_sts_init)
{
  ret_code_t err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t sts_base_uuid = STS_BASE_UUID;

  VERIFY_PARAM_NOT_NULL(p_sts);
  VERIFY_PARAM_NOT_NULL(p_sts_init);

  // Initialize the service structure.
  p_sts->evt_handler               = p_sts_init->evt_handler;
  p_sts->is_notification_supported = p_sts_init->support_notification;

  // Add a custom base UUID.
  err_code = sd_ble_uuid_vs_add(&sts_base_uuid, &p_sts->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_sts->uuid_type;
  ble_uuid.uuid = BLE_UUID_STS_SERVICE;

  // Add the service.
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_sts->service_handle);
  VERIFY_SUCCESS(err_code);

  // Add the STS Characteristics.
  m_ble_sts_add_char(p_sts, p_sts_init, BLE_STS_START_STOP_RECORD_CHAR);

  // Add the STS Characteristics.
  m_ble_sts_add_char(p_sts, p_sts_init, BLE_STS_GET_RECORD_CHAR);
  
  return m_ble_sts_add_char(p_sts, p_sts_init, BLE_STS_CURRENT_RECORD_INDEX_CHAR); 
}

ret_code_t ble_sts_update(ble_sts_t *p_sts, uint8_t acc, uint16_t conn_handle, ble_sts_charaterictic_t charac)
{
  ret_code_t err_code;

  // Send value if connected and notifying.
  if (conn_handle != BLE_CONN_HANDLE_INVALID)
  {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t               len;

    len = sizeof(acc);

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_sts->sts_char_handles[charac].value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &len;
    hvx_params.p_data = (uint8_t *)&acc;

    if (conn_handle == BLE_CONN_HANDLE_ALL)
    {
      ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

      // Try sending notifications to all valid connection handles.
      for (uint32_t i = 0; i < conn_handles.len; i++)
      {
        if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
          err_code = m_ble_sts_send_notification(&hvx_params, conn_handles.conn_handles[i]);
      }
    }
    else
    {
      err_code = m_ble_sts_send_notification(&hvx_params, conn_handle);
    }
  }
  else
  {
    err_code = NRF_ERROR_INVALID_STATE;
  }

  return err_code;
}

void ble_sts_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
  if ((p_context == NULL) || (p_ble_evt == NULL))
    return;

  ble_sts_t *p_sts = (ble_sts_t *)p_context;

  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GAP_EVT_CONNECTED:
    m_ble_sts_on_connect(p_sts, p_ble_evt);
    break;

  case BLE_GATTS_EVT_WRITE:
    m_ble_sts_on_write(p_sts, p_ble_evt);
    break;

  default:
    break;
  }
}

/* Private function definitions --------------------------------------- */
/**
 * @brief         Function for adding the STS characteristic.
 *
 * @param[in]     p_sts         STS Service structure.
 * @param[in]     p_sts_init    Information needed to initialize the service.
 * @param[in]     char_uuid     Charaterictic UUID
 *
 * @attention     None
 *
 * @return        None
 */
static ret_code_t m_ble_sts_add_char(ble_sts_t *p_sts, const ble_sts_init_t *p_sts_init, ble_sts_charaterictic_t charac)
{
  ble_add_char_params_t   add_char_params;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid              = BLE_UUID_CHAR[charac];
  add_char_params.max_len           = sizeof(uint8_t);
  add_char_params.init_len          = sizeof(uint8_t);
  add_char_params.cccd_write_access = SEC_OPEN;
  add_char_params.write_access      = SEC_OPEN;
  add_char_params.read_access       = SEC_OPEN;

  switch (charac)
  {
  case BLE_STS_GET_RECORD_CHAR:
    add_char_params.char_props.notify        = 0;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;
    add_char_params.char_props.read          = 0;
    break;
  
  case BLE_STS_START_STOP_RECORD_CHAR:
    add_char_params.char_props.notify        = 0;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;
    add_char_params.char_props.read          = 0;
    break;

  case BLE_STS_CURRENT_RECORD_INDEX_CHAR:
    add_char_params.char_props.notify        = p_sts->is_notification_supported;
    add_char_params.char_props.write         = 0;
    add_char_params.char_props.write_wo_resp = 0;
    add_char_params.char_props.read          = 1;
    break;
  default:
    break;
  }
  return characteristic_add(p_sts->service_handle, &add_char_params, &(p_sts->sts_char_handles[charac]));
}

/**
 * @brief         Function for sending notifications with the STS characteristic.
 *
 * @param[in]     p_hvx_params Pointer to structure with notification data.
 * @param[in]     conn_handle  Connection handle.
 *
 * @attention     None
 *
 * @return        NRF_SUCCESS on success, otherwise an error code.
 * 
 */
static ret_code_t m_ble_sts_send_notification(ble_gatts_hvx_params_t *const p_hvx_params, uint16_t conn_handle)
{
  ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, p_hvx_params);

  if (err_code == NRF_SUCCESS)
  {
    // NRF_LOG_INFO("STS notification has been sent using conn_handle: 0x%04X", conn_handle);
  }
  else
  {
    // NRF_LOG_DEBUG("Error: 0x%08X while sending notification with conn_handle: 0x%04X", err_code, conn_handle);
  }

  return err_code;
}

/**
 * @brief         Function for handling the Connect event.
 *
 * @param[in]     p_sts       STS Service structure.
 * @param[in]     p_ble_evt   Pointer to the event received from BLE stack.
 *
 * @attention     None
 *
 * @return        None
 */
static void m_ble_sts_on_connect(ble_sts_t *p_sts, ble_evt_t const *p_ble_evt)
{

}

/**
 * @brief         Function for handling the Write event.
 *
 * @param[in]     p_sts       STS Service structure.
 * @param[in]     p_ble_evt   Pointer to the event received from BLE stack.
 *
 * @attention     None
 *
 * @return        None
 */
static void m_ble_sts_on_write(ble_sts_t *p_sts, ble_evt_t const *p_ble_evt)
{
  ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
  uint8_t data_receive = p_evt_write->data[0];

  NRF_LOG_INFO("Receive data: %d, on UUID: %x", data_receive, p_evt_write->uuid.uuid);

  switch (p_evt_write->uuid.uuid)
  {
  case BLE_UUID_STS_GET_RECORD_CHARACTERISTIC:
  {
    // Start reading the record
    sys_logger_flash_start_reading_record(data_receive);
  }
  break;

  case BLE_UUID_STS_START_STOP_RECORD_CHARACTERISTIC:
  {
    switch (data_receive)
    {
    case 0: // Stop recording
      sys_logger_flash_stop_record();
      break;

    case 1: // Start recording
      sys_logger_flash_start_writing_record();
      break;

    case 3: // Erase all recordings
      sys_logger_flash_erase_all_record();
      break;

    default:
      break;
    }
  }
  break;

  default:
    break;
  }

}

/* End of file -------------------------------------------------------- */
