/**
 * @file       ble_acs.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      ACS (BLE Accelerometer Service)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "sdk_common.h"
#include "ble.h"
#include "ble_acs.h"
#include "ble_srv_common.h"
#include "nrf_log.h"

/* Private defines ---------------------------------------------------- */
#define BLE_UUID_ACS_AXIS_X_CHARACTERISTIC  0x1235
#define BLE_UUID_ACS_AXIS_Y_CHARACTERISTIC  0x1236
#define BLE_UUID_ACS_AXIS_Z_CHARACTERISTIC  0x1237

#define ACS_BASE_UUID                                                                                \
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
  BLE_UUID_ACS_AXIS_X_CHARACTERISTIC,
  BLE_UUID_ACS_AXIS_Y_CHARACTERISTIC,
  BLE_UUID_ACS_AXIS_Z_CHARACTERISTIC
};

/* Private function prototypes ---------------------------------------- */
static void m_ble_acs_on_connect(ble_acs_t *p_acs, ble_evt_t const *p_ble_evt);
static void m_ble_acs_on_write(ble_acs_t *p_acs, ble_evt_t const *p_ble_evt);

static ret_code_t m_ble_acs_add_char(ble_acs_t *p_acs, const ble_acs_init_t *p_acs_init, ble_acs_charaterictic_t charac);
static ret_code_t m_ble_acs_send_notification(ble_gatts_hvx_params_t *const p_hvx_params, uint16_t conn_handle);

/* Function definitions ----------------------------------------------- */
uint32_t ble_acs_init(ble_acs_t *p_acs, ble_acs_init_t const *p_acs_init)
{
  ret_code_t err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t acs_base_uuid = ACS_BASE_UUID;

  VERIFY_PARAM_NOT_NULL(p_acs);
  VERIFY_PARAM_NOT_NULL(p_acs_init);

  // Initialize the service structure.
  p_acs->evt_handler               = p_acs_init->evt_handler;
  p_acs->is_notification_supported = p_acs_init->support_notification;

  // Add a custom base UUID.
  err_code = sd_ble_uuid_vs_add(&acs_base_uuid, &p_acs->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_acs->uuid_type;
  ble_uuid.uuid = BLE_UUID_ACS_SERVICE;

  // Add the service.
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_acs->service_handle);
  VERIFY_SUCCESS(err_code);

  // Add the Accelerometer Characteristics.
  err_code = m_ble_acs_add_char(p_acs, p_acs_init, BLE_ACS_AXIS_X_CHAR);
  VERIFY_SUCCESS(err_code);

  err_code = m_ble_acs_add_char(p_acs, p_acs_init, BLE_ACS_AXIS_Y_CHAR);
  VERIFY_SUCCESS(err_code);

  return m_ble_acs_add_char(p_acs, p_acs_init, BLE_ACS_AXIS_Z_CHAR);
}

ret_code_t ble_acs_acc_update(ble_acs_t *p_acs, uint16_t acc, uint16_t conn_handle, ble_acs_charaterictic_t charac)
{
  ret_code_t err_code;

  // Send value if connected and notifying.
  if (conn_handle != BLE_CONN_HANDLE_INVALID)
  {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t               len;

    len = sizeof(acc);

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_acs->acc_char_handles[charac].value_handle;
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
          err_code = m_ble_acs_send_notification(&hvx_params, conn_handles.conn_handles[i]);
      }
    }
    else
    {
      err_code = m_ble_acs_send_notification(&hvx_params, conn_handle);
    }
  }
  else
  {
    err_code = NRF_ERROR_INVALID_STATE;
  }

  return err_code;
}

void ble_acs_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
  if ((p_context == NULL) || (p_ble_evt == NULL))
    return;

  ble_acs_t *p_acs = (ble_acs_t *)p_context;

  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GAP_EVT_CONNECTED:
    m_ble_acs_on_connect(p_acs, p_ble_evt);
    break;

  case BLE_GATTS_EVT_WRITE:
    m_ble_acs_on_write(p_acs, p_ble_evt);
    break;

  default:
    break;
  }
}

/* Private function definitions --------------------------------------- */
/**
 * @brief         Function for adding the Accelerometer characteristic.
 *
 * @param[in]     p_acs         Accelerometer Service structure.
 * @param[in]     p_acs_init    Information needed to initialize the service.
 * @param[in]     char_uuid     Charaterictic UUID
 *
 * @attention     None
 *
 * @return        None
 */
static ret_code_t m_ble_acs_add_char(ble_acs_t *p_acs, const ble_acs_init_t *p_acs_init, ble_acs_charaterictic_t charac)
{
  ble_add_char_params_t   add_char_params;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid              = BLE_UUID_CHAR[charac];
  add_char_params.max_len           = sizeof(uint16_t);
  add_char_params.init_len          = sizeof(uint16_t);
  add_char_params.char_props.notify = p_acs->is_notification_supported;
  add_char_params.char_props.read   = 1;
  add_char_params.cccd_write_access = p_acs_init->bl_cccd_wr_sec;
  add_char_params.read_access       = p_acs_init->bl_rd_sec;

  return characteristic_add(p_acs->service_handle, &add_char_params, &(p_acs->acc_char_handles[charac]));
}

/**
 * @brief         Function for sending notifications with the Accelerometer characteristic.
 *
 * @param[in]     p_hvx_params Pointer to structure with notification data.
 * @param[in]     conn_handle  Connection handle.
 *
 * @attention     None
 *
 * @return        NRF_SUCCESS on success, otherwise an error code.
 * 
 */
static ret_code_t m_ble_acs_send_notification(ble_gatts_hvx_params_t *const p_hvx_params, uint16_t conn_handle)
{
  ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, p_hvx_params);

  if (err_code == NRF_SUCCESS)
  {
    NRF_LOG_INFO("Accelerometer notification has been sent using conn_handle: 0x%04X", conn_handle);
  }
  else
  {
    NRF_LOG_DEBUG("Error: 0x%08X while sending notification with conn_handle: 0x%04X", err_code, conn_handle);
  }

  return err_code;
}

/**
 * @brief         Function for handling the Connect event.
 *
 * @param[in]     p_acs       Accelerometer Service structure.
 * @param[in]     p_ble_evt   Pointer to the event received from BLE stack.
 *
 * @attention     None
 *
 * @return        None
 */
static void m_ble_acs_on_connect(ble_acs_t *p_acs, ble_evt_t const *p_ble_evt)
{

}

/**
 * @brief         Function for handling the Write event.
 *
 * @param[in]     p_acs       Accelerometer Service structure.
 * @param[in]     p_ble_evt   Pointer to the event received from BLE stack.
 *
 * @attention     None
 *
 * @return        None
 */
static void m_ble_acs_on_write(ble_acs_t *p_acs, ble_evt_t const *p_ble_evt)
{

}

/* End of file -------------------------------------------------------- */
