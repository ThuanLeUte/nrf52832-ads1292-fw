/**
 * @file       ble_ecg.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      ECG (BLE ECG Service)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BLE_ECG_H
#define __BLE_ECG_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

/* Public defines ----------------------------------------------------- */
#define BLE_UUID_ECG_SERVICE (0x4234) /**< The UUID of the ECG Service. */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief ECG Charaterictic
 */
typedef enum
{
  BLE_ECG_RAW_DATA_CHAR,
  BLE_ECG_HEART_RATE_CHAR,
  BLE_ECG_RESPIRATION_RATE_CHAR,
  BLE_ECG_MAX_CHAR
} 
ble_ecg_charaterictic_t;

/**
 * @brief ECG Service event type
 */
typedef enum
{
  BLE_ECG_EVT_NOTIFICATION_ENABLED, /**< ECG value notification enabled event. */
  BLE_ECG_EVT_NOTIFICATION_DISABLED /**< ECG value notification disabled event. */
} 
ble_ecg_evt_type_t;

/**
 * @brief ECG Service event.
 */
typedef struct
{
  ble_ecg_evt_type_t evt_type;     /**< Type of event. */
  uint16_t           conn_handle;  /**< Connection handle. */
}
ble_ecg_evt_t;

/* Forward declaration of the ble_ecg_t type. */
typedef struct ble_ecg_s ble_ecg_t;

/* ECG Service event handler type. */
typedef void (* ble_ecg_evt_handler_t) (ble_ecg_t * p_ecg, ble_ecg_evt_t * p_evt);

/**
 * @brief Nordic ECG Service initialization structure.
 */
typedef struct
{
  ble_ecg_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the ECG Service. */
  bool                   support_notification;           /**< TRUE if notification of ECG measurement is supported. */
  ble_srv_report_ref_t * p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the ECG characteristic */
  security_req_t         bl_rd_sec;                      /**< Security requirement for reading the BL characteristic value. */
  security_req_t         bl_cccd_wr_sec;                 /**< Security requirement for writing the BL characteristic CCCD. */
  security_req_t         bl_report_rd_sec;               /**< Security requirement for reading the BL characteristic descriptor. */
}
ble_ecg_init_t;

/**
 * @brief Nordic ECG Service structure.
 */
struct ble_ecg_s
{
  uint8_t                  uuid_type;                           /**< UUID type for ECG Service Base UUID. */
  ble_ecg_evt_handler_t    evt_handler;                         /**< Event handler to be called for handling events in the ECG Service. */
  uint16_t                 service_handle;                      /**< Handle of ECG Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t ecg_char_handles[BLE_ECG_MAX_CHAR];  /**< Handles related to the ECG characteristic. */
  uint16_t                 report_ref_handle;                   /**< Handle of the Report Reference descriptor. */
  bool                     is_notification_supported;           /**< TRUE if notification of ECG is supported. */
};

/* Public macros ------------------------------------------------------ */
/**
 * @brief  Macro for defining a ble_ecg instance.
 *
 * @param[in]     _name  Name of the instance.
 *
 * @attention     None
 *
 * @return        None
 */
#define BLE_ECG_DEF(_name)                        \
static ble_ecg_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,               \
                     BLE_HRS_BLE_OBSERVER_PRIO,   \
                     ble_ecg_on_ble_evt, &_name)

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief                     Function for initializing the Nordic ECG Service.
 *
 * @param[in]     p_ecg_init  Information needed to initialize the service.
 * 
 * @param[out]    p_ecg       Nordic ECG Service structure. This structure must be supplied
 *                            by the application. It is initialized by this function and will
 *                            later be used to identify this particular service instance.
 *
 * @attention     None
 *
 * @return
 * - NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * - NRF_ERROR_NULL If either of the pointers p_ecg or p_ecg_init is NULL.
 */
uint32_t ble_ecg_init(ble_ecg_t *p_ecg, ble_ecg_init_t const *p_ecg_init);

/**
 * @brief                        Function for updating the ECG level.
 *
 * @param[in]     p_bas          ECG Service structure.
 * @param[in]     acc            New ECG measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_ecg_update(ble_ecg_t *p_ecg, uint16_t data, uint16_t conn_handle, ble_ecg_charaterictic_t charac);

/**
 * @brief                     Function for handling the Nordic ECG Service's BLE events.
 *
 * @param[in]     p_ble_evt   Event received from the SoftDevice.
 * @param[in]     p_context   Nordic ECG Service structure.
 * 
 * @attention     None
 *
 * @return        None
 */
void ble_ecg_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

#endif // __BLE_ECG_H

/* End of file -------------------------------------------------------- */
