/**
 * @file       ble_acs.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      ACS (BLE Accelerometer Service)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BLE_ACS_H
#define __BLE_ACS_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

/* Public defines ----------------------------------------------------- */
#define BLE_UUID_ACS_SERVICE (0x1234) /**< The UUID of the Accelerometer Service. */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Accelerometer Charaterictic
 */
typedef enum
{
  BLE_ACS_AXIS_X_CHAR,
  BLE_ACS_AXIS_Y_CHAR,
  BLE_ACS_AXIS_Z_CHAR,
  BLE_ACS_MAX_CHAR
} 
ble_acs_charaterictic_t;

/**
 * @brief Accelerometer Service event type
 */
typedef enum
{
  BLE_ACS_EVT_NOTIFICATION_ENABLED, /**< Accelerometer value notification enabled event. */
  BLE_ACS_EVT_NOTIFICATION_DISABLED /**< Accelerometer value notification disabled event. */
} 
ble_acs_evt_type_t;

/**
 * @brief Accelerometer Service event.
 */
typedef struct
{
  ble_acs_evt_type_t evt_type;     /**< Type of event. */
  uint16_t           conn_handle;  /**< Connection handle. */
}
ble_acs_evt_t;

/* Forward declaration of the ble_acs_t type. */
typedef struct ble_acs_s ble_acs_t;

/* Accelerometer Service event handler type. */
typedef void (* ble_acs_evt_handler_t) (ble_acs_t * p_acs, ble_acs_evt_t * p_evt);

/**
 * @brief Nordic Accelerometer Service initialization structure.
 */
typedef struct
{
  ble_acs_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Accelerometer Service. */
  bool                   support_notification;           /**< TRUE if notification of Accelerometer measurement is supported. */
  ble_srv_report_ref_t * p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Accelerometer characteristic */
  security_req_t         bl_rd_sec;                      /**< Security requirement for reading the BL characteristic value. */
  security_req_t         bl_cccd_wr_sec;                 /**< Security requirement for writing the BL characteristic CCCD. */
  security_req_t         bl_report_rd_sec;               /**< Security requirement for reading the BL characteristic descriptor. */
}
ble_acs_init_t;

/**
 * @brief Nordic Accelerometer Service structure.
 */
struct ble_acs_s
{
  uint8_t                  uuid_type;                           /**< UUID type for Accelerometer Service Base UUID. */
  ble_acs_evt_handler_t    evt_handler;                         /**< Event handler to be called for handling events in the Accelerometer Service. */
  uint16_t                 service_handle;                      /**< Handle of Accelerometer Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t acc_char_handles[BLE_ACS_MAX_CHAR];  /**< Handles related to the Accelerometer characteristic. */
  uint16_t                 report_ref_handle;                   /**< Handle of the Report Reference descriptor. */
  bool                     is_notification_supported;           /**< TRUE if notification of Accelerometer is supported. */
};

/* Public macros ------------------------------------------------------ */
/**
 * @brief  Macro for defining a ble_acs instance.
 *
 * @param[in]     _name  Name of the instance.
 *
 * @attention     None
 *
 * @return        None
 */
#define BLE_ACS_DEF(_name)                        \
static ble_acs_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,               \
                     BLE_HRS_BLE_OBSERVER_PRIO,   \
                     ble_acs_on_ble_evt, &_name)

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief                     Function for initializing the Nordic Accelerometer Service.
 *
 * @param[in]     p_acs_init  Information needed to initialize the service.
 * 
 * @param[out]    p_acs       Nordic Accelerometer Service structure. This structure must be supplied
 *                            by the application. It is initialized by this function and will
 *                            later be used to identify this particular service instance.
 *
 * @attention     None
 *
 * @return
 * - NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * - NRF_ERROR_NULL If either of the pointers p_acs or p_acs_init is NULL.
 */
uint32_t ble_acs_init(ble_acs_t *p_acs, ble_acs_init_t const *p_acs_init);

/**
 * @brief                        Function for updating the Accelerometer level.
 *
 * @param[in]     p_bas          Accelerometer Service structure.
 * @param[in]     acc            New Accelerometer measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_acs_acc_update(ble_acs_t *p_acs, uint16_t acc, uint16_t conn_handle, ble_acs_charaterictic_t charac);

/**
 * @brief                     Function for handling the Nordic Accelerometer Service's BLE events.
 *
 * @param[in]     p_ble_evt   Event received from the SoftDevice.
 * @param[in]     p_context   Nordic Accelerometer Service structure.
 * 
 * @attention     None
 *
 * @return        None
 */
void ble_acs_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

#endif // __BLE_ACS_H

/* End of file -------------------------------------------------------- */
