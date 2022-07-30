/**
 * @file       ble_mgs.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      MGS (BLE Magnetometer Service)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BLE_MGS_H
#define __BLE_MGS_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

/* Public defines ----------------------------------------------------- */
#define BLE_UUID_MGS_SERVICE (0x2234) /**< The UUID of the Magnetometer Service. */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Magnetometer Charaterictic
 */
typedef enum
{
  BLE_MGS_AXIS_X_CHAR,
  BLE_MGS_AXIS_Y_CHAR,
  BLE_MGS_AXIS_Z_CHAR,
  BLE_MGS_MAX_CHAR
} 
ble_mgs_charaterictic_t;

/**
 * @brief Magnetometer Service event type
 */
typedef enum
{
  BLE_MGS_EVT_NOTIFICATION_ENABLED, /**< Magnetometer value notification enabled event. */
  BLE_MGS_EVT_NOTIFICATION_DISABLED /**< Magnetometer value notification disabled event. */
} 
ble_mgs_evt_type_t;

/**
 * @brief Magnetometer Service event.
 */
typedef struct
{
  ble_mgs_evt_type_t evt_type;     /**< Type of event. */
  uint16_t           conn_handle;  /**< Connection handle. */
}
ble_mgs_evt_t;

/* Forward declaration of the ble_mgs_t type. */
typedef struct ble_mgs_s ble_mgs_t;

/* Magnetometer Service event handler type. */
typedef void (* ble_mgs_evt_handler_t) (ble_mgs_t * p_mgs, ble_mgs_evt_t * p_evt);

/**
 * @brief Nordic Magnetometer Service initialization structure.
 */
typedef struct
{
  ble_mgs_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Magnetometer Service. */
  bool                   support_notification;           /**< TRUE if notification of Magnetometer measurement is supported. */
  ble_srv_report_ref_t * p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Magnetometer characteristic */
  security_req_t         bl_rd_sec;                      /**< Security requirement for reading the BL characteristic value. */
  security_req_t         bl_cccd_wr_sec;                 /**< Security requirement for writing the BL characteristic CCCD. */
  security_req_t         bl_report_rd_sec;               /**< Security requirement for reading the BL characteristic descriptor. */
}
ble_mgs_init_t;

/**
 * @brief Nordic Magnetometer Service structure.
 */
struct ble_mgs_s
{
  uint8_t                  uuid_type;                           /**< UUID type for Magnetometer Service Base UUID. */
  ble_mgs_evt_handler_t    evt_handler;                         /**< Event handler to be called for handling events in the Magnetometer Service. */
  uint16_t                 service_handle;                      /**< Handle of Magnetometer Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t acc_char_handles[BLE_MGS_MAX_CHAR];  /**< Handles related to the Magnetometer characteristic. */
  uint16_t                 report_ref_handle;                   /**< Handle of the Report Reference descriptor. */
  bool                     is_notification_supported;           /**< TRUE if notification of Magnetometer is supported. */
};

/* Public macros ------------------------------------------------------ */
/**
 * @brief  Macro for defining a ble_mgs instance.
 *
 * @param[in]     _name  Name of the instance.
 *
 * @attention     None
 *
 * @return        None
 */
#define BLE_MGS_DEF(_name)                        \
static ble_mgs_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,               \
                     BLE_HRS_BLE_OBSERVER_PRIO,   \
                     ble_mgs_on_ble_evt, &_name)

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief                     Function for initializing the Nordic Magnetometer Service.
 *
 * @param[in]     p_mgs_init  Information needed to initialize the service.
 * 
 * @param[out]    p_mgs       Nordic Magnetometer Service structure. This structure must be supplied
 *                            by the application. It is initialized by this function and will
 *                            later be used to identify this particular service instance.
 *
 * @attention     None
 *
 * @return
 * - NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * - NRF_ERROR_NULL If either of the pointers p_mgs or p_mgs_init is NULL.
 */
uint32_t ble_mgs_init(ble_mgs_t *p_mgs, ble_mgs_init_t const *p_mgs_init);

/**
 * @brief                        Function for updating the Magnetometer level.
 *
 * @param[in]     p_bas          Magnetometer Service structure.
 * @param[in]     acc            New Magnetometer measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_mgs_mag_update(ble_mgs_t *p_mgs, uint16_t acc, uint16_t conn_handle, ble_mgs_charaterictic_t charac);

/**
 * @brief                     Function for handling the Nordic Magnetometer Service's BLE events.
 *
 * @param[in]     p_ble_evt   Event received from the SoftDevice.
 * @param[in]     p_context   Nordic Magnetometer Service structure.
 * 
 * @attention     None
 *
 * @return        None
 */
void ble_mgs_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

#endif // __BLE_MGS_H

/* End of file -------------------------------------------------------- */
