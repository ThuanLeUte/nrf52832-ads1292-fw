/**
 * @file       ble_gys.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      GYS (BLE Gyroscope Service)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BLE_GYS_H
#define __BLE_GYS_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

/* Public defines ----------------------------------------------------- */
#define BLE_UUID_GYS_SERVICE (0x3234) /**< The UUID of the Gyroscope Service. */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Gyroscope Charaterictic
 */
typedef enum
{
  BLE_GYS_AXIS_X_CHAR,
  BLE_GYS_AXIS_Y_CHAR,
  BLE_GYS_AXIS_Z_CHAR,
  BLE_GYS_MAX_CHAR
} 
ble_gys_charaterictic_t;

/**
 * @brief Gyroscope Service event type
 */
typedef enum
{
  BLE_GYS_EVT_NOTIFICATION_ENABLED, /**< Gyroscope value notification enabled event. */
  BLE_GYS_EVT_NOTIFICATION_DISABLED /**< Gyroscope value notification disabled event. */
} 
ble_gys_evt_type_t;

/**
 * @brief Gyroscope Service event.
 */
typedef struct
{
  ble_gys_evt_type_t evt_type;     /**< Type of event. */
  uint16_t           conn_handle;  /**< Connection handle. */
}
ble_gys_evt_t;

/* Forward declaration of the ble_gys_t type. */
typedef struct ble_gys_s ble_gys_t;

/* Gyroscope Service event handler type. */
typedef void (* ble_gys_evt_handler_t) (ble_gys_t * p_gys, ble_gys_evt_t * p_evt);

/**
 * @brief Nordic Gyroscope Service initialization structure.
 */
typedef struct
{
  ble_gys_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Gyroscope Service. */
  bool                   support_notification;           /**< TRUE if notification of Gyroscope measurement is supported. */
  ble_srv_report_ref_t * p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Gyroscope characteristic */
  security_req_t         bl_rd_sec;                      /**< Security requirement for reading the BL characteristic value. */
  security_req_t         bl_cccd_wr_sec;                 /**< Security requirement for writing the BL characteristic CCCD. */
  security_req_t         bl_report_rd_sec;               /**< Security requirement for reading the BL characteristic descriptor. */
}
ble_gys_init_t;

/**
 * @brief Nordic Gyroscope Service structure.
 */
struct ble_gys_s
{
  uint8_t                  uuid_type;                           /**< UUID type for Gyroscope Service Base UUID. */
  ble_gys_evt_handler_t    evt_handler;                         /**< Event handler to be called for handling events in the Gyroscope Service. */
  uint16_t                 service_handle;                      /**< Handle of Gyroscope Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t gyr_char_handles[BLE_GYS_MAX_CHAR];  /**< Handles related to the Gyroscope characteristic. */
  uint16_t                 report_ref_handle;                   /**< Handle of the Report Reference descriptor. */
  bool                     is_notification_supported;           /**< TRUE if notification of Gyroscope is supported. */
};

/* Public macros ------------------------------------------------------ */
/**
 * @brief  Macro for defining a ble_gys instance.
 *
 * @param[in]     _name  Name of the instance.
 *
 * @attention     None
 *
 * @return        None
 */
#define BLE_GYS_DEF(_name)                        \
static ble_gys_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,               \
                     BLE_HRS_BLE_OBSERVER_PRIO,   \
                     ble_gys_on_ble_evt, &_name)

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief                     Function for initializing the Nordic Gyroscope Service.
 *
 * @param[in]     p_gys_init  Information needed to initialize the service.
 * 
 * @param[out]    p_gys       Nordic Gyroscope Service structure. This structure must be supplied
 *                            by the application. It is initialized by this function and will
 *                            later be used to identify this particular service instance.
 *
 * @attention     None
 *
 * @return
 * - NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * - NRF_ERROR_NULL If either of the pointers p_gys or p_gys_init is NULL.
 */
uint32_t ble_gys_init(ble_gys_t *p_gys, ble_gys_init_t const *p_gys_init);

/**
 * @brief                        Function for updating the Gyroscope level.
 *
 * @param[in]     p_bas          Gyroscope Service structure.
 * @param[in]     gyr            New Gyroscope measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_gys_gyr_update(ble_gys_t *p_gys, uint16_t gyr, uint16_t conn_handle, ble_gys_charaterictic_t charac);

/**
 * @brief                     Function for handling the Nordic Gyroscope Service's BLE events.
 *
 * @param[in]     p_ble_evt   Event received from the SoftDevice.
 * @param[in]     p_context   Nordic Gyroscope Service structure.
 * 
 * @attention     None
 *
 * @return        None
 */
void ble_gys_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

#endif // __BLE_GYS_H

/* End of file -------------------------------------------------------- */
