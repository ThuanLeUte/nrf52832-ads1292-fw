/**
 * @file       ble_sts.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      Setting (BLE Setting Service)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BLE_STS_H
#define __BLE_STS_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

/* Public defines ----------------------------------------------------- */
#define BLE_UUID_STS_SERVICE (0x5234) /**< The UUID of the Setting Service. */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Setting Charaterictic
 */
typedef enum
{
  BLE_STS_GET_RECORD_CHAR,
  BLE_STS_MAX_CHAR
} 
ble_sts_charaterictic_t;

/**
 * @brief Setting Service event type
 */
typedef enum
{
  BLE_STS_EVT_NOTIFICATION_ENABLED, /**< Setting value notification enabled event. */
  BLE_STS_EVT_NOTIFICATION_DISABLED /**< Setting value notification disabled event. */
} 
ble_sts_evt_type_t;

/**
 * @brief Setting Service event.
 */
typedef struct
{
  ble_sts_evt_type_t evt_type;     /**< Type of event. */
  uint16_t           conn_handle;  /**< Connection handle. */
}
ble_sts_evt_t;

/* Forward declaration of the ble_sts_t type. */
typedef struct ble_sts_s ble_sts_t;

/* Setting Service event handler type. */
typedef void (* ble_sts_evt_handler_t) (ble_sts_t * p_sts, ble_sts_evt_t * p_evt);

/**
 * @brief Nordic Setting Service initialization structure.
 */
typedef struct
{
  ble_sts_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Setting Service. */
  bool                   support_notification;           /**< TRUE if notification of Setting measurement is supported. */
  ble_srv_report_ref_t * p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Setting characteristic */
  security_req_t         bl_rd_sec;                      /**< Security requirement for reading the BL characteristic value. */
  security_req_t         bl_cccd_wr_sec;                 /**< Security requirement for writing the BL characteristic CCCD. */
  security_req_t         bl_report_rd_sec;               /**< Security requirement for reading the BL characteristic descriptor. */
}
ble_sts_init_t;

/**
 * @brief Nordic Setting Service structure.
 */
struct ble_sts_s
{
  uint8_t                  uuid_type;                           /**< UUID type for Setting Service Base UUID. */
  ble_sts_evt_handler_t    evt_handler;                         /**< Event handler to be called for handling events in the Setting Service. */
  uint16_t                 service_handle;                      /**< Handle of Setting Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t sts_char_handles[BLE_STS_MAX_CHAR];  /**< Handles related to the Setting characteristic. */
  uint16_t                 report_ref_handle;                   /**< Handle of the Report Reference descriptor. */
  bool                     is_notification_supported;           /**< TRUE if notification of Setting is supported. */
};

/* Public macros ------------------------------------------------------ */
/**
 * @brief  Macro for defining a ble_sts instance.
 *
 * @param[in]     _name  Name of the instance.
 *
 * @attention     None
 *
 * @return        None
 */
#define BLE_STS_DEF(_name)                        \
static ble_sts_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,               \
                     BLE_HRS_BLE_OBSERVER_PRIO,   \
                     ble_sts_on_ble_evt, &_name)

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief                     Function for initializing the Nordic Setting Service.
 *
 * @param[in]     p_sts_init  Information needed to initialize the service.
 * 
 * @param[out]    p_sts       Nordic Setting Service structure. This structure must be supplied
 *                            by the application. It is initialized by this function and will
 *                            later be used to identify this particular service instance.
 *
 * @attention     None
 *
 * @return
 * - NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * - NRF_ERROR_NULL If either of the pointers p_sts or p_sts_init is NULL.
 */
uint32_t ble_sts_init(ble_sts_t *p_sts, ble_sts_init_t const *p_sts_init);

/**
 * @brief                        Function for updating the Setting level.
 *
 * @param[in]     p_bas          Setting Service structure.
 * @param[in]     acc            New Setting measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_sts_update(ble_sts_t *p_sts, uint8_t data, uint16_t conn_handle, ble_sts_charaterictic_t charac);

/**
 * @brief                     Function for handling the Nordic Setting Service's BLE events.
 *
 * @param[in]     p_ble_evt   Event received from the SoftDevice.
 * @param[in]     p_context   Nordic Setting Service structure.
 * 
 * @attention     None
 *
 * @return        None
 */
void ble_sts_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

#endif // __BLE_STS_H

/* End of file -------------------------------------------------------- */
