#ifndef BLE_UDS_H__
#define BLE_UDS_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_UDS_DEF(_name)                                                                         \
static ble_uds_t _name;                                                                            \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_UDS_BLE_OBSERVER_PRIO,                                                    \
                     ble_uds_on_ble_evt, &_name)

#define BLE_UUID_UDS_SERVICE           0x181C /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_UDS_CTRLPT_CHAR       0x2A9F 
#define BLE_UUID_LASTNAME_CHAR         0x2a90
#define BLE_UUID_FIRSTNAME_CHAR        0x2A8A
#define BLE_UUIS_GENDER_CHAR           0x2A8C
#define BLE_UUID_HEIGHT_CHAR           0x2A8E
#define BLE_UUID_WEIGHT_CHAR           0x2A98
#define BLE_UUID_CHANGE_INCREMENT_CHAR 0x2A99
#define BLE_UUID_USER_INDEX_CHAR       0x2A9A

#define BLE_UDS_MAX_LEN                                      19    
#define BLE_UDS_MIN_LEN                                      1    


#define BLE_UDS_MAX_RX_CHAR_LEN        BLE_UDS_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */

#define BLE_UDS_MAX_TX_CHAR_LEN        BLE_UDS_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */


#define BLE_SRV_UDS_CTRLPT_REGISTER_NEW_USER  0x01                             /**< Support for sensor location related operations */
#define BLE_SRV_UDS_CTRLPT_CONSENT           0x02                             /**< Support for setting cumulative value related operations */
#define BLE_SRV_UDS_CTRLPT_DELETE_USER_SUPPORTED       0x04                             /**< Support for starting calibration related operations */
#define BLE_SRV_UDS_CTRLPT_DELETE_USER_DATA_SUPPORTED       0x08                             /**< Support for starting calibration related operations */
#define BLE_SRV_UDS_CTRLPT_LIST_ALL_USERS_SUPPORTED       0x10                             /**< Support for starting calibration related operations */
#define BLE_SRV_UDS_CTRLPT_RESPONSE_CODE_SUPPORTED       0x20                             /**< Support for starting calibration related operations */


typedef enum
{
   BLE_UDS_CMD_UNKNOWN = 0x0,
} ble_uds_cmd_t;

typedef enum
{
    BLE_UDS_EVT_NOTIFICATION_ENABLED,                                  /**< Cycling Speed and Cadence value notification enabled event. */
    BLE_UDS_EVT_NOTIFICATION_DISABLED,                                  /**< Cycling Speed and Cadence value notification disabled event. */
    BLE_UDS_EVT_INDICATION_ENABLED,
    BLE_UDS_EVT_INDICATION_DISABLED,    
    BLE_UDS_CHGINCREMENT_NOTIFICATION_ENABLED,
    BLE_UDS_CHGINCREMENT_NOTIFICATION_DISABLED,
    BLE_UDS_CTRLPT_EVT_REGISTER_NEW_USER,
    BLE_UDS_CTRLPT_EVT_CONSENT,
    BLE_UDS_CTRLPT_EVT_DELETE_USER,
    BLE_UDS_CTRLPT_EVT_DELETE_USER_DATA,
    BLE_UDS_CTRLPT_EVT_LIST_ALL_USERS,
    BLE_UDS_CTRLPT_EVT_RESPONSE_CODE

} ble_uds_evt_type_t;

typedef struct
{
    ble_uds_evt_type_t evt_type;                                       /**< Type of event. */
    uint16_t    handle;
    const uint8_t    *data;
    uint16_t    len;
} ble_uds_evt_t;

typedef struct ble_uds_s ble_uds_t;

typedef enum {
    BLE_UDS_SUCCESS                                = 0x01,                               /**< Sucess Response. */
    BLE_UDS_OP_CODE_NOT_SUPPORTED                  = 0x02,                               /**< Error Response received opcode not supported. */
    BLE_UDS_INVALID_PARAMETER                      = 0x03,                               /**< Error Response received parameter invalid. */
    BLE_UDS_OPERATION_FAILED                       = 0x04,                               /**< Error Response operation failed. */
} ble_uds_response_t;


typedef enum {
    BLE_UDSPT_RESERVERD                              = 0x00,
    BLE_UDSPT_SUCCESS                                = 0x01,                               /**< Sucess Response. */
    BLE_UDSPT_OP_CODE_NOT_SUPPORTED                  = 0x02,                               /**< Error Response received opcode not supported. */
    BLE_UDSPT_INVALID_PARAMETER                      = 0x03,                               /**< Error Response received parameter invalid. */
    BLE_UDSPT_OPERATION_FAILED                       = 0x04,                               /**< Error Response operation failed. */
    BLE_UDSPT_USER_NOT_AUTHORIZED                    = 0x05
} ble_udspt_response_t;

typedef enum {
    BLE_UDSPT_NO_PROC_IN_PROGRESS                    = 0x00,                               /**< No procedure in progress. */
    BLE_UDSPT_AUTOMATIC_CALIB_IN_PROGRESS            = 0x01,                               /**< Automatic Calibration is in progress. */
    BLE_UDSPT_INDICATION_PENDING                     = 0x02,                               /**< Control Point Indication is pending. */
    BLE_UDSPT_IND_CONFIRM_PENDING                    = 0x03,                               /**< Waiting for the indication confirmation. */
}ble_udspt_procedure_status_t;

typedef void (*ble_uds_evt_handler_t) (ble_uds_t * p_uds, ble_uds_evt_t * p_evt);
typedef struct
{
    ble_uds_evt_handler_t        evt_handler;                           /**< Event handler to be called for handling events in the Cycling Power Service. */
    security_req_t               uds_tx_cccd_rd_sec;                 /**< Security requirement for writing power control point characteristic CCCD. */
    security_req_t               uds_rx_cccd_wr_sec;                      /**< Security requirement for writing power control point characteristic. */
    security_req_t               uds_ctrlpt_cccd_wr_sec;
    security_req_t               uds_ctrlpt_wr_sec;
    security_req_t               uds_weight_wr_sec;
    security_req_t               uds_height_wr_sec;
    security_req_t               uds_change_increment_wr_sec;
    security_req_t               uds_user_index_rd_sec;
    uint8_t                      supported_functions;
    ble_srv_error_handler_t      error_handler;                         /**< Function to be called in case of an error. */        

} ble_uds_init_t;

typedef enum {    
    BLE_UDS_INDICATION_PENDING                     = 0x02,                               /**< Control Point Indication is pending. */
    BLE_UDS_IND_CONFIRM_PENDING                    = 0x03,                               /**< Waiting for the indication confirmation. */
}ble_uds_procedure_status_t;



typedef enum {
    BLE_UDS_RESERVED                        = 0x00,
    BLE_UDS_REGISTER_NEW_USER                                    = 0x01,                               /**< Operator to set a given cumulative value. */
    BLE_UDS_CONSENT            = 0x02,                               /**< Operator to start automatic calibration. */
    BLE_UDS_DELETE_USER_DATA                 = 0x03,                               /**< Operator to update the sensor location. */
    BLE_UDS_LIST_ALL_USERS     = 0x04,                               /**< Operator to request the supported sensor locations. */
    BLE_UDS_DELETE_USER                          = 0x05,                               /**< Response Code. */    
    BLE_UDS_RESPONSE_CODE = 0x20
} ble_udspt_operator_t;

typedef struct{
    ble_udspt_operator_t opcode;
    uint8_t              response_value;
    uint8_t       user_index;
    uint16_t        consent_code;
}ble_udspt_val_t;


typedef struct
{
    ble_udspt_operator_t         opcode;
    ble_uds_response_t           status;                                                  /**< control point response status .*/
    uint8_t                      len;                                                     /**< control point response length .*/
    uint8_t                      encoded_rsp[BLE_UDS_MAX_LEN];                 /**< control point encoded response.*/
    uint8_t responselen;
    uint8_t response;
} ble_uds_resp_t;

struct ble_uds_s
{
    uint8_t                         uuid_type; 
    ble_uds_evt_handler_t           evt_handler;                           /**< Event handler to be called for handling events in the Cycling Power Service. */
    uint16_t                        service_handle;                        /**< Handle of Cycling Power Service (as provided by the BLE stack). */    
    uint16_t                        conn_handle;                           /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */    
    ble_gatts_char_handles_t        ctrlpt_handle;    
    ble_gatts_char_handles_t        ud_weight_handle;
    ble_gatts_char_handles_t        ud_height_handle;
    ble_gatts_char_handles_t        ud_lastname_handle;
    ble_gatts_char_handles_t        ud_firstname_handle;
    ble_gatts_char_handles_t        ud_change_increment_handle;
    ble_gatts_char_handles_t        ud_index_handle;
    ble_uds_resp_t                  response;
    uint8_t                         supported_functions;
    ble_uds_procedure_status_t      procedure_status;
    ble_srv_error_handler_t      error_handler;                         /**< Function to be called in case of an error. */        
};
uint32_t ble_uds_init(ble_uds_t * p_uds, ble_uds_init_t const * p_uds_init);

void ble_uds_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
static bool is_cccd_configured(ble_uds_t * p_uds);
static void uds_ctrlpt_resp_send(ble_uds_t * p_uds);
static uint32_t uds_ctrlpt_decode(uint8_t const * p_rcvd_val, uint8_t len, ble_udspt_val_t * p_write_val);
static int ctrlpt_rsp_encode(ble_uds_t     * p_uds,          ble_uds_resp_t * p_rsp,                        uint8_t             * p_data);
static void on_uds_tx_complete(ble_uds_t * p_uds);
static void on_uds_hvc_confirm(ble_uds_t * p_uds, ble_evt_t const * p_ble_evt);
static void on_ctrlpt_write(ble_uds_t             * p_uds,                            ble_gatts_evt_write_t const * p_evt_write);
#ifdef __cplusplus
}
#endif

#endif // BLE_STERZO_H__