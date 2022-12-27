#ifndef BLE_ATOM_H__
#define BLE_ATOM_H__

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

#define BLE_ATOM_DEF(_name)                                                                         \
static ble_atom_t _name;                                                                            \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_ATOM_BLE_OBSERVER_PRIO,                                                    \
                     ble_atom_on_ble_evt, &_name)

#define BLE_ATOM_MAX_LEN                                      19    
#define BLE_ATOM_MIN_LEN                                      1    

#define BLE_UUID_ATOM_SERVICE                 0x1223 /**< The UUID of the Atom Wattbike custom Service. */
#define BLE_UUID_ATOM_READINGS_CHARACTERISTIC 0x1224 /**< The UUID of the Atom Wattbike readings characteristic. */
#define BLE_UUID_ATOM_CTRLPT_CHAR             0x1225 /**< The UUID of the Atom Wattbike controlpoint characteristic. */
// Needs write

#define BLE_ATOM_MAX_READINGS_CHAR_LEN        BLE_ATOM_MAX_DATA_LEN /**< Maximum length of the readings Characteristic (in bytes). */

#define ATOM_BASE_UUID                        {{0xd1, 0x60, 0x28, 0xad, 0x17, 0x12, 0xb9, 0xad, 0xae, 0x4c, 0x02, 0xbc, 0x00, 0x00, 0xcc, 0xb4}}  /**< Used vendor specific UUID. */


// Enum for commands sent via the controlpoint characteristic (yet unknown)
typedef enum
{
   BLE_ATOM_CMD_UNKNOWN = 0x0,   
} ble_atom_cmd_t;

typedef enum
{
    BLE_ATOM_EVT_NOTIFICATION_ENABLED,                                  
    BLE_ATOM_EVT_NOTIFICATION_DISABLED,                                 
    BLE_ATOM_EVT_INDICATION_ENABLED,
    BLE_ATOM_EVT_INDICATION_DISABLED,    
} ble_atom_evt_type_t;

typedef struct
{
    ble_atom_evt_type_t evt_type;                                       /**< Type of event. */
    uint16_t            handle;
    const uint8_t *     data;
    uint16_t            len;
} ble_atom_evt_t;

typedef struct ble_atom_s ble_atom_t;

// Default definitions for potential command responses
typedef enum {
    BLE_ATOM_SUCCESS                                = 0x01,                               /**< Sucess Response. */
    BLE_ATOM_OP_CODE_NOT_SUPPORTED                  = 0x02,                               /**< Error Response received opcode not supported. */
    BLE_ATOM_INVALID_PARAMETER                      = 0x03,                               /**< Error Response received parameter invalid. */
    BLE_ATOM_OPERATION_FAILED                       = 0x04,                               /**< Error Response operation failed. */
    BLE_ATOM_CONTROL_NOT_PERMITTED                  = 0x05,    
} ble_atom_cmd_response_t;

// The message types
typedef enum {
    BLE_ATOM_INFO                                   = 0x03,
} ble_atom_message_t;

// The known submessage types
typedef enum {     
    BLE_ATOM_SUBMESSAGE_CURRENT_GEAR                = 0xb6,
    BLE_ATOM_SUBMESSAGE_UNKNOWN_C3                  = 0xc3,
    BLE_ATOM_SUBMESSAGE_UNKNOWN_CE                  = 0xce,
    BLE_ATOM_SUBMESSAGE_GEAR_CHANGE                 = 0xf0,
} ble_atom_submessage_t;

// The potential values for the gear up/down indication
typedef enum {
    BLE_ATOM_GEAR_UP =   0x02,
    BLE_ATOM_GEAR_DOWN = 0x04,
} ble_atom_gearchange_direction_t; 

// The potential detail values for gear up/down
typedef enum {
    BLE_ATOM_GEAR_CHANGE_1 = 0x01,
    BLE_ATOM_GEAR_CHANGE_4 = 0x04,
} ble_atom_gearchange_detailvalue_t; 

typedef struct
{
      uint8_t               sequence_numner;
      ble_atom_submessage_t commandbyte;
      ble_atom_submessage_t submessage;
      uint8_t               data[16];
} ble_atom_reading_data_t;  // 20 bytes

// Eventhandler definition
typedef void (*ble_atom_evt_handler_t) (ble_atom_t *p_atom, ble_atom_evt_t *p_evt);

typedef struct
{
    ble_atom_evt_handler_t       evt_handler;                           /**< Event handler to be called for handling events in the Cycling Power Service. */
    security_req_t               atom_readings_cccd_wr_sec;             /**< Security requirement for writing readings CCCD. */
    security_req_t               atom_readings_rd_sec;                  /**< Security requirement for reading readings characteristics. */
    security_req_t               atom_ctrlpt_cccd_wr_sec;               /**< Security requirement for writing control point characteristic CCCD. */
    security_req_t               atom_ctrlpt_wr_sec;                    /**< Security requirement for writing control point characteristic. */
    ble_srv_error_handler_t      error_handler;                         /**< Function to be called in case of an error. */    
    volatile uint8_t *           gear_offset;                           /**< Pointer to the variable holding the gear_offset */
} ble_atom_init_t;

struct ble_atom_s
{
    uint8_t                      uuid_type;                             /**< UUID characteritics type. */
    uint16_t                     conn_handle;                           /**< Client connection handle. */
    ble_atom_evt_handler_t       evt_handler;                           /**< Event handler to be called for handling events in the Atom Service. */
    uint16_t                     service_handle;                        /**< Handle of Atom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     readings_handles;                      /**< Handles related to the Atom readings characteristic. */
    ble_gatts_char_handles_t     ctrlpt_handles;            
};

// Function prototypes
uint32_t ble_atom_init(ble_atom_t * p_atom, ble_atom_init_t const * p_atom_init);
uint32_t ble_atom_measurement_send(ble_atom_t * p_atom, ble_atom_reading_data_t * p_measurement, uint8_t datalen);
void ble_atom_on_ble_evt(ble_evt_t const * p_atom_evt, void * p_context);
void ble_atom_send_gear_shift_notification(ble_atom_t *p_atom, ble_atom_gearchange_direction_t direction);

#ifdef __cplusplus
}
#endif

#endif // BLE_ATOM_H__