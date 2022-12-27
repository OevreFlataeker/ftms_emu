/******************************************************************
 * ble_atom.c
 * Implements part of the BLE characteristics of the Atom Wattbike
 * via means of the vendor specific ID
 *
 * Implemented feature/notification so far:
 * - Gear up/down indication
 *  
 ******************************************************************/

#include "sdk_common.h"

#if NRF_MODULE_ENABLED(BLE_ATOM)
#define NRF_LOG_MODULE_NAME ble_atom
#if BLE_ATOM_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_ATOM_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_ATOM_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_ATOM_CONFIG_DEBUG_COLOR
#else
#define NRF_LOG_LEVEL       0
#endif // BLE_ATOM_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#include "nrf_log_ctrl.h"
#include "helper.h"
#include "stdio.h"
#include "assert.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "math.h"
#include "ble_atom.h"
#include "cryptoutil.h"
#include "shimano.h"

#define LOGHERE 1

#define OPCODE_LENGTH 1                                                             /**< Length of opcode inside Atom Notification packet. */
#define HANDLE_LENGTH 2 

#define MAX_ATOMMM_LEN  (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)    /**< Maximum size of a transmitted Cycling Power Measurement. */

// Out message sequence number (roll over after 255)
uint8_t sequence_number = 0;

// Pointer to the global gear_offset value from main.c
volatile uint8_t *p_gear_offset = 0;

static void on_connect(ble_atom_t * p_atom, ble_evt_t const * p_ble_evt)
{    
    p_atom->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;    
}

static void on_disconnect(ble_atom_t * p_atom, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_atom->conn_handle = BLE_CONN_HANDLE_INVALID;    
}

static void on_atom_meas_cccd_write(ble_atom_t * p_atom, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // NRF_LOG_INFO("ble_atom::atom_meas_cccd_written");
        // CCCD written, update notification state
        if (p_atom->evt_handler != NULL)
        {
            ble_atom_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_ATOM_EVT_NOTIFICATION_ENABLED;                
            }
            else
            {
                evt.evt_type = BLE_ATOM_EVT_NOTIFICATION_DISABLED;
            }

            evt.handle = p_atom->readings_handles.cccd_handle;
            p_atom->evt_handler(p_atom, &evt);
        }
    }
}

static void on_atom_ctrlpt_cccd_write(ble_atom_t * p_atom, ble_gatts_evt_write_t const * p_evt_write)
{
    // NRF_LOG_INFO("ble_atom::on_atom_ctrlpt_cccd_write");
    if (p_evt_write->len == 2)
    {
        // NRF_LOG_INFO("ble_atom::atom_ctrlpt_cccd_written");
        // CCCD written, update notification state
        if (p_atom->evt_handler != NULL)
        {
            ble_atom_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_ATOM_EVT_NOTIFICATION_ENABLED;               
            }
            else
            {
                evt.evt_type = BLE_ATOM_EVT_NOTIFICATION_DISABLED;
            }

            if (ble_srv_is_indication_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_ATOM_EVT_INDICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_ATOM_EVT_INDICATION_DISABLED;
            }

            evt.handle = p_atom->ctrlpt_handles.cccd_handle;
            p_atom->evt_handler(p_atom, &evt);
        }
    }
}

static void on_write(ble_atom_t * p_atom, ble_evt_t const * p_ble_evt)
{
    ble_atom_evt_t evt;

    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if (p_evt_write->uuid.uuid == 0x2902) // CCCD
    {
        
        if (p_evt_write->handle == p_atom->readings_handles.cccd_handle)
        {   
          // NRF_LOG_DEBUG("Want to write to UUID 2902 of meas");
          on_atom_meas_cccd_write(p_atom, p_evt_write);
        }                
        else if (p_evt_write->handle == p_atom->ctrlpt_handles.cccd_handle)
        {
          // NRF_LOG_DEBUG("Want to write to UUID 2902 of controlpoint");
          on_atom_ctrlpt_cccd_write(p_atom, p_evt_write);
        }        
    }

    if (p_evt_write->handle == p_atom->ctrlpt_handles.value_handle)
    {
        // NRF_LOG_INFO("Data sent to ctrlpoint value handle:");
        // NRF_LOG_HEXDUMP_INFO(p_evt_write->data, p_evt_write->len);
        
        /*        
        ble_atom_cmd_t command = parse_atom_command(p_evt_write->data, p_evt_write->len);
        switch (command)
        {                                                   
            default:
                break;
        } 
        */
    }
}


void ble_atom_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_atom_t * p_atom = (ble_atom_t *)p_context;

    if (p_atom == NULL || p_ble_evt == NULL)
    {
        return;
    }
   
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            // NRF_LOG_INFO("ble_atom::ble_atom_on_ble_evt::BLE_GAP_EVT_CONNECTED");
            on_connect(p_atom, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            // NRF_LOG_INFO("ble_atom::ble_atom_on_ble_evt::BLE_GAP_EVT_DISCONNECTED");
            on_disconnect(p_atom, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            // NRF_LOG_DEBUG("ble_atom::ble_atom_on_ble_evt::BLE_GATTS_EVT_WRITE");
            on_write(p_atom, p_ble_evt);
            break;
        
        default:
            // No implementation needed.
            // NRF_LOG_DEBUG("ble_atom::ble_atom_on_ble_evt::Event unknown - 0x%0x", p_ble_evt->header.evt_id);
            break;
    }
}

uint32_t ble_atom_init(ble_atom_t * p_atom, ble_atom_init_t const * p_atom_init)
{ 
    ble_uuid128_t         atom_base_uuid = ATOM_BASE_UUID;
    uint32_t              err_code;
    uint8_t               init_value_encoded[MAX_ATOMMM_LEN];    
    ble_add_char_params_t add_char_params;
    ble_uuid_t            ble_uuid;
 
    if (p_atom == NULL || p_atom_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

   
    err_code = sd_ble_uuid_vs_add(&atom_base_uuid, &p_atom->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    // Initialize service structure
    p_atom->evt_handler = p_atom_init->evt_handler;    
    p_atom->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_gear_offset = p_atom_init->gear_offset;
            
    // Add service
    ble_uuid.type = p_atom->uuid_type;
    ble_uuid.uuid = BLE_UUID_ATOM_SERVICE;   
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_atom->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
   
    memset(&add_char_params, 0, sizeof(add_char_params));
    memset(init_value_encoded, 0, sizeof(init_value_encoded));

    add_char_params.uuid            = BLE_UUID_ATOM_READINGS_CHARACTERISTIC;     
    add_char_params.uuid_type       = p_atom->uuid_type;
    add_char_params.max_len         = 20;
    add_char_params.is_var_len      = true;
    add_char_params.init_len        = 0;
    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.notify = true;
    add_char_params.cccd_write_access = SEC_OPEN; 

    err_code = characteristic_add(p_atom->service_handle, 
                                  &add_char_params, 
                                  &p_atom->readings_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
           
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = BLE_UUID_ATOM_CTRLPT_CHAR; 
    add_char_params.uuid_type           = p_atom->uuid_type;
    add_char_params.max_len             = 20;
    add_char_params.init_len            = 0;
    add_char_params.is_var_len          = true;
    add_char_params.p_init_value        = init_value_encoded;
    
    add_char_params.char_props.write    = true;
    add_char_params.write_access        = SEC_OPEN;

    err_code = characteristic_add(p_atom->service_handle,
                              &add_char_params,
                              &p_atom->ctrlpt_handles);
    APP_ERROR_CHECK(err_code);        
    return err_code;
}

void ble_atom_send_gear_shift_notification(ble_atom_t *p_atom, ble_atom_gearchange_direction_t direction)
{
    ble_atom_reading_data_t data;
    uint32_t err_code;
   
    memset(&data, 0, sizeof(ble_atom_reading_data_t));
    data.sequence_numner = ++sequence_number;
    data.commandbyte = BLE_ATOM_INFO;
    data.submessage  = BLE_ATOM_SUBMESSAGE_GEAR_CHANGE;
    switch (direction)
    {
        case BLE_ATOM_GEAR_UP:
            data.data[0] = BLE_ATOM_GEAR_UP;
            data.data[1] = BLE_ATOM_GEAR_CHANGE_1;
            // NRF_LOG_INFO("Sending gear shift notificiation UP");
            // NRF_LOG_HEXDUMP_INFO(&data, 5);
            break;
        case BLE_ATOM_GEAR_DOWN:
            data.data[0] = BLE_ATOM_GEAR_DOWN;
            data.data[1] = BLE_ATOM_GEAR_CHANGE_1;
            // NRF_LOG_INFO("Sending gear shift notificiation DOWN");
            // NRF_LOG_HEXDUMP_INFO(&data, 5);
            break;
    }
    err_code = ble_atom_measurement_send(p_atom, &data, 5);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
         APP_ERROR_HANDLER(err_code);
    }   

    memset(&data, 0, sizeof(ble_atom_reading_data_t));
    data.sequence_numner = ++sequence_number;
    data.commandbyte = BLE_ATOM_INFO;
    data.submessage  = BLE_ATOM_SUBMESSAGE_CURRENT_GEAR;
    data.data[0] = *p_gear_offset + INITIAL_GEAR;
    // NRF_LOG_INFO("Sending gear status notificiation");
    // NRF_LOG_HEXDUMP_INFO(&data, 4);
    err_code = ble_atom_measurement_send(p_atom, &data, 4);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
         APP_ERROR_HANDLER(err_code);
    }   
}

uint32_t ble_atom_measurement_send(ble_atom_t * p_atom, ble_atom_reading_data_t* p_data, uint8_t datalen)
{    
    uint32_t err_code;

    if (p_atom == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    // Send value if connected and notifying
    if (p_atom->conn_handle != BLE_CONN_HANDLE_INVALID)
    {        
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;
        
        len     = datalen; 
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_atom->readings_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = (uint8_t *) p_data; 
        
        err_code = sd_ble_gatts_hvx(p_atom->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

#endif // NRF_MODULE_ENABLED(BLE_ATOM)
