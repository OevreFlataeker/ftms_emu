/**
 * Copyright (c) 2020 daubsi
 *
 * 
 */

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_FTMS)

#include <string.h>
#include "ble_srv_common.h"
#include "ble_conn_state.h"

#define NRF_LOG_MODULE_NAME ble_ftms
#if BLE_FTMS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_FTMS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_FTMS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_FTMS_CONFIG_DEBUG_COLOR
#else // BLE_FTMS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_FTMS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include <string.h>
#include "nordic_common.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_util_bds.h"
#include "nrf_log.h"
#include "math.h"
#include "calculations.h"
#include "helper.h"

#include "oled_controller.h"

#define OPCODE_LENGTH 1                                                             /**< Length of opcode inside Cycling Power Measurement packet. */
#define HANDLE_LENGTH 2                                                             /**< Length of handle inside Cycling Power Measurement packet. */
#define MAX_FTMM_LEN  (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)    /**< Maximum size of a transmitted Cycling Power Measurement. */

// Cycling Power Measurement flag bits
//#define CP_MEAS_FLAG_MASK_WHEEL_REV_DATA_PRESENT (0x01 << 4)  /**< Wheel revolution data present flag bit. */
//#define CP_MEAS_FLAG_MASK_CRANK_REV_DATA_PRESENT (0x01 << 5)  /**< Crank revolution data present flag bit. */

// #define NRF_LOG_MODULE_NAME ble_ftms
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
// NRF_LOG_MODULE_REGISTER();
#include "ble_ftms.h"
#include "ble_cscs_c.h" 
#include "ble_cps.h"

ble_ftms_machine_status_t machine_status;
ble_ftms_indoor_bike_simulation_parameters_t ftms_indoor_bike_simulation_parameters;
ble_ftms_indoor_bike_data_t indoor_bike_data;
ble_ftms_training_status_t training_status;

#define LOGHERE 0
void set_ftms_incline(ble_ftms_t *p_ftms, int16_t target_incline)
{
    p_ftms->incline = target_incline;
}

void set_ftms_target_cadence(ble_ftms_t *p_ftms, uint16_t  target_cadence)
{
    p_ftms->target_cadence = target_cadence;
}

void set_ftms_resistance(ble_ftms_t *p_ftms, uint8_t  target_resistance)
{
    p_ftms->target_resistance = target_resistance;
    
    set_target_resistance(target_resistance);
}

void set_ftms_target_power(ble_ftms_t *p_ftms, int16_t target_power)
{
    p_ftms->target_power = target_power;
    // Dispatch change
    if (p_ftms->target_cadence == 0)
    {
        set_target_power(p_ftms->target_power, indoor_bike_data.average_cadence);
    }
    else
    {
        set_target_power(p_ftms->target_power, p_ftms->target_cadence);
    }    
}

void ftms_op_reset(ble_ftms_t * p_ftms)
{
    NRF_LOG_INFO("ftms::reset() called");
    // Update machine characteristic and notify

    p_ftms->control_acquired = false;

    
    training_status = BLE_FTMS_TRAINING_STATUS_IDLE;
    ble_ftms_training_status_send(p_ftms, &training_status);

    // Notify Fitness Machine Status RESET 0x01
    memset(&machine_status, 0, sizeof(ble_ftms_machine_status_t));
    machine_status.op_code = BLE_FTMS_MACHINE_STATUS_OP_CODE_RESET;
    // No response
    ble_ftms_machine_status_send(p_ftms, &machine_status);
}

void ftms_op_acquire_control(ble_ftms_t * p_ftms)
{
    NRF_LOG_INFO("ftms::ftms_op_acquire_control() called");
    // Update machine characteristic and notify

    training_status = BLE_FTMS_TRAINING_STATUS_IDLE;
    ble_ftms_training_status_send(p_ftms, &training_status);

    // Notify Fitness Machine Status STARTED_OR_RESUMED 0x04
    memset(&machine_status, 0, sizeof(ble_ftms_machine_status_t));
    machine_status.op_code = BLE_FTMS_MACHINE_STATUS_OP_CODE_STARTED_OR_RESUMED_BY_USER;
    // No response
    ble_ftms_machine_status_send(p_ftms, &machine_status);
    
    // Will stay on until reset or disconnect
    p_ftms->control_acquired = true;
   
}
void ftms_op_set_targeted_cadence(ble_ftms_t * p_ftms, uint16_t target_cadence)    
{
    NRF_LOG_INFO("ftms::ftms_opt_set_targeted_cadence() called with target_cadence = %u", target_cadence);
    set_ftms_target_cadence(p_ftms, target_cadence);
    
    // Update machine characteristic and notify
    memset(&machine_status, 0, sizeof(ble_ftms_machine_status_t));
    machine_status.op_code = BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGETED_CADENCE_CHANGED;
    machine_status.response.target_cadence_rpm = target_cadence;
    ble_ftms_machine_status_send(p_ftms, &machine_status);    
}
void ftms_op_set_target_inclination(ble_ftms_t * p_ftms, int16_t target_inclination)
{
    NRF_LOG_INFO("ftms::ftms_opt_set_target_inclination() called with target_inclination = %d", target_inclination);

    set_ftms_incline(p_ftms, target_inclination);
    
    // Update machine characteristic and notify
    memset(&machine_status, 0, sizeof(ble_ftms_machine_status_t));
    machine_status.op_code = BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_INCLINE_CHANGED;
    machine_status.response.target_incline_percent = target_inclination;
    ble_ftms_machine_status_send(p_ftms, &machine_status);    
}

void ftms_op_set_target_power(ble_ftms_t * p_ftms, int16_t target_power)
{
    NRF_LOG_INFO("ftms::ftms_opt_set_target_power() called with target_power = %d", target_power);
    set_ftms_target_power(p_ftms, target_power);
    
    // Update machine characteristic and notify
    memset(&machine_status, 0, sizeof(ble_ftms_machine_status_t));    
    machine_status.op_code = BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_POWER_CHANGED;
    machine_status.response.target_power_watts = target_power;
    ble_ftms_machine_status_send(p_ftms, &machine_status);
}

void ftms_op_set_target_resistance(ble_ftms_t * p_ftms, uint8_t target_resistance)
{
    NRF_LOG_INFO("ftms::ftms_opt_set_target_resistance() called with target_resistance = %u", target_resistance);
    set_ftms_resistance(p_ftms, target_resistance);
    
    // Update machine characteristic and notify
    memset(&machine_status, 0, sizeof(ble_ftms_machine_status_t));
    machine_status.op_code = BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_RESISTANCE_LEVEL_CHANGED;
    machine_status.response.target_resistance = target_resistance;
    ble_ftms_machine_status_send(p_ftms, &machine_status);
}

void ftms_op_start_resume_training(ble_ftms_t * p_ftms)
{
    // NRF_LOG_INFO("ftms::ftms_op_start_resume_training() called");

    // Update Trainer characteristic and notify
    memset(&machine_status, 0, sizeof(ble_ftms_machine_status_t));
    machine_status.op_code = BLE_FTMS_MACHINE_STATUS_OP_CODE_STARTED_OR_RESUMED_BY_USER;
    
    ble_ftms_machine_status_send(p_ftms, &machine_status);
    
    training_status = BLE_FTMS_TRAINING_STATUS_QUICKSTART;
    ble_ftms_training_status_send(p_ftms, &training_status);
}

void ftms_op_stop_pause_training(ble_ftms_t * p_ftms, uint8_t stop_pause_value)
{
    NRF_LOG_INFO("ftms::ftms_op_stop_pause_training() called with stop_pause_value = %u", stop_pause_value);
    
    // Update Trainer characteristic and notify
    memset(&machine_status, 0, sizeof(ble_ftms_machine_status_t));
    machine_status.op_code = BLE_FTMS_MACHINE_STATUS_OP_CODE_STOPPED_OR_PAUSED_BY_USER;
    // TODO: No response detail? Need to check spec
    ble_ftms_machine_status_send(p_ftms, &machine_status);

    training_status = BLE_FTMS_TRAINING_STATUS_IDLE;
    ble_ftms_training_status_send(p_ftms, &training_status);
}

void ftms_op_set_indoor_training_bike_parameters(ble_ftms_t * p_ftms,  ble_ftms_indoor_bike_simulation_parameters_t indoor_bike_simulation_parameters)
{    
    //NRF_LOG_INFO("ftms::ftms_op_set_indoor_training_bike_parameters() with wind_speed %d, grade %d, crr %u, cw %u", 
    //                                        indoor_bike_simulation_parameters.wind_speed,
    //                                        indoor_bike_simulation_parameters.grade,
    //                                        indoor_bike_simulation_parameters.crr,
    //                                        indoor_bike_simulation_parameters.cw
    //                                            );

  
    ftms_indoor_bike_simulation_parameters.wind_speed = indoor_bike_simulation_parameters.wind_speed;
    ftms_indoor_bike_simulation_parameters.grade = indoor_bike_simulation_parameters.grade;
    ftms_indoor_bike_simulation_parameters.crr = indoor_bike_simulation_parameters.crr;
    ftms_indoor_bike_simulation_parameters.cw = indoor_bike_simulation_parameters.cw;
    set_ftms_simulation_parameter(indoor_bike_simulation_parameters);
    
    // Update machine status
    memset(&machine_status, 0, sizeof(ble_ftms_machine_status_t));
    machine_status.op_code = BLE_FTMS_MACHINE_STATUS_OP_CODE_INDOOR_BIKE_SIMULATION_PARAMETERS_CHANGED;
    machine_status.response.new_indoor_bike_simulation_parameters = &ftms_indoor_bike_simulation_parameters;
    ble_ftms_machine_status_send(p_ftms, &machine_status);
}

//
// ...
//
void ble_ftms_on_cscs_evt(ble_ftms_t * p_ftms, ble_cscs_c_evt_t * p_cscs_c_evt)
{
    ret_code_t         err_code;

    // The "instantaneous" measurements are not the same as in CPS and CSCS
    // Here "instantaneous" is already the averaged value
    // The "average" here means: Since the beginning of the measurements

    indoor_bike_data.instantaneous_speed = (uint16_t) getAverageSpeed(); //i_kmh; // Kilometer per hour with a resolution of 0.01, Exponent: -2
    indoor_bike_data.average_speed = indoor_bike_data.instantaneous_speed; // a_kmh; // Kilometer per hour with a resolution of 0.01, Exponent: -2
    indoor_bike_data.instantaneous_cadence = (uint16_t) getAverageCadence(); // a_cadence; //TODO Also used in encode - cleanup! // 1/minute with a resolution of 0.5, Exponent: -1
    indoor_bike_data.average_cadence = indoor_bike_data.instantaneous_cadence; // a_cadence; // 1/minute with a resolution of 0.5, Exponent: -1
    indoor_bike_data.instantaneous_power = (int16_t) getInstantaneousPower(); // calculate_power_from_avg_cadence(getAverageCadence(),*(p_ftms->resistance_level));//TODO // i_power; // Watts with a resolution of 1 
    indoor_bike_data.average_power = indoor_bike_data.instantaneous_power;// TODO // a_power; // Watts with a resolution of 1
    indoor_bike_data.resistance_level = (int16_t) getResistanceLevel();
        
    err_code = ble_ftms_indoor_bike_data_send(p_ftms, &indoor_bike_data);
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


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ftms       Fitness Machine Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_ftms_t * p_ftms, ble_evt_t const * p_ble_evt)
{    
    p_ftms->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;    
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ftms       Fitness Machine Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_ftms_t * p_ftms, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ftms->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the CPS Measurement characteristic.
 *
 * @param[in]   p_ftms         Fitness Machine Service structure.
 * @param[in]   p_evt_write    Write event received from the BLE stack.
 */
static void on_indoor_bike_cccd_write(ble_ftms_t * p_ftms, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // NRF_LOG_INFO("indoor_bike_cccd_written");
        // CCCD written, update notification state
        if (p_ftms->evt_handler != NULL)
        {
            ble_ftms_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_FTMS_EVT_INDOORBIKEDATA_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_FTMS_EVT_INDOORBIKEDATA_NOTIFICATION_DISABLED;
            }

            p_ftms->evt_handler(p_ftms, &evt);
        }
    }
}


/**@brief Function for handling write events to the CPS Measurement characteristic.
 *
 * @param[in]   p_ftms         Fitness Machine Service structure.
 * @param[in]   p_evt_write    Write event received from the BLE stack.
 */
static void on_training_status_cccd_write(ble_ftms_t * p_ftms, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // NRF_LOG_INFO("training_status_cccd_written");
        // CCCD written, update notification state
        if (p_ftms->evt_handler != NULL)
        {
            ble_ftms_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_FTMS_EVT_TRAININGSTATUS_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_FTMS_EVT_TRAININGSTATUS_NOTIFICATION_DISABLED;
            }

            p_ftms->evt_handler(p_ftms, &evt);
        }
    }
}

static void on_ctrlpt_cccd_write(ble_ftms_t * p_ftms, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // NRF_LOG_INFO("ctrlpt_cccd_written");
        // CCCD written, update notification state
        if (p_ftms->evt_handler != NULL)
        {
            ble_ftms_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_FTMS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_FTMS_EVT_NOTIFICATION_DISABLED;
            }

            if (ble_srv_is_indication_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_FTMS_EVT_INDICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_FTMS_EVT_INDICATION_DISABLED;
            }
            p_ftms->evt_handler(p_ftms, &evt);
        }
    }
}

/**@brief Function for handling write events to the CPS Measurement characteristic.
 *
 * @param[in]   p_ftms         Fitness Machine Service structure.
 * @param[in]   p_evt_write    Write event received from the BLE stack.
 */
static void on_machine_status_cccd_write(ble_ftms_t * p_ftms, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // NRF_LOG_INFO("machine_status_cccd_written");
        // CCCD written, update notification state
        if (p_ftms->evt_handler != NULL)
        {
            ble_ftms_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_FTMS_EVT_MACHINESTATUS_NOTIFICATION_ENABLED;
               // NRF_LOG_INFO("machine_status notificiations enabled");
            }
            else
            {
                evt.evt_type = BLE_FTMS_EVT_MACHINESTATUS_NOTIFICATION_DISABLED;
               //  NRF_LOG_INFO("machine_status notificiations disabled");
            }

            p_ftms->evt_handler(p_ftms, &evt);
        }
    }
}


void ble_ftms_on_ftms_evt(ble_ftms_t *m_ftms, ble_ftms_evt_t * p_ftms_evt)
{
    ble_ftms_meas_t   ftms_measurment;
    ret_code_t         err_code;
    
    
    err_code = ble_ftms_measurement_send(m_ftms, &ftms_measurment);
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

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ftms       Fitness Machine Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_ftms_t * p_ftms, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // We'l probably need to differentiate here between the various charcs that can be notified
    /*if (p_evt_write->handle == p_ftms->meas_handles.cccd_handle)
    {
        on_meas_cccd_write(p_ftms, p_evt_write);
    }
    */
    if (p_evt_write->uuid.uuid == 0x2902) // CCCD
    {
        
        if (p_evt_write->handle == p_ftms->indoor_bike_handles.cccd_handle)
        {   
          on_indoor_bike_cccd_write(p_ftms, p_evt_write);
        }
        else if (p_evt_write->handle == p_ftms->machine_status_handles.cccd_handle)
        {   
          on_machine_status_cccd_write(p_ftms, p_evt_write);
        }
        else if (p_evt_write->handle == p_ftms->training_status_handles.cccd_handle)
        {   
          on_training_status_cccd_write(p_ftms, p_evt_write);
        }
        else if (p_evt_write->handle == p_ftms->ctrl_pt.ftms_ctrlpt_handles.cccd_handle)
        {
          on_ctrlpt_cccd_write(p_ftms, p_evt_write);
        }
#if LOGHERE
        else {
            NRF_LOG_INFO("CCCD handle 0x%04x unknown" , p_evt_write->handle);
            NRF_LOG_INFO("ble_ftms::on_write() gatts_evt: handle: 0x%02x, uuid: 0x%04x, type: 0x%02x, len: 0x%04x, data[0]=0x%02x", p_evt_write->handle, p_evt_write->uuid.uuid, p_evt_write->uuid.type, p_evt_write->len, p_evt_write->data[0]); 
      
        }  
#endif            
    }
    
    //NRF_LOG_INFO("ble_ftms::on_write() will need Impl!");
}


void ble_ftms_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_ftms_t * p_ftms = (ble_ftms_t *)p_context;

    if (p_ftms == NULL || p_ble_evt == NULL)
    {
        return;
    }


    ble_ftms_ctrlpt_on_ble_evt(&(p_ftms->ctrl_pt), p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //NRF_LOG_INFO("ble_ftms::ble_ftms_on_ble_evt::BLE_GAP_EVT_CONNECTED");
            on_connect(p_ftms, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //NRF_LOG_INFO("ble_ftms::ble_ftms_on_ble_evt::BLE_GAP_EVT_DISCONNECTED");
            on_disconnect(p_ftms, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            //NRF_LOG_INFO("ble_ftms::ble_ftms_on_ble_evt::BLE_GATTS_EVT_WRITE");
            on_write(p_ftms, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static uint8_t ftms_training_status_encode(ble_ftms_t      * p_ftms,
                                     ble_ftms_training_status_t * training_status,
                                     uint8_t        * p_encoded_buffer)
{
    uint8_t len = 0;
    uint8_t out_len = 0;

    ble_srv_utf8_str_t training_status_string;
    uint8_t out_buf[32];

    /**(&p_encoded_buffer[len++]) = 0;
    *(&p_encoded_buffer[len++]) = *training_status;
    return len;
    */
    // New code

    *(&p_encoded_buffer[len++]) = 1 << 0; // Flags: Training Status string present: Yes, Extended String present: No
    *(&p_encoded_buffer[len++]) = *training_status; // Training Status hex value
    
    switch (*training_status)
    {
        case BLE_FTMS_TRAINING_STATUS_OTHER:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Other");
            break;
        case BLE_FTMS_TRAINING_STATUS_IDLE:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Idle");
            break;
        case BLE_FTMS_TRAINING_STATUS_WARMING_UP:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Idle");
            break;
        case BLE_FTMS_TRAINING_STATUS_LOW_INTENSITY_INTERVAL:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "LII");
            break;
        case BLE_FTMS_TRAINING_STATUS_HIGH_INTENSITY_INTERVAL:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "HII");
            break;
        case BLE_FTMS_TRAINING_STATUS_RECOVERY_INTERVAL:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Recovery");
            break;
        case BLE_FTMS_TRAINING_STATUS_ISOMETRIC:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Isometric");
            break;
        case BLE_FTMS_TRAINING_STATUS_HEARTRATE_CONTROL:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Heart rate");
            break;
        case BLE_FTMS_TRAINING_STATUS_FITNESS_TEST:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Fitness Test");
            break;
        case BLE_FTMS_TRAINING_STATUS_SPEED_OUTSIDE_CONTROL_REGION_LOW:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Out Reg Low");
            break;
        case BLE_FTMS_TRAINING_STATUS_SPEED_OUTSIDE_CONTROL_REGION_HIGH:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Out Reg High");
            break;
        case BLE_FTMS_TRAINING_STATUS_COOL_DOWN:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Cool down");
            break;
        case BLE_FTMS_TRAINING_STATUS_WATT_CONTROL:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Watt Ctrl");
            break;
        case BLE_FTMS_TRAINING_STATUS_MANUAL_MODE:
            // Same internal ID with BLE_FTMS_TRAINING_STATUS_QUICKSTART
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Manual");
            break;        
        case BLE_FTMS_TRAINING_STATUS_PRE_WORKOUT:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Pre WO");
            break;
        case BLE_FTMS_TRAINING_STATUS_POST_WORKOUT:
            ble_srv_ascii_to_utf8(&training_status_string, (char *) "Post WO");
            break;
    }
    
    out_len = bds_ble_srv_utf8_str_encode(&training_status_string, out_buf);
    memcpy(&p_encoded_buffer[len], out_buf, out_len);
    len += out_len;

    return len;
}




static uint8_t ftms_indoor_bike_data_encode(ble_ftms_t      * p_ftms,
                                     ble_ftms_indoor_bike_data_t * indoor_bike_data,
                                     uint8_t        * p_encoded_buffer)
{
    uint8_t len   = 0; 

    // Setup flags bit
    uint16_t flags = 0; // Initialize MORE_DATA = 0
        
    if ((p_ftms->fitness_machine_feature.fitness_machine_features & BLE_FTMS_FEATURE_AVERAGE_SPEED_SUPPORTED_BIT) == BLE_FTMS_FEATURE_AVERAGE_SPEED_SUPPORTED_BIT)
    {
        // Add average speed
        //NRF_LOG_INFO("ftms: Setting flag AVGSPEED");
        flags |= BLE_FTMS_INDOOR_FLAGS_FIELD_AVERAGE_SPEED_PRESENT;
    }

    if ((p_ftms->fitness_machine_feature.fitness_machine_features & BLE_FTMS_FEATURE_CADENCE_SUPPORTED_BIT) == BLE_FTMS_FEATURE_CADENCE_SUPPORTED_BIT)
    {
    // Add Instantaneous cadenace AND average cadence
        //NRF_LOG_INFO("ftms: Setting flag INS CAD and AVG CAD");
        flags |= BLE_FTMS_INDOOR_FLAGS_FIELD_INSTANTANEOUS_CADENCE_PRESENT; // Instant
        flags |= BLE_FTMS_INDOOR_FLAGS_FIELD_AVERAGE_CADENCE_PRESENT; // Average // Apparently not absoluely necessary
    }

    if ((p_ftms->fitness_machine_feature.fitness_machine_features & BLE_FTMS_FEATURE_RESISTANCE_SUPPORTED_BIT) == BLE_FTMS_FEATURE_RESISTANCE_SUPPORTED_BIT)
    {
    // Add Instantaneous cadenace AND average cadence
        //NRF_LOG_INFO("ftms: Setting flag RESIST PRESENT");
        flags |= BLE_FTMS_INDOOR_FLAGS_FIELD_RESISTANCE_LEVEL_PRESENT; //         
    }

    if ((p_ftms->fitness_machine_feature.fitness_machine_features & BLE_FTMS_FEATURE_POWER_MEASUREMENT_SUPPORTED_BIT) == BLE_FTMS_FEATURE_POWER_MEASUREMENT_SUPPORTED_BIT)
    {
    // Add Instantaneous cadenace AND average power
        //NRF_LOG_INFO("ftms: Setting flag INS POW and AVG POW");
        flags |= BLE_FTMS_INDOOR_FLAGS_FIELD_INSTANTANEOUS_POWER_PRESENT; // Instant
        flags |= BLE_FTMS_INDOOR_FLAGS_FIELD_AVERAGE_POWER_PRESENT; // Average // Apparently not absoluely necessary https://github.com/erikboto
    }

    len += uint16_encode(flags, &p_encoded_buffer[len]);

    // Instantaneous speed is mandatory
    // Kilometer per hour with a resolution of 0.01
    // uint16 with 0.01 resolution, decimal point -2
    
    // Resolution = 0.01 --> we have to send the 100x value: 25.3 kmh -> 2530 to send. 
    len += uint16_encode(indoor_bike_data->instantaneous_speed * 100, &p_encoded_buffer[len]);
    
    if ((p_ftms->fitness_machine_feature.fitness_machine_features & BLE_FTMS_FEATURE_AVERAGE_SPEED_SUPPORTED_BIT) == BLE_FTMS_FEATURE_AVERAGE_SPEED_SUPPORTED_BIT)
    {
        // NRF_LOG_ERROR("Cannot encode average speed in indoor bike data point!"); 
          
        //len += uint16_encode(indoor_bike_data->average_speed, &p_encoded_buffer[len]);
        // Resolution = 0.01 --> we have to send the 100x value: 25.3 kmh -> 2530 to send. 
        len += uint16_encode(indoor_bike_data->average_speed * 100, &p_encoded_buffer[len]);
    }

    if ((p_ftms->fitness_machine_feature.fitness_machine_features & BLE_FTMS_FEATURE_CADENCE_SUPPORTED_BIT) == BLE_FTMS_FEATURE_CADENCE_SUPPORTED_BIT)
    {
        // Add Instantaneous cadenace AND average cadence
        // Instantaneous: uint16, decimal exponent -2, resolution 0.5 --> We have to double the value --> cadence of 76 rpm -> send 152
        len += uint16_encode(indoor_bike_data->instantaneous_cadence * 2, &p_encoded_buffer[len]);

        // Average: uint16, , decimal exponent -2
        len += uint16_encode(indoor_bike_data->average_cadence * 2, &p_encoded_buffer[len]);
    }

    if ((p_ftms->fitness_machine_feature.fitness_machine_features & BLE_FTMS_FEATURE_RESISTANCE_SUPPORTED_BIT) == BLE_FTMS_FEATURE_RESISTANCE_SUPPORTED_BIT)
    {
        // Add resistance
        // Instantaneous: sint16, resolution 1 --> 
        //len += uint16_encode(incline_level * 10, &p_encoded_buffer[len]);        
        len += sint16_encode(resistance_level * 10, &p_encoded_buffer[len]); 
    }

    if ((p_ftms->fitness_machine_feature.fitness_machine_features & BLE_FTMS_FEATURE_POWER_MEASUREMENT_SUPPORTED_BIT) == BLE_FTMS_FEATURE_POWER_MEASUREMENT_SUPPORTED_BIT)
    {
        // Add Instantaneous AND average power
        // Instantaneous: int16 in watt
        //len += uint16_encode(indoor_bike_data->instantaneous_power, &p_encoded_buffer[len]);
        len += sint16_encode(indoor_bike_data->instantaneous_power, &p_encoded_buffer[len]);

        // Average: int16 in watt
        //len += uint16_encode(indoor_bike_data->average_power, &p_encoded_buffer[len]);
        len += sint16_encode(indoor_bike_data->average_power, &p_encoded_buffer[len]);
    }
    // NRF_LOG_INFO("Total length: %d", len);
    return len;
}


/**@brief Function for encoding a CPS Measurement.
 *
 * @param[in]   p_ftms               Cycling Power Service structure.
 * @param[in]   p_cp_measurement    Measurement to be encoded.
 * @param[out]  p_encoded_buffer    Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t ftms_machine_status_encode(ble_ftms_t      * p_ftms,
                                     ble_ftms_machine_status_t * ble_ftms_machine_status,
                                     uint8_t        * p_encoded_buffer)
{    
    uint8_t len   = 1; // Minimum length for flags
    *(&p_encoded_buffer[0]) = ble_ftms_machine_status->op_code;
    
    switch (ble_ftms_machine_status->op_code)
    {
        case BLE_FTMS_MACHINE_STATUS_OP_CODE_RESET:            
        case BLE_FTMS_MACHINE_STATUS_OP_CODE_STOPPED_OR_PAUSED_BY_SAFETY_KEY:
        case BLE_FTMS_MACHINE_STATUS_OP_CODE_STARTED_OR_RESUMED_BY_USER:
        case BLE_FTMS_MACHINE_STATUS_OP_CODE_CONTROL_PERMISSION_LOST:
            // These don' have any params
            break;           
        case BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_POWER_CHANGED:
            //len += uint16_encode(ble_ftms_machine_status->response.target_power_watts, &p_encoded_buffer[len]);
            len += sint16_encode(ble_ftms_machine_status->response.target_power_watts, &p_encoded_buffer[len]);
            break;
        case BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_RESISTANCE_LEVEL_CHANGED:
            *(&p_encoded_buffer[len++]) = ble_ftms_machine_status->response.target_resistance;            
            break;
        case BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_INCLINE_CHANGED:
            //len += uint16_encode(ble_ftms_machine_status->response.target_incline_percent, &p_encoded_buffer[len]);
            len += sint16_encode(ble_ftms_machine_status->response.target_incline_percent, &p_encoded_buffer[len]);
            break;
        case BLE_FTMS_MACHINE_STATUS_OP_CODE_STOPPED_OR_PAUSED_BY_USER:
            *(&p_encoded_buffer[len++]) = ble_ftms_machine_status->response.control_information;            
            break;
        case BLE_FTMS_MACHINE_STATUS_OP_CODE_INDOOR_BIKE_SIMULATION_PARAMETERS_CHANGED:
            // len += uint16_encode(ble_ftms_machine_status->response.new_indoor_bike_simulation_parameters->wind_speed, &p_encoded_buffer[len]);
            // len += uint16_encode(ble_ftms_machine_status->response.new_indoor_bike_simulation_parameters->grade, &p_encoded_buffer[len]);
            len += sint16_encode(ble_ftms_machine_status->response.new_indoor_bike_simulation_parameters->wind_speed, &p_encoded_buffer[len]);
            len += sint16_encode(ble_ftms_machine_status->response.new_indoor_bike_simulation_parameters->grade, &p_encoded_buffer[len]);
            *(&p_encoded_buffer[len++]) = ble_ftms_machine_status->response.new_indoor_bike_simulation_parameters->crr;
            *(&p_encoded_buffer[len++]) = ble_ftms_machine_status->response.new_indoor_bike_simulation_parameters->cw;            
            break;
        case BLE_FTMS_MACHINE_STATUS_OP_CODE_WHEEL_CIRCUMFENCE_CHANGED:
            len += uint16_encode(ble_ftms_machine_status->response.new_wheel_circumfence_mm, &p_encoded_buffer[len]);
            break;
        default:
            NRF_LOG_ERROR("Unhandled encoded response type: 0x%02x - Needs implementation", ble_ftms_machine_status->op_code);
    }
    
    return len;
}


uint32_t ble_ftms_init(ble_ftms_t * p_ftms, ble_ftms_init_t const * p_ftms_init)
{
    if (p_ftms == NULL || p_ftms_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t              err_code;
    uint8_t               init_value_encoded[MAX_FTMM_LEN];
    ble_ftms_meas_t       initial_scm = {0};
    ble_add_char_params_t add_char_params;
    ble_uuid_t            ble_uuid;
    ble_ftms_ctrlpt_init_t  ftms_ctrlpt_init;

    
    // Initialize service structure
    p_ftms->evt_handler = p_ftms_init->evt_handler;
    p_ftms->conn_handle = BLE_CONN_HANDLE_INVALID;

    p_ftms->fitness_machine_feature.fitness_machine_features     = p_ftms_init->fitness_machine_feature.fitness_machine_features;
    p_ftms->fitness_machine_feature.target_setting_features      = p_ftms_init->fitness_machine_feature.target_setting_features;

    p_ftms->machine_status = p_ftms_init->machine_status;
    p_ftms->training_status =  p_ftms_init->training_status;

    p_ftms->trainer_started = false;
    // We don' have control acquired on init
    p_ftms->control_acquired = false;
    //set_ftms_incline_rough(p_ftms, p_ftms_init->incline);

    p_ftms->resistance_level = p_ftms_init->resistance_level;
    memset(init_value_encoded, 0, sizeof(init_value_encoded));
    
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_FITNESS_MACHINE);
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_ftms->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
 
    // Add Fitness Machine feature characteristic  
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_FITNESS_MACHINE_FEATURE;
    add_char_params.max_len         = 2*sizeof(uint32_t);
    add_char_params.is_var_len      = false;
    add_char_params.init_len        = uint32_encode(p_ftms_init->fitness_machine_feature.fitness_machine_features, &init_value_encoded[0])
                                     +uint32_encode(p_ftms_init->fitness_machine_feature.target_setting_features, &init_value_encoded[4]);
    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = p_ftms_init->ftms_feature_rd_sec;

    err_code = characteristic_add(p_ftms->service_handle, 
                                  &add_char_params, 
                                  &p_ftms->feature_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }


    // Add Fitness Machine status characteristic    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_FITNESS_MACHINE_STATUS;
    add_char_params.max_len         = MAX_FTMM_LEN;;
    add_char_params.is_var_len      = true;
    add_char_params.init_len        = uint16_encode(p_ftms_init->machine_status, &init_value_encoded[0]);
    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = p_ftms_init->ftms_machine_status_rd_sec;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_ftms_init->ftms_cccd_wr_sec;

    err_code = characteristic_add(p_ftms->service_handle, 
                                  &add_char_params, 
                                  &p_ftms->machine_status_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Fitness Machine indoor bike characteristic    
    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_FITNESS_MACHINE_INDOOR_BIKE;
    add_char_params.max_len         = MAX_FTMM_LEN;
    add_char_params.is_var_len      = true;
    
    add_char_params.init_len        = 0; // uint16_encode(p_ftms_init->indoor_bike, &init_value_encoded[0]);
    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = p_ftms_init->ftms_indoor_bike_rd_sec;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_ftms_init->ftms_cccd_wr_sec;

    err_code = characteristic_add(p_ftms->service_handle, 
                                  &add_char_params, 
                                  &p_ftms->indoor_bike_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Fitness Machine training status characteristic        
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_FITNESS_MACHINE_TRAINING_STATUS;
    add_char_params.max_len         = MAX_FTMM_LEN;
    add_char_params.is_var_len      = true;
    
    add_char_params.init_len        = uint16_encode(p_ftms_init->training_status, &init_value_encoded[0]);    
    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.read = 1;
    add_char_params.char_props.notify = 1;
    add_char_params.read_access     = p_ftms_init->ftms_training_status_rd_sec;
    add_char_params.cccd_write_access = p_ftms_init->ftms_cccd_wr_sec;

    err_code = characteristic_add(p_ftms->service_handle, 
                                  &add_char_params, 
                                  &p_ftms->training_status_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add supported power range characteristic    
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid            = BLE_UUID_SUPPORTED_POWER_RANGE;
    add_char_params.max_len         = 3 * sizeof(uint16_t);
    add_char_params.init_len        = 3; // uint16_encode(p_ftms_init->feature, &init_value_encoded[0]);

    sint16_encode(BLE_FTMS_POWER_MINIMUM, &init_value_encoded[0]); // Minimum Watts
    sint16_encode(BLE_FTMS_POWER_MAXIMUM, &init_value_encoded[2]); // Maximum Watts
    uint16_encode(BLE_FTMS_POWER_INCREMENT, &init_value_encoded[4]); // Minimum Increment
    
    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = p_ftms_init->ftms_sup_power_range_rd_sec;

    err_code = characteristic_add(p_ftms->service_handle, 
                                  &add_char_params, 
                                  &p_ftms->sup_power_range_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add supported inclination range characteristic    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_SUPPORTED_INCLINATION_RANGE;
    add_char_params.max_len         = 3*sizeof(uint16_t);
    add_char_params.init_len        = 6; //uint16_encode(p_ftms_init->feature, &init_value_encoded[0]);
    
    sint16_encode(BLE_FTMS_INCLINE_MINIMUM, &init_value_encoded[0]);
    sint16_encode(BLE_FTMS_INCLINE_MAXIMUM, &init_value_encoded[2]);
    uint16_encode(BLE_FTMS_INCLINE_INCREMENT, &init_value_encoded[4]);

    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = p_ftms_init->ftms_sup_inclination_range_rd_sec;

    err_code = characteristic_add(p_ftms->service_handle, 
                                  &add_char_params, 
                                  &p_ftms->sup_inclination_range_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add supported resistance range characteristic    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_SUPPORTED_RESISTANCE_RANGE;
    add_char_params.max_len         = 3*sizeof(uint16_t);
    add_char_params.init_len        = 6; // uint16_encode(p_ftms_init->feature, &init_value_encoded[0]);
    
    sint16_encode(BLE_FTMS_RESISTANCE_MINIMUM, &init_value_encoded[0]); // Minimum resistance (unit less)
    sint16_encode(BLE_FTMS_RESISTANCE_MAXIMUM, &init_value_encoded[2]); // Maximum resistance (unit less)
    uint16_encode(BLE_FTMS_RESISTANCE_INCREMENT, &init_value_encoded[4]); // Minimum Increment (unitless)
    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = p_ftms_init->ftms_sup_resistance_range_rd_sec;

    err_code = characteristic_add(p_ftms->service_handle, 
                                  &add_char_params, 
                                  &p_ftms->sup_resistance_range_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add power control point characteristic
    ftms_ctrlpt_init.error_handler                 = p_ftms_init->error_handler;        

    ftms_ctrlpt_init.supported_functions           = p_ftms_init->ctrplt_supported_functions;
    ftms_ctrlpt_init.evt_handler                   = p_ftms_init->ctrlpt_evt_handler;
    // ftms_ctrlpt_init.trainer_status                = p_ftms_init->trainer_status;
    ftms_ctrlpt_init.trainer_started               = &p_ftms->trainer_started;
    ftms_ctrlpt_init.control_acquired              = &p_ftms->control_acquired;
    ftms_ctrlpt_init.ftms_ctrlpt_wr_sec            = p_ftms_init->ftms_ctrlpt_wr_sec;
    ftms_ctrlpt_init.ftms_ctrlpt_cccd_wr_sec       = p_ftms_init->ftms_ctrlpt_cccd_wr_sec;    
    ftms_ctrlpt_init.service_handle                = p_ftms->service_handle;

    // NRF_LOG_INFO("ble_ftms::ble_ftms_init() successfully finished");
    return ble_ftms_ctrlpt_init(&p_ftms->ctrl_pt, &ftms_ctrlpt_init);
    
    // return err_code;
}


uint32_t ble_ftms_indoor_bike_data_send(ble_ftms_t * p_ftms, ble_ftms_indoor_bike_data_t * p_indoor_bike_data)
{
    // TODO: Check maximum length!
    if (p_ftms == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_ftms->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_indoor_bike_data[MAX_FTMM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        memset(encoded_indoor_bike_data, 0, sizeof(ble_ftms_indoor_bike_data_t));
        len     = ftms_indoor_bike_data_encode(p_ftms, p_indoor_bike_data, encoded_indoor_bike_data);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ftms->indoor_bike_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_indoor_bike_data;

        // Log
        //char i_cad_s[16];
        //sprintf(i_cad_s, "%f" , p_indoor_bike_data->instantaneous_cadence);
        // Not needed! The inst cadence is NOT a fraction as in CSCS!

        char buf[128];
        hex2str(buf, sizeof(buf), encoded_indoor_bike_data, hvx_len);

        char mbuf[128];
        sprintf(mbuf, "(Inst speed: %d, Inst cadence: %d, Avg Cadence: %d, Resistance: %d, Inst Power: %d, Avg: Power: %d)", 
                p_indoor_bike_data->instantaneous_speed, 
                p_indoor_bike_data->instantaneous_cadence,
                p_indoor_bike_data->average_cadence,
                resistance_level * 10,
                p_indoor_bike_data->instantaneous_power,
                p_indoor_bike_data->average_power);
        // NRF_LOG_INFO("ftms_indoor: %s %s", buf, mbuf);
        
        
        err_code = sd_ble_gatts_hvx(p_ftms->conn_handle, &hvx_params);
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


uint32_t ble_ftms_training_status_send(ble_ftms_t * p_ftms, ble_ftms_training_status_t * p_training_status)
{
    // TODO: Check maximum length!
    if (p_ftms == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_ftms->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_training_status[MAX_FTMM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        memset(encoded_training_status, 0, sizeof(encoded_training_status));
        len     = ftms_training_status_encode(p_ftms, p_training_status, encoded_training_status);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ftms->training_status_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_training_status;

#if LOGHERE
        // Log
        char buf[128];
        hex2str(buf, sizeof(buf), encoded_training_status, hvx_len);
        NRF_LOG_INFO("ftms_training_stats: %s", buf);                
#endif        
        err_code = sd_ble_gatts_hvx(p_ftms->conn_handle, &hvx_params);
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


uint32_t ble_ftms_machine_status_send(ble_ftms_t * p_ftms, ble_ftms_machine_status_t * p_machine_status)
{
    if (p_ftms == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_ftms->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_machine_status[MAX_FTMM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        memset(encoded_machine_status, 0, sizeof(encoded_machine_status));
        len     = ftms_machine_status_encode(p_ftms, p_machine_status, encoded_machine_status);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ftms->machine_status_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_machine_status;

        
#if LOGHERE        
        // Log
        char buf[128];
        hex2str(buf, sizeof(buf), encoded_machine_status, hvx_len);
        NRF_LOG_INFO("ftms_machine_stats: %s", buf);                
#endif        
        
        err_code = sd_ble_gatts_hvx(p_ftms->conn_handle, &hvx_params);
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


#endif // NRF_MODULE_ENABLED(BLE_FTMS)