/**
 * Copyright (c) 2012 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/* Attention!
 * To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile
 * qualification listings, this section of source code must not be modified.
 */

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_CPS)
#include "ble_cps.h"
#include <string.h>
#include "ble_srv_common.h"
#include "ble_conn_state.h"

#define NRF_LOG_MODULE_NAME ble_cps
#if BLE_CPS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_CPS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_CPS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_CPS_CONFIG_DEBUG_COLOR
#else // BLE_CPS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_CPS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include <string.h>
#include "nordic_common.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "nrf_log.h"
#include "calculations.h"
#include "ble_cscs_c.h"
#include "math.h"
#include "helper.h"
#include "assert.h"

#define OPCODE_LENGTH 1                                                             /**< Length of opcode inside Cycling Power Measurement packet. */
#define HANDLE_LENGTH 2                                                             /**< Length of handle inside Cycling Power Measurement packet. */
#define MAX_CPM_LEN  (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)    /**< Maximum size of a transmitted Cycling Power Measurement. */

// Cycling Power Measurement flag bits
#define CP_MEAS_FLAG_MASK_WHEEL_REV_DATA_PRESENT (0x01 << 4)  /**< Wheel revolution data present flag bit. */
#define CP_MEAS_FLAG_MASK_CRANK_REV_DATA_PRESENT (0x01 << 5)  /**< Crank revolution data present flag bit. */

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/*
#define SPEEDS_SIZE 5
double speeds[SPEEDS_SIZE] = {0.0, 0.0, 0.0, 0.0, 0.0 };
int speed_ptr = 0;
int speed_size = 0;
*/
#define LOGHERE 1



// ble_cps_calculation_helper_t cps_calculation_helper;

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cps       Cycling Power Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cps_t * p_cps, ble_evt_t const * p_ble_evt)
{
    p_cps->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cps       Cycling Power Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cps_t * p_cps, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cps->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the CPS Measurement characteristic.
 *
 * @param[in]   p_cps         Cycling Power Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_meas_cccd_write(ble_cps_t * p_cps, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_cps->evt_handler != NULL)
        {
            ble_cps_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_CPS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_CPS_EVT_NOTIFICATION_DISABLED;
            }

            p_cps->evt_handler(p_cps, &evt);
        }
    }
}

void ble_cps_on_cscs_evt(ble_cps_t * m_cps, ble_cscs_c_evt_t * p_cscs_c_evt)
{
    ble_cps_meas_t     cps_measurment;
    ret_code_t         err_code;
        
    cps_measurment.cumulative_wheel_revs      = getCumulativeWheelRevs(); 
    cps_measurment.last_wheel_event_time      = getLastWheelEventTime(); 
    cps_measurment.cumulative_crank_revs      = getCumulativeCrankRevs(); 
    cps_measurment.last_crank_event_time      = getLastCrankEventTime(); 
    cps_measurment.is_wheel_rev_data_present  = p_cscs_c_evt->params.csc.is_wheel_rev_data_present;
    cps_measurment.is_crank_rev_data_present  = p_cscs_c_evt->params.csc.is_crank_rev_data_present;
    cps_measurment.instantaneous_power        = getInstantaneousPower();    
    
    err_code = ble_cps_measurement_send(m_cps, &cps_measurment);
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
 * @param[in]   p_cps       Cycling Power Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_cps_t * p_cps, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_cps->meas_handles.cccd_handle)
    {
        on_meas_cccd_write(p_cps, p_evt_write);
    }
}


void ble_cps_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cps_t * p_cps = (ble_cps_t *)p_context;

    if (p_cps == NULL || p_ble_evt == NULL)
    {
        return;
    }


    ble_cps_ctrlpt_on_ble_evt(&(p_cps->ctrl_pt), p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cps, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cps, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_cps, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for encoding a CPS Measurement.
 *
 * @param[in]   p_cps               Cycling Power Service structure.
 * @param[in]   p_cp_measurement    Measurement to be encoded.
 * @param[out]  p_encoded_buffer    Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t cp_measurement_encode(ble_cps_t      * p_cps,
                                     ble_cps_meas_t * p_cp_measurement,
                                     uint8_t        * p_encoded_buffer)
{
    uint16_t flags = 0;
    uint8_t len   = 2; // Minimum length for flags

    if (p_cps->feature & BLE_CPS_FEATURE_WHEEL_REV_BIT)
    {
        if (p_cp_measurement->is_wheel_rev_data_present)
        {
            flags |= CP_MEAS_FLAG_MASK_WHEEL_REV_DATA_PRESENT;
        }
    }

    if (p_cps->feature & BLE_CPS_FEATURE_CRANK_REV_BIT)
    {
        if (p_cp_measurement->is_crank_rev_data_present)
        {
            flags |= CP_MEAS_FLAG_MASK_CRANK_REV_DATA_PRESENT;
        }
    }

    p_encoded_buffer[0] = LSB_16(flags);
    p_encoded_buffer[1] = MSB_16(flags);
    
    len += uint16_encode(p_cp_measurement->instantaneous_power, &p_encoded_buffer[len]); // Should be signed int but nvm

    // Cumulative Wheel Revolutions and Last Wheel Event Time Fields
    if (p_cps->feature & BLE_CPS_FEATURE_WHEEL_REV_BIT)
    {
        if (p_cp_measurement->is_wheel_rev_data_present)
        {
            // flags |= CP_MEAS_FLAG_MASK_WHEEL_REV_DATA_PRESENT;
            len += uint32_encode(p_cp_measurement->cumulative_wheel_revs, &p_encoded_buffer[len]);
            len += uint16_encode(p_cp_measurement->last_wheel_event_time*2, &p_encoded_buffer[len]); // The last wheel eventtime in cps is 1/2048, in cscs it's 1/1024! -> multiply with 2
        }        
    }
    
    // Cumulative Crank Revolutions and Last Crank Event Time Fields
    if (p_cps->feature & BLE_CPS_FEATURE_CRANK_REV_BIT)
    {
        if (p_cp_measurement->is_crank_rev_data_present)
        {
            // flags |= CP_MEAS_FLAG_MASK_CRANK_REV_DATA_PRESENT;
            len += uint16_encode(p_cp_measurement->cumulative_crank_revs, &p_encoded_buffer[len]);
            len += uint16_encode(p_cp_measurement->last_crank_event_time, &p_encoded_buffer[len]);
        }
    }
    

    // Flags Field    
    return len;
}


uint32_t ble_cps_init(ble_cps_t * p_cps, ble_cps_init_t const * p_cps_init)
{
    if (p_cps == NULL || p_cps_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t              err_code;
    uint8_t               init_value_encoded[MAX_CPM_LEN];
    ble_cps_meas_t        initial_scm = {0};
    ble_add_char_params_t add_char_params;
    ble_uuid_t            ble_uuid;
    ble_cps_ctrlpt_init_t  cps_ctrlpt_init;

    // Initialize service structure
    p_cps->evt_handler = p_cps_init->evt_handler;
    p_cps->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_cps->feature     = p_cps_init->feature;

    memset(init_value_encoded, 0, sizeof(init_value_encoded));
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CYCLING_POWER);

    // memset(&cps_calculation_helper, 0, sizeof(ble_cps_calculation_helper_t));

    // Set initial difficulty level
    p_cps->resistance_level = p_cps_init->resistance_level; // This one is not really needed -> Refactor
    // cps_calculation_helper.cscs_c.resistance_level = p_cps_init->resistance_level;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_cps->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add cycling power measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_CYCLING_POWER_MEASUREMENT;
    add_char_params.max_len           = MAX_CPM_LEN;
    add_char_params.is_var_len        = true;
    add_char_params.init_len          = cp_measurement_encode(p_cps, &initial_scm, init_value_encoded);
    add_char_params.p_init_value      = init_value_encoded;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_cps_init->cps_meas_cccd_wr_sec;

    err_code = characteristic_add(p_cps->service_handle, &add_char_params, &p_cps->meas_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add cycling power feature characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_CYCLING_POWER_FEATURE;
    add_char_params.max_len         = sizeof(uint16_t);
    add_char_params.init_len        = uint16_encode(p_cps_init->feature, &init_value_encoded[0]);
    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = p_cps_init->cps_feature_rd_sec;

    err_code = characteristic_add(p_cps->service_handle, 
                                  &add_char_params, 
                                  &p_cps->feature_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Sensor Location characteristic (optional)
    if (p_cps_init->sensor_location != NULL)
    {
        memset(&add_char_params, 0, sizeof(add_char_params));
        add_char_params.uuid            = BLE_UUID_SENSOR_LOCATION_CHAR;
        add_char_params.max_len         = sizeof(uint8_t);
        add_char_params.init_len        = sizeof(uint8_t);
        add_char_params.p_init_value    = (uint8_t *)p_cps_init->sensor_location;
        add_char_params.char_props.read = 1;
        add_char_params.read_access     = p_cps_init->cps_location_rd_sec;

        err_code = characteristic_add(p_cps->service_handle, &add_char_params, &p_cps->sensor_loc_handles);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    // Add power control point characteristic
    cps_ctrlpt_init.error_handler                 = p_cps_init->error_handler;
    cps_ctrlpt_init.size_list_supported_locations = p_cps_init->size_list_supported_locations;
    cps_ctrlpt_init.supported_functions           = p_cps_init->ctrplt_supported_functions;
    cps_ctrlpt_init.evt_handler                   = p_cps_init->ctrlpt_evt_handler;
    cps_ctrlpt_init.list_supported_locations      = p_cps_init->list_supported_locations;
    cps_ctrlpt_init.cps_ctrlpt_wr_sec              = p_cps_init->cps_ctrlpt_wr_sec;
    cps_ctrlpt_init.cps_ctrlpt_cccd_wr_sec         = p_cps_init->cps_ctrlpt_cccd_wr_sec;
    cps_ctrlpt_init.sensor_location_handle        = p_cps->sensor_loc_handles.value_handle;
    cps_ctrlpt_init.service_handle                = p_cps->service_handle;

    return ble_cps_ctrlpt_init(&p_cps->ctrl_pt, &cps_ctrlpt_init);
    
    // return err_code;
}


uint32_t ble_cps_measurement_send(ble_cps_t * p_cps, ble_cps_meas_t * p_measurement)
{
    if (p_cps == NULL || p_measurement == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code;

    // Send value if connected and notifying
    if (p_cps->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_cp_meas[MAX_CPM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        memset(encoded_cp_meas, 0, sizeof(encoded_cp_meas));
        len     = cp_measurement_encode(p_cps, p_measurement, encoded_cp_meas);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_cps->meas_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_cp_meas;
        
#if LOGHERE2
        char i_cad_s[16];
        sprintf(i_cad_s, "%f" , getInstantaneousCadence());

        char buf[128];
        hex2str(buf, sizeof(buf), encoded_cp_meas, hvx_len);
        NRF_LOG_INFO("cpm: %s (Incline %d, Watts: %d, Inst Cadence: %s, Avg Cadence: %d, Avg speed: %d)", 
                buf, 
                *cps_calculation_helper.cscs_c.incline_level, 
                getInstantaneousPower(),
                i_cad_s,
                getAverageCadence(),                 
                getAverageSpeed()
                );
#endif
        err_code = sd_ble_gatts_hvx(p_cps->conn_handle, &hvx_params);
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

#endif // NRF_MODULE_ENABLED(BLE_CPS)