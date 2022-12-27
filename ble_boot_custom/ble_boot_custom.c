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
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_BOOT)
#include "ble.h"
#include "ble_boot_custom.h"
#include "ble_srv_common.h"

#define NRF_LOG_MODULE_NAME ble_boot
#if BLE_BOOT_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_BOOT_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_BOOT_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_BOOT_CONFIG_DEBUG_COLOR
#else // BLE_BOOT_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_BOOT_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

// The 669aa501-0c08-969e-e211-86ad5062675f is unkown...
// It has one characteristic: 669aac01-0c08-969e-e211-86ad5062675f

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_boot     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_boot_t * p_boot, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_boot_evt_t              evt;
    ble_gatts_value_t          gatts_val;
    uint8_t                    cccd_value[2];
    ble_boot_client_context_t * p_client = NULL;

    err_code = blcm_link_ctx_get(p_boot->p_link_ctx_storage,
                                 p_ble_evt->evt.gap_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gap_evt.conn_handle);
    }

    /* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_boot->tx_handles.cccd_handle,
                                      &gatts_val);

    NRF_LOG_INFO("sd_ble_gatts_value_get Err_code = %d", err_code);
    if ((err_code == NRF_SUCCESS)     &&
        (p_boot->data_handler != NULL) &&
        ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        if (p_client != NULL)
        {
            p_client->is_notification_enabled = true;
        }
        NRF_LOG_INFO("Past isnotify check");
        memset(&evt, 0, sizeof(ble_boot_evt_t));
        evt.type        = BLE_BOOT_EVT_COMM_STARTED;
        evt.p_boot       = p_boot;
        evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_boot->data_handler(&evt);
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_boot     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_boot_t * p_boot, ble_evt_t const * p_ble_evt)
{
    ret_code_t                    err_code;
    ble_boot_evt_t                 evt;
    ble_boot_client_context_t    * p_client;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    err_code = blcm_link_ctx_get(p_boot->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }

    memset(&evt, 0, sizeof(ble_boot_evt_t));
    evt.p_boot       = p_boot;
    evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    evt.p_link_ctx  = p_client;

    if ((p_evt_write->handle == p_boot->tx_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        NRF_LOG_INFO("BOOT Handle = CCCD_handle")
        if (p_client != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                p_client->is_notification_enabled = true;
                evt.type                          = BLE_BOOT_EVT_COMM_STARTED;
                NRF_LOG_INFO("BOOT enabled notification")
            }
            else
            {
                p_client->is_notification_enabled = false;
                evt.type                          = BLE_BOOT_EVT_COMM_STOPPED;
                NRF_LOG_INFO("BOOT disabled notification")
            }

            if (p_boot->data_handler != NULL)
            {
                // NRF_LOG_INFO("Calling data_handler()")
                p_boot->data_handler(&evt);
            }

        }
    }
    else if ((p_evt_write->handle == p_boot->rx_handles.value_handle) &&
             (p_boot->data_handler != NULL))
    {
        NRF_LOG_INFO("Calling data_handler() directly")
        evt.type                  = BLE_BOOT_EVT_RX_DATA;
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;

        p_boot->data_handler(&evt);
    }
    else
    {
        //uint16_t uuid = p_evt_write->uuid.uuid;
        // Do Nothing. This event is not relevant for this service.
        //NRF_LOG_INFO("UUID: %x", uuid);
        //NRF_LOG_HEXDUMP_INFO(p_evt_write->data, p_evt_write->len);
        
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event from the SoftDevice.
 *
 * @param[in] p_boot     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_hvx_tx_complete(ble_boot_t * p_boot, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_boot_evt_t              evt;
    ble_boot_client_context_t * p_client;

    err_code = blcm_link_ctx_get(p_boot->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
        return;
    }

    if ((p_client->is_notification_enabled) && (p_boot->data_handler != NULL))
    {
        memset(&evt, 0, sizeof(ble_boot_evt_t));
        evt.type        = BLE_BOOT_EVT_TX_RDY;
        evt.p_boot       = p_boot;
        evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_boot->data_handler(&evt);
    }
}


void ble_boot_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_boot_t * p_boot = (ble_boot_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:            
            on_connect(p_boot, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:            
            on_write(p_boot, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:            
            on_hvx_tx_complete(p_boot, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_boot_init(ble_boot_t * p_boot, ble_boot_init_t const * p_boot_init)
{
    ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         boot_base_uuid = BOOT_BASE_UUID;
    ble_add_char_params_t add_char_params;

    VERIFY_PARAM_NOT_NULL(p_boot);
    VERIFY_PARAM_NOT_NULL(p_boot_init);

    // Initialize the service structure.
    p_boot->data_handler = p_boot_init->data_handler;

    /**@snippet [Adding proprietary Service to the SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&boot_base_uuid, &p_boot->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_boot->uuid_type;
    ble_uuid.uuid = BLE_UUID_BOOT_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_boot->service_handle);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the ?? Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = BLE_UUID_BOOT_CHARACTERISTIC;
    add_char_params.uuid_type                = p_boot->uuid_type;
    add_char_params.max_len                  = BLE_BOOT_MAX_DATA_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    return characteristic_add(p_boot->service_handle, &add_char_params, &p_boot->rx_handles);
        
}


uint32_t ble_boot_data_send(ble_boot_t * p_boot,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    ble_boot_client_context_t * p_client;

    VERIFY_PARAM_NOT_NULL(p_boot);
    NRF_LOG_INFO("ble_boot_data_send()");

    err_code = blcm_link_ctx_get(p_boot->p_link_ctx_storage, conn_handle, (void *) &p_client);
    VERIFY_SUCCESS(err_code);

    if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!p_client->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (*p_length > BLE_BOOT_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_boot->tx_handles.value_handle;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = p_length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}


#endif // NRF_MODULE_ENABLED(BLE_BOOT)
