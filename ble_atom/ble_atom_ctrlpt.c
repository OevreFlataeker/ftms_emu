

#include "ble_atom_ctrlpt.h"
#include <string.h>
#include "nordic_common.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ble_atom.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "helper.h"
#define STERZO_CTRLPT_NACK_PROC_ALREADY_IN_PROGRESS   (BLE_GATT_STATUS_ATTERR_APP_BEGIN + 0)
#define STERZO_CTRLPT_NACK_CCCD_IMPROPERLY_CONFIGURED (BLE_GATT_STATUS_ATTERR_APP_BEGIN + 1)

#define LOGHERE 1

uint32_t ble_atom_ctrlpt_init(ble_atom_ctrlpt_t            * p_atom_ctrlpt,
                            const ble_atom_ctrlpt_init_t * p_atom_ctrlpt_init)
{
    NRF_LOG_ERRROR("We should not be here!");
    if (p_atom_ctrlpt == NULL || p_atom_ctrlpt_init == NULL)
    {
        return NRF_ERROR_NULL;
    }
    NRF_LOG_INFO("ble_atom_ctrlpt::ble_atom_ctrlpt_init()");
    ble_add_char_params_t add_char_params;

    p_atom_ctrlpt->conn_handle      = BLE_CONN_HANDLE_INVALID;
    p_atom_ctrlpt->procedure_status = BLE_ATOMPT_NO_PROC_IN_PROGRESS;
    p_atom_ctrlpt->authenticated          = &(p_atom_ctrlpt_init->authenticated);
    p_atom_ctrlpt->service_handle         = p_atom_ctrlpt_init->service_handle;  
    p_atom_ctrlpt->evt_handler            = p_atom_ctrlpt_init->evt_handler;  // From main (ftms_ctrlpt_event_handler)
    p_atom_ctrlpt->supported_functions    = p_atom_ctrlpt_init->supported_functions; // From main
    p_atom_ctrlpt->error_handler          = p_atom_ctrlpt_init->error_handler;  // From main -> NULL!

    // Add sterzo control point characteristic
    //   pRx = pService->createCharacteristic(STEERING_RX_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    //pRx->addDescriptor(new BLE2902());
   // pRx->setCallbacks(new MyCharacteristicCallbacks());


    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = BLE_UUID_STERZO_CTRLPT_CHAR; //BLE_UUID_FTMS_CTRLPT_CHAR;
    add_char_params.uuid_type           = p_atom_ctrlpt_init->service_uuid_type;
    add_char_params.max_len             = 4;
    add_char_params.init_len             = sizeof(uint8_t);
    add_char_params.is_var_len          = true;
    add_char_params.char_props.indicate = 1;
    add_char_params.char_props.write    = 1;
    
    add_char_params.cccd_write_access   = p_atom_ctrlpt_init->sterzo_ctrlpt_cccd_wr_sec;
    add_char_params.write_access        = p_atom_ctrlpt_init->sterzo_ctrlpt_wr_sec;
    add_char_params.is_defered_write    = true;


    return characteristic_add(p_atom_ctrlpt->service_handle,
                              &add_char_params,
                              &p_atom_ctrlpt->sterzo_ctrlpt_handles);
}


/**@brief Decode an incoming control point write.
 *
 * @param[in]    rcvd_val       received write value
 * @param[in]    len            value length
 * @param[out]   decoded_ctrlpt decoded control point structure
 */
static uint32_t sterzo_ctrlpt_decode(uint8_t const       * p_rcvd_val,
                                 uint8_t               len,
                                 ble_atom_ctrlpt_val_t * p_write_val)
{
    int pos = 0;
    uint16_t value16;
    int16_t value_s16;
    uint8_t value8;
    int8_t value_s8;
    NRF_LOG_INFO("ble_atom_ctrlpt::sterzo_ctrlpt_decode");
    if (len < BLE_ATOM_CTRLPT_MIN_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    p_write_val->opcode = (ble_atompt_operator_t) p_rcvd_val[pos++];

    switch (p_write_val->opcode)
    {        
        case BLE_ATOMPT_AUTHENTICATE:
            NRF_LOG_INFO("ble_atom_ctrlpt::sterzo_ctrlpr_decode() - Decoding params for BLE_ATOMPT_AUTHENTICATE");
            // No params
            break;

        default:
            NRF_LOG_INFO("ble_atom_ctrlpt::sterzo_ctrlpr_decode() - NRF_ERROR_INVALID_PARAM");
            return NRF_ERROR_INVALID_PARAM;
    }
    return NRF_SUCCESS;
}


/**@brief encode a control point response indication.
 *
 * @param[in]   p_sc_ctrlpt      SC Ctrlpt structure.
 * @param[in]   p_ctrlpt_rsp  structure containing response data to be encoded
 * @param[out]  p_data        pointer where data needs to be written
 * @return                    size of encoded data
 */
static int ctrlpt_rsp_encode(ble_atom_ctrlpt_t     * p_atom_ctrlpt,
                             ble_atom_ctrlpt_rsp_t * p_ctrlpt_rsp,
                             uint8_t             * p_data)
{
    int len = 0;

    p_data[len++] = BLE_ATOMPT_RESPONSE_CODE;
    p_data[len++] = p_ctrlpt_rsp->opcode;
    p_data[len++] = p_ctrlpt_rsp->status;
#if LOGHERE
    NRF_LOG_ERROR("Needs impl.");
    NRF_LOG_INFO("ble_atom_ctrlpt::ctrlpt_rsp_encode: %02x %02x %02x", BLE_ATOMPT_RESPONSE_CODE, p_ctrlpt_rsp->opcode, p_ctrlpt_rsp->status); 
#endif
    if (p_ctrlpt_rsp->status == BLE_ATOMPT_SUCCESS)
    {
        switch (p_ctrlpt_rsp->opcode)
        {           
            default:
                // No implementation needed.
                break;
        }
    }
    return len;
}


/**@brief check if the cccd is configured
 *
 * @param[in]   p_sc_ctrlpt      SC Ctrlpt structure.
 * @return  true if the sc_control point's cccd is correctly configured, false otherwise.
 */
static bool is_cccd_configured(ble_atom_ctrlpt_t * p_atom_ctrlpt)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    bool     is_sterzocp_indic_enabled = false;
    ble_gatts_value_t gatts_value;

    NRF_LOG_INFO("ble_atom_ctrlpt::is_cccd_configured()");
    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = BLE_CCCD_VALUE_LEN;
    gatts_value.offset  = 0;
    gatts_value.p_value = cccd_value_buf;

    err_code = sd_ble_gatts_value_get(p_atom_ctrlpt->conn_handle,
                                      p_atom_ctrlpt->sterzo_ctrlpt_handles.cccd_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application
        if (p_atom_ctrlpt->error_handler != NULL)
        {
            p_atom_ctrlpt->error_handler(err_code);
        }
    }

    is_sterzocp_indic_enabled = ble_srv_is_indication_enabled(cccd_value_buf);
    if (!is_sterzocp_indic_enabled)
    {
        NRF_LOG_INFO("Indication is NOT enabled");
    }
    return is_sterzocp_indic_enabled;
}


/**@brief sends a control point indication.
 *
 * @param[in]   p_sc_ctrlpt      SC Ctrlpt structure.
 */
static void sterzo_ctrlpt_resp_send(ble_atom_ctrlpt_t * p_atom_ctrlpt)
{
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    uint32_t               err_code;
        
    if (p_atom_ctrlpt->procedure_status == BLE_ATOMPT_INDICATION_PENDING)
    {
        NRF_LOG_INFO("ble_atom_ctrlpt::sterzo_ctrlpt_resp_send() - BLE_ATOMPT_INDICATION_PENDING");
        hvx_len = p_atom_ctrlpt->response.len;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_atom_ctrlpt->sterzo_ctrlpt_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_INDICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = p_atom_ctrlpt->response.encoded_ctrl_rsp;

        err_code = sd_ble_gatts_hvx(p_atom_ctrlpt->conn_handle, &hvx_params);

        // Error handling
        if ((err_code == NRF_SUCCESS) && (hvx_len != p_atom_ctrlpt->response.len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

        switch (err_code)
        {
            case NRF_SUCCESS:
                NRF_LOG_INFO("ble_atom_ctrlpt::sterzo_ctrlpt_resp_send() - NRF_SUCCESS");
                p_atom_ctrlpt->procedure_status = BLE_ATOMPT_IND_CONFIRM_PENDING;
                // Wait for HVC event
                break;

            case NRF_ERROR_RESOURCES:
                // NRF_LOG_INFO("ble_ftms_ctrlpt::ftms_ctrlpt_resp_send() - NRF_ERROR_RESOURCES");
                // Wait for TX_COMPLETE event to retry transmission.
                p_atom_ctrlpt->procedure_status = BLE_ATOMPT_INDICATION_PENDING;
                break;

            default:
                // Report error to application.
                // NRF_LOG_INFO("ble_ftms_ctrlpt::ftms_ctrlpt_resp_send() - BLE_FTMSPT_NO_PROC_IN_PROGRESS");
                p_atom_ctrlpt->procedure_status = BLE_ATOMPT_NO_PROC_IN_PROGRESS;
                if (p_atom_ctrlpt->error_handler != NULL)
                {
                    p_atom_ctrlpt->error_handler(err_code);
                }
                break;
        }
    }
}


/**@brief Handle a write event to the Speed and Cadence Control Point.
 *
 * @param[in]   p_sc_ctrlpt      SC Ctrlpt structure.
 * @param[in]   p_evt_write      WRITE event to be handled.
 */
static void on_ctrlpt_write(ble_atom_ctrlpt_t             * p_atom_ctrlpt,
                            ble_gatts_evt_write_t const * p_evt_write)
{
    //ble_atom_ctrlpt_val_t                   rcvd_ctrlpt =
    //{ BLE_ATOMPT_RESPONSE_CODE , 0};

    ble_atom_ctrlpt_rsp_t                   rsp;
    uint32_t                              err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;
    ble_atom_ctrlpt_evt_t                   evt;

    NRF_LOG_HEXDUMP_INFO(p_evt_write->data, p_evt_write->len);

    if (p_evt_write->data[0] == 0x03 && p_evt_write->data[1] == 0x10)
        {
            // issue the challenge of 0x0310yyyy on 0x0032
            NRF_LOG_INFO("Received challenge request! (0x0310)");
            ble_atom_ctrlpt_evt_t evt;
            evt.evt_type = BLE_ATOM_CTRLPT_EVT_AUTHENTICATE;
            p_atom_ctrlpt->service_handle.procedure_status == BLE_ATOM_INDICATION_PENDING;
            p_atom_ctrlpt->evt_handler(p_atom_ctrlpt, &evt);            
            
            //ble_cus_tx_value_update(p_cus, challengeRequest, 4);

            // APP_ERROR_CHECK(app_sched_event_put(NULL, 0,
            // send_challenge_response));
        }
    
    if (p_evt_write->data[0] == 0x03 && p_evt_write->data[1] == 0x11)
        {
            // emit 0x0311ffff on 0x0032
            NRF_LOG_INFO("Received start sending sterring data request (0x0311)");
            // ble_cus_tx_value_update(p_cus, someOtherThing, 4);
            
            // tell app to start firing off steering data

            ble_atom_ctrlpt_evt_t evt;
            evt.evt_type = BLE_ATOM_CTRLPT_EMIT_RESPONSE;
            p_atom_ctrlpt->service_handle.procedure_status == BLE_ATOM_INDICATION_PENDING;
            p_atom_ctrlpt->evt_handler(p_atom_ctrlpt, &evt);                            
        }

    /*
    NRF_LOG_INFO("ble_atom_ctrlpt::on_ctrlpt_write()");
    auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
    auth_reply.params.write.offset      = 0;
    auth_reply.params.write.len         = 0;
    auth_reply.params.write.p_data      = NULL;
    auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
    auth_reply.params.write.update      = 1;

    /* We do not seem to need it
    if (is_cccd_configured(p_atom_ctrlpt))
    {
        if (p_atom_ctrlpt->procedure_status == BLE_ATOMPT_NO_PROC_IN_PROGRESS)
        {
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        }
        else
        {
            auth_reply.params.write.gatt_status = STERZO_CTRLPT_NACK_PROC_ALREADY_IN_PROGRESS;
        }
    }
    else
    {
        auth_reply.params.write.gatt_status = STERZO_CTRLPT_NACK_CCCD_IMPROPERLY_CONFIGURED;
    }
    */

    /*
    err_code = sd_ble_gatts_rw_authorize_reply(p_atom_ctrlpt->conn_handle, &auth_reply);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application.
        if (p_atom_ctrlpt->error_handler != NULL)
        {
            p_atom_ctrlpt->error_handler(err_code);
        }
    }

    if (auth_reply.params.write.gatt_status != BLE_GATT_STATUS_SUCCESS)
    {
        return;
    }

    p_atom_ctrlpt->procedure_status = BLE_ATOMPT_INDICATION_PENDING;
    rsp.status                    = BLE_ATOMPT_OP_CODE_NOT_SUPPORTED;

    err_code = sterzo_ctrlpt_decode(p_evt_write->data, p_evt_write->len, &rcvd_ctrlpt);
    if (err_code != NRF_SUCCESS)
    {
        rsp.opcode = rcvd_ctrlpt.opcode;
        rsp.status = BLE_ATOMPT_OP_CODE_NOT_SUPPORTED;
    }
    else
    {
        rsp.opcode = rcvd_ctrlpt.opcode;
        NRF_LOG_INFO("Received opcode = 0x%x", rsp.opcode);
        switch (rcvd_ctrlpt.opcode)
        {
            case BLE_ATOMPT_RESPONSE_CODE:
                NRF_LOG_INFO("Hm, BLE_ATOMPT_RESPONSE_CODE in a request? Strange...");
                    rsp.status = BLE_ATOMPT_OP_CODE_NOT_SUPPORTED;
              
                break;
            case BLE_ATOMPT_AUTHENTICATE:
                if (1)
                    {
                        NRF_LOG_INFO("Received authenticate");
                        // Check if control has been acquired - if YES (in contrary to all the other cases, we return an error)
                        if (*(p_atom_ctrlpt->authenticated))
                        {
                            // Fail early - we don' have control!
                            rsp.status = BLE_ATOMPT_ALREADY_AUTHENTICATED;
                            break;
                        }  
                        rsp.status = BLE_ATOMPT_SUCCESS;
                        evt.evt_type                = BLE_ATOM_CTRLPT_EVT_AUTHENTICATE;
                        if (p_atom_ctrlpt->evt_handler != NULL)
                        {
                            // Do the action (calls main)
                            rsp.status = p_atom_ctrlpt->evt_handler(p_atom_ctrlpt, &evt);
                        }
                        NRF_LOG_INFO("We received %d as rsp.status", rsp.status);
                        // Does not seem to be needed
                        /*
                        if (rsp.status == BLE_FTMSPT_SUCCESS)
                        {
                            err_code = sd_ble_gatts_value_set(p_ftms_ctrlpt->conn_handle,
                                                              p_ftms_ctrlpt->trainer_status_handle, // Needs to be added
                                                              &gatts_value);
                            if (err_code != NRF_SUCCESS)
                            {
                                // Report error to application
                                if (p_ftms_ctrlpt->error_handler != NULL)
                                {
                                    p_ftms_ctrlpt->error_handler(err_code);
                                }
                                rsp.status = BLE_FTMSPT_OPERATION_FAILED;
                            }
                        } // No else?
                   
                    }
                else
                {
                    rsp.status = BLE_ATOMPT_OP_CODE_NOT_SUPPORTED;
                }
                break;
               
            
            
            default:
                NRF_LOG_INFO("Received an unsupported opcode in ble_atom_ctrlpt::on_ctrlpt_write(): 0x%02x", rcvd_ctrlpt.opcode);
                rsp.status = BLE_ATOMPT_OP_CODE_NOT_SUPPORTED;
                break;
        }

    }

    p_atom_ctrlpt->response.len = ctrlpt_rsp_encode(p_atom_ctrlpt, &rsp,
                                                  p_atom_ctrlpt->response.encoded_ctrl_rsp);

         */

    if (p_atom_ctrlpt->procedure_status == BLE_ATOMPT_INDICATION_PENDING)
    {
        sterzo_ctrlpt_resp_send(p_atom_ctrlpt);
    }
}


/**@brief Authorize WRITE request event handler.
 *
 * @details Handles WRITE events from the BLE stack.
 *
 * @param[in]   p_sc_ctrlpt  SC Ctrlpt structure.
 * @param[in]   p_gatts_evt  GATTS Event received from the BLE stack.
 *
 */
static void on_rw_authorize_request(ble_atom_ctrlpt_t       * p_atom_ctrlpt,
                                    ble_gatts_evt_t const * p_gatts_evt)
{
    ble_gatts_evt_rw_authorize_request_t const * p_auth_req =
        &p_gatts_evt->params.authorize_request;

    //NRF_LOG_INFO("ble_atom_ctrlpt::on_rw_authorize_request()");
    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        //NRF_LOG_INFO("ble_atom_ctrlpt::on_rw_authorize_request(), type == BLE_GATTS_AUTHORIZE_TYPE_WRITE");
        if (   (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_PREP_WRITE_REQ)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)
           )
        {
            if (p_auth_req->request.write.handle == p_atom_ctrlpt->sterzo_ctrlpt_handles.value_handle)
            {                
                on_ctrlpt_write(p_atom_ctrlpt, &p_auth_req->request.write);
            }
            else
            {
                NRF_LOG_ERROR("ble_atom_ctrlpt::on_rw_authorize_request(), type == BLE_GATTS_AUTHORIZE_TYPE_WRITE, addressed handle: 0x%02x", p_auth_req->request.write.handle);
            }
        }
    }
    else
    {
        NRF_LOG_ERROR("Different request type in on_rw_authorize_request: 0x%02x", p_auth_req->type);
    }
}


/**@brief Tx Complete event handler.
 *
 * @details Tx Complete event handler.
 *          Handles WRITE events from the BLE stack and if an indication was pending try sending it
 *          again.
 *
 * @param[in]   p_sc_ctrlpt  SC Ctrlpt structure.
 *
 */
static void on_ctrlpt_tx_complete(ble_atom_ctrlpt_t * p_atom_ctrlpt)
{
    if (p_atom_ctrlpt->procedure_status == BLE_ATOMPT_INDICATION_PENDING)
    {
        sterzo_ctrlpt_resp_send(p_atom_ctrlpt);
    }
}


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_sc_ctrlpt  SC Ctrlpt structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_atom_ctrlpt_t * p_atom_ctrlpt, ble_evt_t const * p_ble_evt)
{
    // NRF_LOG_INFO("ble_atom_ctrlpt::on_connect: Central connected to us");
    p_atom_ctrlpt->conn_handle      = p_ble_evt->evt.gap_evt.conn_handle;
    p_atom_ctrlpt->procedure_status = BLE_ATOMPT_NO_PROC_IN_PROGRESS;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_sc_ctrlpt  SC Ctrlpt structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_atom_ctrlpt_t * p_atom_ctrlpt, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    // NRF_LOG_INFO("ble_atom_ctrlpt::on_disconnect: Central disconnected from us");
    p_atom_ctrlpt->conn_handle      = BLE_CONN_HANDLE_INVALID;
    p_atom_ctrlpt->procedure_status = BLE_ATOMPT_NO_PROC_IN_PROGRESS;

}


/**@brief Function for handling the BLE_GATTS_EVT_HVC event.
 *
 * @param[in]   p_sc_ctrlpt  SC Ctrlpt structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_sterzo_hvc_confirm(ble_atom_ctrlpt_t * p_atom_ctrlpt, ble_evt_t const * p_ble_evt)
{
    // NRF_LOG_INFO("ble_atom_ctrlpt::on_sterzo_hvc_confirm() enter");
    if (p_ble_evt->evt.gatts_evt.params.hvc.handle == p_atom_ctrlpt->sterzo_ctrlpt_handles.value_handle)
    {
        NRF_LOG_INFO("ble_atom_ctrlpt::on_sterzo_hvc_confirm()");
        if (p_atom_ctrlpt->procedure_status == BLE_ATOMPT_IND_CONFIRM_PENDING)
        {
            p_atom_ctrlpt->procedure_status = BLE_ATOMPT_NO_PROC_IN_PROGRESS;
        }
    }
}


void ble_atom_ctrlpt_on_ble_evt(ble_atom_ctrlpt_t * p_atom_ctrlpt, ble_evt_t const * p_ble_evt)
{
    if (p_atom_ctrlpt == NULL || p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //NRF_LOG_INFO("ble_atom_ctrlpt::ble_atom_ctrlpt_on_ble_evt::BLE_GAP_EVT_CONNECTED");
            on_connect(p_atom_ctrlpt, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //NRF_LOG_INFO("ble_atom_ctrlpt::ble_atom_ctrlpt_on_ble_evt::BLE_GAP_EVT_DISCONNECTED");
            on_disconnect(p_atom_ctrlpt, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            // NRF_LOG_INFO("ble_atom_ctrlpt::ble_atom_ctrlpt_on_ble_evt::BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST");
            on_rw_authorize_request(p_atom_ctrlpt, &p_ble_evt->evt.gatts_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            //NRF_LOG_INFO("ble_atom_ctrlpt::ble_atom_ctrlpt_on_ble_evt::BLE_GATTS_EVT_HVC");
            on_sterzo_hvc_confirm(p_atom_ctrlpt, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            // NRF_LOG_INFO("ble_atom_ctrlpt::ble_atom_ctrlpt_on_ble_evt::BLE_GATTS_EVT_HVN_TX_COMPLETE");
            on_ctrlpt_tx_complete(p_atom_ctrlpt);
            break;

        case BLE_GATTS_EVT_WRITE:
            {

            
             ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
             //NRF_LOG_INFO("Requested handle: %02x, our ctrlpt handle: %02x", p_evt_write->handle, p_atom_ctrlpt->sterzo_ctrlpt_handles.value_handle);
             if (p_evt_write->handle == p_atom_ctrlpt->sterzo_ctrlpt_handles.value_handle) 
             {
                NRF_LOG_INFO("ble_atom_ctrlpt::ble_atom_ctrlpt_on_ble_evt::BLE_GATTS_EVT_WRITE");                            
                on_ctrlpt_write(p_atom_ctrlpt, p_evt_write);
                }
            
            else
            {
               // NRF_LOG_INFO("ble_atom_ctrlpt::ble_atom_ctrlpt_on_ble_evt::BLE_GATTS_EVT_WRITE - but not for us :-( (Handle: 0x%x", p_evt_write->handle);                          
            }
            }
            break;
        default:
            if (p_ble_evt->header.evt_id != BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST &&
                p_ble_evt->header.evt_id != 0x1d )
            {
                // NRF_LOG_INFO("Unhandled ble_atom_ctrlpt::on_ble_evt: 0x%x", p_ble_evt->header.evt_id);
            }
            
            break;
    }
}

uint32_t ble_atom_ctrlpt_rsp_send(ble_atom_ctrlpt_t * p_atom_ctrlpt, ble_atompt_response_t response_status)
{
    // This does not seem to be needed for our use case
    if (p_atom_ctrlpt == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t               err_code = NRF_SUCCESS;
    ble_atom_ctrlpt_rsp_t    rsp;
    uint8_t                encoded_ctrl_rsp[BLE_ATOM_CTRLPT_MAX_LEN];
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    NRF_LOG_INFO("ble_atom_ctrlpt::ble_atom_ctrlpt_rsp_send()");

    // WE need to add here all commands which would require a response

    if (p_atom_ctrlpt->procedure_status != BLE_ATOMPT_AUTHENTICATION_IN_PROGRESS)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    // Only automatic calibration here, switch case opcode
    rsp.status = response_status;
    rsp.opcode = BLE_ATOMPT_CONTROL_NOT_PERMITTED; //BLE_FTMSPT_START_AUTOMATIC_CALIBRATION; // all others that need a response
    hvx_len    = ctrlpt_rsp_encode(p_atom_ctrlpt, &rsp, encoded_ctrl_rsp);

    // Send more...


    // Send indication
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_atom_ctrlpt->sterzo_ctrlpt_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_INDICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = encoded_ctrl_rsp;
    NRF_LOG_INFO("Sending indication");
    NRF_LOG_HEXDUMP_INFO(&encoded_ctrl_rsp, hvx_len);
    NRF_LOG_INFO("End of indication");
    err_code = sd_ble_gatts_hvx(p_atom_ctrlpt->conn_handle, &hvx_params);

    if (err_code == NRF_SUCCESS)
    {
        p_atom_ctrlpt->procedure_status = BLE_ATOMPT_NO_PROC_IN_PROGRESS;
    }
    return err_code;
}
