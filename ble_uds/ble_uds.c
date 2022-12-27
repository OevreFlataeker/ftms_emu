#include "sdk_common.h"

#if NRF_MODULE_ENABLED(BLE_UDS)
#define NRF_LOG_MODULE_NAME ble_uds
#if BLE_UDS_CONFIG_LOG_ENABLED // Disabled for debugging
#define NRF_LOG_LEVEL       BLE_UDS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_UDS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_UDS_CONFIG_DEBUG_COLOR
#else // HELPER_UDS_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_UDS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "nrf_log_ctrl.h"
#include "stdio.h"
#include "assert.h"
#include "nrf_delay.h"
#include "ble_uds.h"

#define UDS_CTRLPT_NACK_PROC_ALREADY_IN_PROGRESS   (BLE_GATT_STATUS_ATTERR_APP_BEGIN + 0)
#define UDS_CTRLPT_NACK_CCCD_IMPROPERLY_CONFIGURED (BLE_GATT_STATUS_ATTERR_APP_BEGIN + 1)

#define LOGHERE 0

static void on_ctrlpt_cccd_write(ble_uds_t * p_uds, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // NRF_LOG_INFO("ctrlpt_cccd_written");
        // CCCD written, update notification state
        if (p_uds->evt_handler != NULL)
        {
            ble_uds_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_UDS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_UDS_EVT_NOTIFICATION_DISABLED;
            }

            if (ble_srv_is_indication_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_UDS_EVT_INDICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_UDS_EVT_INDICATION_DISABLED;
            }
            p_uds->evt_handler(p_uds, &evt);
        }
    }
}

static void on_change_increment_cccd_write(ble_uds_t * p_uds, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // NRF_LOG_INFO("training_status_cccd_written");
        // CCCD written, update notification state
        if (p_uds->evt_handler != NULL)
        {
            ble_uds_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_UDS_CHGINCREMENT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_UDS_CHGINCREMENT_NOTIFICATION_DISABLED;
            }

            p_uds->evt_handler(p_uds, &evt);
        }
    }
}

uint32_t ble_uds_init(ble_uds_t * p_uds, ble_uds_init_t const * p_uds_init)
{
    ret_code_t      err_code;
    ble_uuid_t      ble_uuid;

    uint16_t initial_weight = 200*100; // 200 * weight in kg
    uint16_t initial_height = 186; // height in cm
    uint8_t  initial_userindex = 0xff;
    uint32_t inital_change_increment = 0;
    /* Add User Data Service */

    BLE_UUID_BLE_ASSIGN (ble_uuid, BLE_UUID_UDS_SERVICE);

    err_code = sd_ble_gatts_service_add (BLE_GATTS_SRVC_TYPE_PRIMARY,
                                         &ble_uuid,
                                         &p_uds->service_handle);
    APP_ERROR_CHECK (err_code);

    p_uds->supported_functions = p_uds_init->supported_functions;
    p_uds->evt_handler = p_uds_init->evt_handler;
    p_uds->error_handler = p_uds_init->error_handler;


    /* Add weight characteristic */
    ble_add_char_params_t weight_char_params = {
        .uuid               = BLE_UUID_WEIGHT_CHAR,
        .max_len            = 2,
        .init_len           = 2,
        .p_init_value       = (uint8_t *) &initial_weight, 
        .char_props.read    = 1,
          .char_props.write    = 1,
        .read_access        = SEC_OPEN
    };
    
    err_code = characteristic_add (p_uds->service_handle,
                                   &weight_char_params,
                                   &p_uds->ud_weight_handle);
    APP_ERROR_CHECK (err_code);

    // Add height characteristic
    ble_add_char_params_t height_char_params = {
        .uuid               = BLE_UUID_HEIGHT_CHAR,
        .max_len            = 1,
        .init_len           = 1,
        .p_init_value       = (uint8_t *) &initial_height,
        .char_props.read    = 1,
          .char_props.write    = 1,
        .read_access        = SEC_OPEN
    };

    err_code = characteristic_add (p_uds->service_handle,
                                   &height_char_params,
                                   &p_uds->ud_height_handle);
    APP_ERROR_CHECK (err_code);
    /*
    // Add First Name characteristic 
    char * fname = "Markus";
    ble_add_char_params_t fname_char_params = {
        .uuid               = BLE_UUID_FIRSTNAME_CHAR,
        .max_len            = strlen (fname),
        .init_len           = strlen (fname),
        .p_init_value       = (uint8_t *) fname,
        .char_props.read    = 1u,
          .char_props.write    = 1,
        .read_access        = SEC_OPEN
    };

    err_code = characteristic_add (p_uds->service_handle,
                                   &fname_char_params,
                                   &p_uds->ud_firstname_handle);
    APP_ERROR_CHECK (err_code);

    // Add Last Name characteristic 
    char * lname = "Dauberschmidt";
    ble_add_char_params_t lname_char_params = {
        .uuid               = BLE_UUID_LASTNAME_CHAR,
        .max_len            = strlen (lname),
        .init_len           = strlen (lname),
        .p_init_value       = (uint8_t *) lname,
        .char_props.read    = 1u,
          .char_props.write    = 1,
        .read_access        = SEC_OPEN
    };

    err_code = characteristic_add (p_uds->service_handle,
                                   &lname_char_params,
                                   &p_uds->ud_lastname_handle);
    APP_ERROR_CHECK (err_code);
    */

    // Add database change increment characteristic    
    ble_add_char_params_t change_increment_char_params = {
        .uuid               = BLE_UUID_CHANGE_INCREMENT_CHAR,
        .max_len            = sizeof(uint32_t),
        .init_len           = sizeof(uint32_t),
        .p_init_value       = (uint8_t *) &inital_change_increment,     // On reset    
        .read_access        = SEC_OPEN,
        .char_props.read    = 1,
        .char_props.write    = 1,
        .char_props.notify  = 1,
        .cccd_write_access   = SEC_OPEN,
        .write_access        = SEC_OPEN,

    };

    err_code = characteristic_add (p_uds->service_handle,
                                   &change_increment_char_params,
                                   &p_uds->ud_change_increment_handle);
    APP_ERROR_CHECK (err_code);

    // Add user index characteristic    
    ble_add_char_params_t user_index_char_params = {
        .uuid               = BLE_UUID_USER_INDEX_CHAR,
        .max_len            = sizeof(uint8_t),
        .init_len           = sizeof(uint8_t),
        .p_init_value       = &initial_userindex,         // Maybe also 0xff?
        .read_access        = SEC_OPEN,
        .char_props.read    = 1,
    };

    err_code = characteristic_add (p_uds->service_handle,
                                   &user_index_char_params,
                                   &p_uds->ud_index_handle);
    APP_ERROR_CHECK (err_code);

    // Add user controlpoint characteristic    
    ble_add_char_params_t user_ctrlpt_char_params = {
        .uuid                = BLE_UUID_UDS_CTRLPT_CHAR,
        .is_var_len          = true,
        .char_props.indicate = 1,
        .char_props.write    = 1,
        .cccd_write_access   = SEC_OPEN,
        .write_access        = SEC_OPEN,
        .is_defered_write    = true,
        .max_len             = 19*sizeof(uint8_t),
        .init_len            = 19*sizeof(uint8_t),
        .p_init_value        = 0,         // Maybe also 0xff?
        .read_access         = SEC_OPEN
    };

    err_code = characteristic_add (p_uds->service_handle,
                                   &user_ctrlpt_char_params,
                                   &p_uds->ctrlpt_handle);
    APP_ERROR_CHECK (err_code);
    NRF_LOG_INFO("Initialized UDS");
}

static void on_write(ble_uds_t * p_uds, ble_evt_t const * p_ble_evt)
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
        
        if (p_evt_write->handle == p_uds->ud_weight_handle.cccd_handle)
        {   
          //on_indoor_bike_cccd_write(p_uds, p_evt_write);
        }
        else if (p_evt_write->handle == p_uds->ud_height_handle.cccd_handle)
        {   
          //on_machine_status_cccd_write(p_uds, p_evt_write);
        }
        else if (p_evt_write->handle == p_uds->ud_lastname_handle.cccd_handle)
        {   
          //on_training_status_cccd_write(p_uds, p_evt_write);
        }
        else if (p_evt_write->handle == p_uds->ud_change_increment_handle.cccd_handle)
        {
          on_change_increment_cccd_write(p_uds, p_evt_write);
        }
        else if (p_evt_write->handle == p_uds->ctrlpt_handle.cccd_handle)
        {
          on_ctrlpt_cccd_write(p_uds, p_evt_write);
        }
        
#if LOGHERE
        else {
            NRF_LOG_INFO("CCCD handle 0x%04x unknown" , p_evt_write->handle);
            NRF_LOG_INFO("ble_ftms::on_write() gatts_evt: handle: 0x%02x, uuid: 0x%04x, type: 0x%02x, len: 0x%04x, data[0]=0x%02x", p_evt_write->handle, p_evt_write->uuid.uuid, p_evt_write->uuid.type, p_evt_write->len, p_evt_write->data[0]); 
      
        }  
#endif            
    }
    if (p_evt_write->handle == p_uds->ctrlpt_handle.value_handle)
    {
        NRF_LOG_INFO("Writing to Ctrlpt value");
        on_ctrlpt_write(p_uds, p_evt_write);
    }
    
    //NRF_LOG_INFO("ble_uds::on_write() will need Impl!");
}


static void on_ctrlpt_write(ble_uds_t             * p_uds,
                            ble_gatts_evt_write_t const * p_evt_write)
{
    // THIS ONE IS NOT FULLY IMPLEMENTED! UDS does not seem to be used by Zwift?
    // Might crash at ease!

    ble_udspt_val_t                  rcvd_ctrlpt;


    ble_uds_resp_t                   rsp;
    uint32_t                              err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;
    ble_uds_evt_t                   evt;


    memset(&rcvd_ctrlpt, 0, sizeof(rcvd_ctrlpt));
    NRF_LOG_INFO("ble_uds::on_ctrlpt_write()");
    auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
    auth_reply.params.write.offset      = 0;
    auth_reply.params.write.len         = 0;
    auth_reply.params.write.p_data      = NULL;
    auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
    auth_reply.params.write.update      = 1;

    if (is_cccd_configured(p_uds))
    {
        if (p_uds->procedure_status == BLE_UDSPT_NO_PROC_IN_PROGRESS)
        {
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        }
        else
        {
            auth_reply.params.write.gatt_status = UDS_CTRLPT_NACK_PROC_ALREADY_IN_PROGRESS;
        }
    }
    else
    {
        auth_reply.params.write.gatt_status = UDS_CTRLPT_NACK_CCCD_IMPROPERLY_CONFIGURED;
    }

    err_code = sd_ble_gatts_rw_authorize_reply(p_uds->conn_handle, &auth_reply);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application.
        if (p_uds->error_handler != NULL)
        {
            p_uds->error_handler(err_code);
        }
    }

    if (auth_reply.params.write.gatt_status != BLE_GATT_STATUS_SUCCESS)
    {
        return;
    }

    p_uds->procedure_status = BLE_UDSPT_INDICATION_PENDING;
    rsp.status                    = BLE_UDSPT_OP_CODE_NOT_SUPPORTED;

    NRF_LOG_HEXDUMP_INFO(p_evt_write->data, p_evt_write->len);
    
    err_code = uds_ctrlpt_decode(p_evt_write->data, p_evt_write->len, &rcvd_ctrlpt);
    if (err_code != NRF_SUCCESS)
    {
        rsp.opcode = rcvd_ctrlpt.opcode;
        rsp.status = BLE_UDSPT_OP_CODE_NOT_SUPPORTED;
    }
    else
    {
        rsp.opcode = rcvd_ctrlpt.opcode;

        switch (rcvd_ctrlpt.opcode)
        {
            
            case BLE_UDS_REGISTER_NEW_USER:
                if ((p_uds->supported_functions &
                     BLE_SRV_UDS_CTRLPT_REGISTER_NEW_USER) ==
                     BLE_SRV_UDS_CTRLPT_REGISTER_NEW_USER)
                    {                        
                        rsp.status = BLE_UDS_SUCCESS;
                        evt.evt_type                = BLE_UDS_CTRLPT_EVT_REGISTER_NEW_USER;
                        
                        if (p_uds->evt_handler != NULL)
                        {
                            // Do the action (calls main)
                            //rsp.status = p_uds->evt_handler(p_uds, &evt);
                            p_uds->evt_handler(p_uds, &evt);
                        }   
                        // Send response
                        rsp.responselen = 1;
                        rsp.response = 0;                 
                    }
                else
                {
                    rsp.status = BLE_UDS_OP_CODE_NOT_SUPPORTED;
                }
                break;
               
            case BLE_UDS_CONSENT:
             if ((p_uds->supported_functions &
                     BLE_SRV_UDS_CTRLPT_CONSENT) ==
                     BLE_SRV_UDS_CTRLPT_CONSENT)
                    {
                        // Check if control has been acquired                                            

                        rsp.status = BLE_UDS_SUCCESS;
                        evt.evt_type                = BLE_UDS_CTRLPT_EVT_CONSENT;
                        if (p_uds->evt_handler != NULL)
                        {
                            // Do the action (calls main)
                            // rsp.status = p_uds->evt_handler(p_uds, &evt);
                            p_uds->evt_handler(p_uds, &evt);
                        }
                        NRF_LOG_INFO("We received %d as rsp.status", rsp.status);                                                
                    }
                    else
                    {
                        rsp.status = BLE_UDSPT_OP_CODE_NOT_SUPPORTED;
                    }                
                break;  
            case BLE_UDS_DELETE_USER:
             if ((p_uds->supported_functions &
                     BLE_SRV_UDS_CTRLPT_DELETE_USER_SUPPORTED) ==
                     BLE_SRV_UDS_CTRLPT_DELETE_USER_SUPPORTED)
                    {
                        rsp.status = BLE_UDS_SUCCESS;
                        evt.evt_type                = BLE_UDS_CTRLPT_EVT_DELETE_USER;
                        if (p_uds->evt_handler != NULL)
                        {
                            // Do the action (calls main)
                            //rsp.status = p_uds->evt_handler(p_uds, &evt);
                            p_uds->evt_handler(p_uds, &evt);
                        }
                        // Send response
                        rsp.responselen = 1;
                        rsp.response = 0xff;     
                        NRF_LOG_INFO("We received %d as rsp.status", rsp.status);    
                    }
                    else
                {
                    rsp.status = BLE_UDSPT_OP_CODE_NOT_SUPPORTED;
                }
                break;
                
            case BLE_UDS_DELETE_USER_DATA:
            if ((p_uds->supported_functions &
                     BLE_SRV_UDS_CTRLPT_DELETE_USER_DATA_SUPPORTED) ==
                     BLE_SRV_UDS_CTRLPT_DELETE_USER_DATA_SUPPORTED)
                    {
                        rsp.status = BLE_UDS_SUCCESS;
                        evt.evt_type                = BLE_UDS_CTRLPT_EVT_DELETE_USER_DATA;
                        if (p_uds->evt_handler != NULL)
                        {
                            // Do the action (calls main)
                            //rsp.status = p_uds->evt_handler(p_uds, &evt);
                            p_uds->evt_handler(p_uds, &evt);
                        }
                        NRF_LOG_INFO("We received %d as rsp.status", rsp.status);        
                    }
                    else
                {
                    rsp.status = BLE_UDSPT_OP_CODE_NOT_SUPPORTED;
                }
                break;
                
            case BLE_UDS_LIST_ALL_USERS:
             if ((p_uds->supported_functions &
                     BLE_SRV_UDS_CTRLPT_LIST_ALL_USERS_SUPPORTED) ==
                     BLE_SRV_UDS_CTRLPT_LIST_ALL_USERS_SUPPORTED)
                    {
                         rsp.status = BLE_UDS_SUCCESS;
                        evt.evt_type                = BLE_UDS_CTRLPT_EVT_LIST_ALL_USERS;
                        if (p_uds->evt_handler != NULL)
                        {
                            // Do the action (calls main)
                            //rsp.status = p_uds->evt_handler(p_uds, &evt);
                            p_uds->evt_handler(p_uds, &evt);
                        }
                        NRF_LOG_INFO("We received %d as rsp.status", rsp.status);  
                        // Send response
                        rsp.responselen = 1;
                        rsp.response = 1;  // 1 User          
                    }
                    else
                {
                    rsp.status = BLE_UDSPT_OP_CODE_NOT_SUPPORTED;
                }
                break;

                 case BLE_UDS_RESPONSE_CODE:
             if ((p_uds->supported_functions &
                     BLE_SRV_UDS_CTRLPT_RESPONSE_CODE_SUPPORTED) ==
                     BLE_SRV_UDS_CTRLPT_RESPONSE_CODE_SUPPORTED)
                    {
                        rsp.status = BLE_UDS_SUCCESS;
                        evt.evt_type                = BLE_UDS_CTRLPT_EVT_RESPONSE_CODE;
                        if (p_uds->evt_handler != NULL)
                        {
                            // Do the action (calls main)
                            //rsp.status = p_uds->evt_handler(p_uds, &evt);
                            p_uds->evt_handler(p_uds, &evt);
                        }
                        NRF_LOG_INFO("We received %d as rsp.status", rsp.status);           
                    }
                    else
                {
                    rsp.status = BLE_UDSPT_OP_CODE_NOT_SUPPORTED;
                }
                break;

            default:
                NRF_LOG_INFO("Received an unsupported opcode in ble_uds::on_ctrlpt_write(): 0x%02x", rcvd_ctrlpt.opcode);
                rsp.status = BLE_UDS_OP_CODE_NOT_SUPPORTED;
                break;
        }

    }

    p_uds->response.len = ctrlpt_rsp_encode(p_uds, &rsp,
                                                  p_uds->response.encoded_rsp);


    if (p_uds->procedure_status == BLE_UDSPT_INDICATION_PENDING)
    {
        NRF_LOG_INFO("Sending a resp");
        uds_ctrlpt_resp_send(p_uds);
    }
}

static void on_rw_authorize_request(ble_uds_t       * p_uds,
                                    ble_gatts_evt_t const * p_gatts_evt)
{
    ble_gatts_evt_rw_authorize_request_t const * p_auth_req =
        &p_gatts_evt->params.authorize_request;

    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        // NRF_LOG_INFO("ble_ftms_ctrlpt::on_rw_authorize_request(), type == BLE_GATTS_AUTHORIZE_TYPE_WRITE");
        if (   (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_PREP_WRITE_REQ)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)
           )
        {
            if (p_auth_req->request.write.handle == p_uds->ctrlpt_handle.value_handle)
            {
                NRF_LOG_INFO("success");
                on_ctrlpt_write(p_uds, &p_auth_req->request.write);
            }            
        }
    }
}

static void on_uds_tx_complete(ble_uds_t * p_uds)
{
    if (p_uds->procedure_status == BLE_UDSPT_INDICATION_PENDING)
    {
        NRF_LOG_INFO("Sending a resp from on_tx");
        uds_ctrlpt_resp_send(p_uds);
    }
}

static void on_connect(ble_uds_t * p_uds, ble_evt_t const * p_ble_evt)
{
    p_uds->conn_handle      = p_ble_evt->evt.gap_evt.conn_handle;
    p_uds->procedure_status = BLE_UDSPT_NO_PROC_IN_PROGRESS;    
}

static void on_disconnect(ble_uds_t * p_uds, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_uds->conn_handle      = BLE_CONN_HANDLE_INVALID;
    p_uds->procedure_status = BLE_UDSPT_NO_PROC_IN_PROGRESS;
}

static void on_uds_hvc_confirm(ble_uds_t * p_uds, ble_evt_t const * p_ble_evt)
{
    if (p_ble_evt->evt.gatts_evt.params.hvc.handle == p_uds->ctrlpt_handle.value_handle)
    {
        NRF_LOG_INFO("ble_uds::on_uds_hvc_confirm()");
        if (p_uds->procedure_status == BLE_UDSPT_IND_CONFIRM_PENDING)
        {
            p_uds->procedure_status = BLE_UDSPT_NO_PROC_IN_PROGRESS;
        }
    }
}


void ble_uds_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{

    ble_uds_t * p_uds = (ble_uds_t *)p_context;
    if (p_uds == NULL || p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_DEBUG("ble_uds::ble_uds_on_ble_evt::BLE_GAP_EVT_CONNECTED");
            on_connect(p_uds, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_DEBUG("ble_uds::ble_uds_on_ble_evt::BLE_GAP_EVT_DISCONNECTED");
            on_disconnect(p_uds, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            NRF_LOG_DEBUG("ble_uds::ble_uds_on_ble_evt::BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST");
            on_rw_authorize_request(p_uds, &p_ble_evt->evt.gatts_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            NRF_LOG_DEBUG("ble_uds::ble_uds_on_ble_evt::BLE_GATTS_EVT_HVC");
            on_uds_hvc_confirm(p_uds, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            NRF_LOG_DEBUG("ble_uds::ble_uds_on_ble_evt::BLE_GATTS_EVT_HVN_TX_COMPLETE");
            on_uds_tx_complete(p_uds);
            break;

        case BLE_GATTS_EVT_WRITE:
            NRF_LOG_DEBUG("ble_uds::ble_uds_on_ble_evt::BLE_GATTS_EVT_WRITE");
            on_write(p_uds, p_ble_evt);
            break;
        default:
            break;
    }
}

static uint32_t uds_ctrlpt_decode(uint8_t const       * p_rcvd_val,
                                 uint8_t               len,
                                 ble_udspt_val_t * p_write_val)
{
    // THIS ONE IS NOT FULLY IMPLEMENTED! UDS does not seem to be used by Zwift?
    int pos = 0;
    uint16_t value16;
    int16_t value_s16;
    uint8_t value8;
    int8_t value_s8;

    if (len < BLE_UDS_MIN_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    p_write_val->opcode = (ble_udspt_operator_t) p_rcvd_val[pos++];

    switch (p_write_val->opcode)
    {
        // Table 4.16.1 Fitness Machine Control Point Procedure Requirement
                     
        case BLE_UDS_REGISTER_NEW_USER:
            NRF_LOG_INFO("ble_uds_ctrlpt::uds_ctrlpr_decode() - Decoding params for BLE_UDS_REGISTER_NEW_USER");
            value16 = uint16_decode(&(p_rcvd_val[pos])); // TODO: According to spec it should decode a sint16_t, percent with resolution 0,1%
            //value_s16 = sint16_decode(&(p_rcvd_val[pos])); // TODO: According to spec it should decode a sint16_t, percent with resolution 0,1%           
            //p_write_val->target_inclination = value_s16;
            p_write_val->consent_code = value16; 
            break;

        case BLE_UDS_CONSENT:
            NRF_LOG_INFO("ble_uds_ctrlpt::uds_ctrlpr_decode() - Decoding params for BLE_UDSPT_CONSENT");
            value8 = p_rcvd_val[pos]; // userid
            value16 = uint16_decode(&(p_rcvd_val[pos+1])); // TODO: According to spec it should decode a sint16_t, Watt with resolution 1 W
            //value_s16 = sint16_decode(&(p_rcvd_val[pos])); // TODO: According to spec it should decode a sint16_t, Watt with resolution 1 W           
            p_write_val->user_index = value8;
            p_write_val->consent_code = value16; 
            break;

        case BLE_UDS_DELETE_USER:
            NRF_LOG_INFO("ble_uds_ctrlpt::uds_ctrlpr_decode() - Decoding params for BLE_UDSPT_DELETE_USER");
            //value16 = uint16_decode(&(p_rcvd_val[pos])); // TODO: According to spec it should decode a sint16_t, Watt with resolution 1 W
            //value_s16 = sint16_decode(&(p_rcvd_val[pos])); // TODO: According to spec it should decode a sint16_t, Watt with resolution 1 W           
            value8 = p_rcvd_val[pos];
            p_write_val->user_index = value8;
            break;

        case BLE_UDS_DELETE_USER_DATA:
            NRF_LOG_INFO("ble_uds_ctrlpt::uds_ctrlpr_decode() - Decoding params for BLE_UDSPT_DELETE_USER_DATA");
            //value16 = uint16_decode(&(p_rcvd_val[pos])); // TODO: According to spec it should decode a sint16_t, Watt with resolution 1 W
            //value_s16 = sint16_decode(&(p_rcvd_val[pos])); // TODO: According to spec it should decode a sint16_t, Watt with resolution 1 W           
            break;
        case BLE_UDS_LIST_ALL_USERS:
            NRF_LOG_INFO("ble_uds_ctrlpt::uds_ctrlpr_decode() - Decoding params for BLE_UDSPT_LIST_ALL_USERS");
            //value16 = uint16_decode(&(p_rcvd_val[pos])); // TODO: According to spec it should decode a sint16_t, Watt with resolution 1 W
            //value_s16 = sint16_decode(&(p_rcvd_val[pos])); // TODO: According to spec it should decode a sint16_t, Watt with resolution 1 W           
            break;
            
        case BLE_UDS_RESPONSE_CODE:
            NRF_LOG_INFO("ble_ftms_ctrlpt::ftms_ctrlpr_decode() - Decoding params for BLE_UDS_RESPONSE_CODE");
            value16 = uint16_decode(&(p_rcvd_val[pos]));                        
            break;               
        default:
            NRF_LOG_INFO("ble_uds_ctrlpt::uds_ctrlpr_decode() - NRF_ERROR_INVALID_PARAM");
            return NRF_ERROR_INVALID_PARAM;
    }
    return NRF_SUCCESS;
}


uint32_t ble_uds_ctrlpt_rsp_send(ble_uds_t * p_uds, ble_udspt_response_t response_status)
{
    // This does not seem to be needed for our use case
    if (p_uds == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t               err_code = NRF_SUCCESS;
    ble_uds_resp_t    rsp;
    uint8_t                encoded_ctrl_rsp[BLE_UDS_MAX_LEN];
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    NRF_LOG_INFO("ble_uds::ble_udss_ctrlpt_rsp_send()");

    // WE need to add here all commands which would require a response

       
    // Only automatic calibration here, switch case opcode
    rsp.status = response_status;
    rsp.opcode = BLE_UDSPT_USER_NOT_AUTHORIZED; //BLE_FTMSPT_START_AUTOMATIC_CALIBRATION; // all others that need a response
    hvx_len    = ctrlpt_rsp_encode(p_uds, &rsp, encoded_ctrl_rsp);

    // Send more...


    // Send indication
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_uds->ctrlpt_handle.value_handle;
    hvx_params.type   = BLE_GATT_HVX_INDICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = encoded_ctrl_rsp;
    NRF_LOG_INFO("Sending indication");
    NRF_LOG_HEXDUMP_INFO(&encoded_ctrl_rsp, hvx_len);
    //NRF_LOG_INFO("End of indication");
    err_code = sd_ble_gatts_hvx(p_uds->conn_handle, &hvx_params);

    if (err_code == NRF_SUCCESS)
    {
        p_uds->procedure_status = BLE_UDSPT_NO_PROC_IN_PROGRESS;
    }
    return err_code;
}

static int ctrlpt_rsp_encode(ble_uds_t     * p_uds,
                             ble_uds_resp_t * p_rsp,
                             uint8_t             * p_data)
{
    int len = 0;

    p_data[len++] = BLE_UDS_RESPONSE_CODE;
    p_data[len++] = p_rsp->opcode;
    p_data[len++] = p_rsp->status;
    uint8_t *resp_ptr = &p_rsp->response;

    for (int i = 0; i<p_rsp->responselen;i++)
    {
        p_data[len++] = *(&resp_ptr[i]);
    }

#if LOGHERE
    NRF_LOG_INFO("ble_uds_ctrlpt::ctrlpt_rsp_encode: %02x %02x %02x", BLE_UDSPT_RESPONSE_CODE, p_ctrlpt_rsp->opcode, p_ctrlpt_rsp->status); 
#endif
    if (p_rsp->status == BLE_UDS_SUCCESS)
    {
        switch (p_rsp->opcode)
        {           
            default:
                // No implementation needed.
                break;
        }
    }
    return len;
}


static bool is_cccd_configured(ble_uds_t * p_uds)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    bool     is_udscp_indic_enabled = false;
    ble_gatts_value_t gatts_value;

    NRF_LOG_INFO("ble_uds::is_cccd_configured()");
    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = BLE_CCCD_VALUE_LEN;
    gatts_value.offset  = 0;
    gatts_value.p_value = cccd_value_buf;

    err_code = sd_ble_gatts_value_get(p_uds->conn_handle,
                                      p_uds->ctrlpt_handle.cccd_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application
        if (p_uds->error_handler != NULL)
        {
            p_uds->error_handler(err_code);
        }
    }

    is_udscp_indic_enabled = ble_srv_is_indication_enabled(cccd_value_buf);
    if (!is_udscp_indic_enabled)
    {
        NRF_LOG_INFO("Indication is NOT enabled");
    }
    return is_udscp_indic_enabled;
}


static void uds_ctrlpt_resp_send(ble_uds_t * p_uds)
{
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    uint32_t               err_code;
        
    if (p_uds->procedure_status == BLE_UDSPT_INDICATION_PENDING)
    {
        NRF_LOG_INFO("ble_uds::uds_ctrlpt_resp_send() - BLE_UDSPT_INDICATION_PENDING");
        hvx_len = p_uds->response.len;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_uds->ctrlpt_handle.value_handle;
        hvx_params.type   = BLE_GATT_HVX_INDICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = p_uds->response.encoded_rsp;

        NRF_LOG_HEXDUMP_INFO(hvx_params.p_data, *hvx_params.p_len);
        err_code = sd_ble_gatts_hvx(p_uds->conn_handle, &hvx_params);

        // Error handling
        if ((err_code == NRF_SUCCESS) && (hvx_len != p_uds->response.len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

        switch (err_code)
        {
            case NRF_SUCCESS:
                NRF_LOG_INFO("ble_ftms_ctrlpt::ftms_ctrlpt_resp_send() - NRF_SUCCESS");
                p_uds->procedure_status = BLE_UDSPT_IND_CONFIRM_PENDING;
                // Wait for HVC event
                break;

            case NRF_ERROR_RESOURCES:
                // NRF_LOG_INFO("ble_ftms_ctrlpt::ftms_ctrlpt_resp_send() - NRF_ERROR_RESOURCES");
                // Wait for TX_COMPLETE event to retry transmission.
                p_uds->procedure_status = BLE_UDSPT_INDICATION_PENDING;
                break;

            default:
                // Report error to application.
                // NRF_LOG_INFO("ble_ftms_ctrlpt::ftms_ctrlpt_resp_send() - BLE_FTMSPT_NO_PROC_IN_PROGRESS");
                p_uds->procedure_status = BLE_UDSPT_NO_PROC_IN_PROGRESS;
                if (p_uds->error_handler != NULL)
                {
                    p_uds->error_handler(err_code);
                }
                break;
        }
    }
}
#endif // NRF_MODULE_ENABLED(BLE_UDS)