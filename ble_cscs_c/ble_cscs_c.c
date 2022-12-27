
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_CSCS_C)
#include "ble_cscs_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_gattc.h"
#include "nrf_drv_clock.h"

#define NRF_LOG_MODULE_NAME ble_cscs_c
#if BLE_CSCS_C_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_CSCS_C_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_CSCS_C_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_CSCS_C_CONFIG_DEBUG_COLOR
#else // BLE_CSCS_C_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_CSCS_C_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

//#define NRF_LOG_MODULE_NAME ble_cscs_c
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
//NRF_LOG_MODULE_REGISTER();

#include "math.h"
#include "helper.h"
#include "calculations.h"

#include "oled_controller.h"

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */
#define SIM_INTERVAL 1000

APP_TIMER_DEF(m_sim_timer_id);

// ble_cscs_c_calculation_helper_t cscs_c_calculation_helper;

void setupHomeTrainerTimer(void);
void disconnectCentralForcefully(void);

#if CSV_LOGGING
char csvline[CSV_LINE_LENGTH];

volatile bool doInjectSimEvent;

#endif // CSV_LOGGING==1


/**@brief Function for intercepting the errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void     * p_ctx,
                               uint16_t   conn_handle)
{
    ble_cscs_c_t * p_ble_cscs_c = (ble_cscs_c_t *)p_ctx;
    
    NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

    if (p_ble_cscs_c->error_handler != NULL)
    {
        p_ble_cscs_c->error_handler(nrf_error);
    }
}

/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function uses the Handle Value Notification received from the SoftDevice
 *            and checks whether it is a notification of the Cycling Speed and Cadence measurement from
 *            the peer. If it is, this function decodes the Cycling Speed measurement and sends it
 *            to the application.
 *
 * @param[in] p_ble_cscs_c Pointer to the Cycling Speed and Cadence Client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event received.
 */

static void sim_timer_handler(void *p_ble_cscs_c)
{
    if (!doInjectSimEvent)
    {
        doInjectSimEvent = true;
    }
/*
static uint16_t cum_c_revs = 0;
static uint16_t last_c_evt_time = 0;

    uint8_t data[10];
    ble_gattc_evt_hvx_t *notif = (ble_gattc_evt_hvx_t *) data;
    
    uint8_t len = 5;
    uint8_t index = 0;

    notif->handle = 0xffff;
    notif->type = 0xff;
    notif->len = 5;

    cum_c_revs += 1;
    last_c_evt_time += 1000;

    index = 0;
    
    notif->data[index] = 1 << BLE_CSCS_CRANK_REV_DATA_PRESENT;
    index++;

    uint16_encode(cum_c_revs, &notif->data[index]);
    index += sizeof(uint16_t);

    uint16_encode(last_c_evt_time, &notif->data[index]);
    NRF_LOG_INFO("Tick! %u/%u", cum_c_revs, last_c_evt_time);
    updateModelFromCSCSensorData(notif);
        
    ble_cscs_c_evt_t ble_cscs_c_evt;
    ble_cscs_c_evt.evt_type    = BLE_CSCS_C_EVT_CSC_NOTIFICATION;
    
    ble_cscs_c_evt.params.csc.is_crank_rev_data_present = true;
    ble_cscs_c_evt.params.csc.is_wheel_rev_data_present = true;
        
    ble_cscs_c_evt.params.csc.cumulative_crank_revs = getCumulativeCrankRevs(); 
    ble_cscs_c_evt.params.csc.last_crank_event_time = getLastCrankEventTime(); 
      
    ble_cscs_c_evt.params.csc.cumulative_wheel_revs = getCumulativeWheelRevs(); 
    ble_cscs_c_evt.params.csc.last_wheel_event_time = getLastWheelEventTime(); 
    ((ble_cscs_c_t *) p_ble_cscs_c)->evt_handler(((ble_cscs_c_t *) p_ble_cscs_c), &ble_cscs_c_evt);
*/
}

static void on_hvx(ble_cscs_c_t * p_ble_cscs_c, const ble_evt_t * p_ble_evt)
{
    const ble_gattc_evt_hvx_t * p_notif = &p_ble_evt->evt.gattc_evt.params.hvx;
    
    // Check if the event is on the link for this instance
    if (p_ble_cscs_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    
    // Check if this is a Running Speed and Cadence notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_cscs_c->peer_db.csc_handle)
    {
        uint32_t         index = 0;
        ble_cscs_c_evt_t ble_cscs_c_evt;
        ble_cscs_c_evt.evt_type    = BLE_CSCS_C_EVT_CSC_NOTIFICATION;
        ble_cscs_c_evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
        
        //lint -save -e415 -e416 -e662 "Access of out of bounds pointer" "Creation of out of bounds pointer"

        // Flags field
        ble_cscs_c_evt.params.csc.is_crank_rev_data_present  = p_notif->data[index] >> BLE_CSCS_CRANK_REV_DATA_PRESENT      & 0x01;
        ble_cscs_c_evt.params.csc.is_wheel_rev_data_present  = p_notif->data[index] >> BLE_CSCS_WHEEL_REV_DATA_PRESENT      & 0x01;
        
        index++;
        
        updateModelFromCSCSensorData(p_notif);
        

        if (ble_cscs_c_evt.params.csc.is_wheel_rev_data_present == true)
        {
          NRF_LOG_ERROR("We have wheel data present?? This should not happen!");         
          
          // Filled below: ble_cscs_c_evt.params.csc.cumulative_wheel_revs = getCumulativeWheelRevs();//cscs_c_calculation_helper.cumulative_wheel_revs;
          // Filled below: ble_cscs_c_evt.params.csc.last_wheel_event_time = getLastWheelEventTime(); //cscs_c_calculation_helper.last_wheel_event_time;
        }       
        
        if (ble_cscs_c_evt.params.csc.is_crank_rev_data_present == true)
        {            
          // Always set the raw values
          ble_cscs_c_evt.params.csc.cumulative_crank_revs = getCumulativeCrankRevs(); //TODO: WARNING: THIS IS NO LONGER RAW! //_cumulative_crank_revs;
          ble_cscs_c_evt.params.csc.last_crank_event_time = getLastCrankEventTime(); // TODO: WARNING: THIS IS NO LONGER RAW! //_last_crank_event_time;
       }

        //NRF_LOG_INFO("Received CSCS peripheral event");
        //NRF_LOG_HEXDUMP_INFO(p_notif->data, index);       
        // Set in event
        NRF_LOG_DEBUG("Sending cumulative_wheel_revs=%d, last_wheel_event_time=%d", 
              getCumulativeWheelRevs(),//cscs_c_calculation_helper.cumulative_wheel_revs, 
              getLastWheelEventTime()); // cscs_c_calculation_helper.last_wheel_event_time);
        ble_cscs_c_evt.params.csc.is_wheel_rev_data_present = true;
        
        ble_cscs_c_evt.params.csc.cumulative_wheel_revs = getCumulativeWheelRevs(); //cscs_c_calculation_helper.cumulative_wheel_revs;
        ble_cscs_c_evt.params.csc.last_wheel_event_time = getLastWheelEventTime(); // cscs_c_calculation_helper.last_wheel_event_time;        
        p_ble_cscs_c->evt_handler(p_ble_cscs_c, &ble_cscs_c_evt);
        
        //lint -restore
    }

}


/**@brief     Function for handling events from the Database Discovery module.
 *
 * @details   This function handles an event from the Database Discovery module, and determines
 *            whether it relates to the discovery of Cycling Speed and Cadence service at the peer. If it does, the function
 *            calls the application's event handler to indicate that the Cycling Speed and Cadence
 *            service was discovered at the peer. The function also populates the event with service-related
 * 			  information before providing it to the application.
 *
 * @param[in] p_evt Pointer to the event received from the Database Discovery module.
 *
 */
void ble_cscs_on_db_disc_evt(ble_cscs_c_t * p_ble_cscs_c, const ble_db_discovery_evt_t * p_evt)
{
    // Check if the Heart Rate Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_CYCLING_SPEED_AND_CADENCE &&
        p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE)
    {
        ble_cscs_c_evt_t evt;
        evt.conn_handle = p_evt->conn_handle;

        // Find the CCCD Handle of the Cycling Speed and Cadence characteristic.
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_CSC_MEASUREMENT_CHAR)
            {
                // Found Cycling Speed and Cadence characteristic. Store CCCD handle and break.
                evt.params.cscs_db.csc_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.params.cscs_db.csc_handle      =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                break;
            }
        }

        NRF_LOG_DEBUG("Cycling Speed and Cadence Service discovered at peer.");

        // If the instance has been assigned prior to db_discovery, assign the db_handles.
        if (p_ble_cscs_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if ((p_ble_cscs_c->peer_db.csc_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_cscs_c->peer_db.csc_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_cscs_c->peer_db = evt.params.cscs_db;
            }
        }

        evt.evt_type = BLE_CSCS_C_EVT_DISCOVERY_COMPLETE;              
        p_ble_cscs_c->evt_handler(p_ble_cscs_c, &evt);
    }
}


uint32_t ble_cscs_c_init(ble_cscs_c_t * p_ble_cscs_c, ble_cscs_c_init_t * p_ble_cscs_c_init)
{
    VERIFY_PARAM_NOT_NULL(p_ble_cscs_c);
    VERIFY_PARAM_NOT_NULL(p_ble_cscs_c_init);
    ble_uuid_t cscs_uuid;

    cscs_uuid.type = BLE_UUID_TYPE_BLE;
    cscs_uuid.uuid = BLE_UUID_CYCLING_SPEED_AND_CADENCE;

    p_ble_cscs_c->evt_handler             = p_ble_cscs_c_init->evt_handler;
    p_ble_cscs_c->error_handler           = p_ble_cscs_c_init->error_handler;
    p_ble_cscs_c->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_ble_cscs_c->peer_db.csc_cccd_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_cscs_c->peer_db.csc_handle      = BLE_GATT_HANDLE_INVALID;
    p_ble_cscs_c->p_gatt_queue            = p_ble_cscs_c_init->p_gatt_queue;
    p_ble_cscs_c->sim_mode                 = p_ble_cscs_c_init->sim_mode;
    
    initializeCalculationHelper(); 

    if (p_ble_cscs_c->sim_mode)
    {    
       // Create timers
       ret_code_t err_code = app_timer_create(&m_sim_timer_id,
                                APP_TIMER_MODE_REPEATED,                                
                                sim_timer_handler);
       APP_ERROR_CHECK(err_code);   
  
      // Fire every SIM_INTERVAL ms
      NRF_LOG_INFO("Sim mode activated (%dms)", SIM_INTERVAL);
  
      err_code = app_timer_start(m_sim_timer_id, APP_TIMER_TICKS(SIM_INTERVAL), p_ble_cscs_c);
      APP_ERROR_CHECK(err_code);

      setupHomeTrainerTimer(); // Implicit declaration is ok for now (would need to be defined in "main.h")
      startOLEDUpdates();
      startProcessing();
    }

          
    return ble_db_discovery_evt_register(&cscs_uuid);
}


uint32_t ble_cscs_c_handles_assign(ble_cscs_c_t *    p_ble_cscs_c,
                                   uint16_t          conn_handle,
                                   ble_cscs_c_db_t * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_cscs_c);
    p_ble_cscs_c->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_cscs_c->peer_db = *p_peer_handles;
    }

    return nrf_ble_gq_conn_handle_register(p_ble_cscs_c->p_gatt_queue, conn_handle);
}


/**@brief     Function for handling Disconnected event received from the SoftDevice.
 *
 * @details   This function check whether the disconnect event is happening on the link
 *            associated with the current instance of the module. If the event is happening, the function sets the instance's
 *            conn_handle to invalid.
 *
 * @param[in] p_ble_cscs_c Pointer to the CSC Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_cscs_c_t * p_ble_cscs_c, const ble_evt_t * p_ble_evt)
{
uint32_t err_code = 0;

    if (p_ble_cscs_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_cscs_c->conn_handle             = BLE_CONN_HANDLE_INVALID;
        p_ble_cscs_c->peer_db.csc_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_cscs_c->peer_db.csc_handle      = BLE_GATT_HANDLE_INVALID; 
        NRF_LOG_INFO("CSCS peripheral (cadence sensor) disconnected (due to inactivity?). Stopping model calculations and OLED updates");
        stopProcessing();
        stopOLEDUpdates();
        oled_printStringAt(45,10, "CSCS disconnected", true, true);

        // Forcefully disconnect client (central)

        disconnectCentralForcefully();
        
    }
}


void ble_cscs_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_cscs_c_t * p_ble_cscs_c = (ble_cscs_c_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_cscs_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_cscs_c, p_ble_evt);            
            break;

        case BLE_GAP_EVT_CONNECTED:
            startOLEDUpdates();
            break;

        default:
            break;
    }
}


/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(ble_cscs_c_t * p_ble_cscs_c, bool enable)
{
    NRF_LOG_DEBUG("Configuring CSCS_C CCCD. CCCD Handle = %d, Connection Handle = %d",
                  p_ble_cscs_c->peer_db.csc_cccd_handle,
                  p_ble_cscs_c->conn_handle);

    uint8_t          cccd[WRITE_MESSAGE_LENGTH];
    uint16_t         cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    nrf_ble_gq_req_t cscs_c_req;

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    memset(&cscs_c_req, 0, sizeof(cscs_c_req));
    cscs_c_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    cscs_c_req.error_handler.cb            = gatt_error_handler;
    cscs_c_req.error_handler.p_ctx         = p_ble_cscs_c;
    cscs_c_req.params.gattc_write.handle   = p_ble_cscs_c->peer_db.csc_cccd_handle;
    cscs_c_req.params.gattc_write.len      = WRITE_MESSAGE_LENGTH;
    cscs_c_req.params.gattc_write.p_value  = cccd;
    cscs_c_req.params.gattc_write.offset   = 0;
    cscs_c_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;

    return nrf_ble_gq_item_add(p_ble_cscs_c->p_gatt_queue, &cscs_c_req, p_ble_cscs_c->conn_handle);
}


uint32_t ble_cscs_c_csc_notif_enable(ble_cscs_c_t * p_ble_cscs_c)
{
    ret_code_t err_code;

    VERIFY_PARAM_NOT_NULL(p_ble_cscs_c);

    if (p_ble_cscs_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    // initializeCalculationHelper(); //TODO: Will crash, unclear why
    startProcessing(); // Kicks off the calculators

    return cccd_configure(p_ble_cscs_c, true);
}



/** @}
 *  @endcond
 */
#endif // NRF_MODULE_ENABLED(BLE_CSCS_C)
