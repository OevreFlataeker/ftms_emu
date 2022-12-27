

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
// Provided services
#include "ble_hrs.h"
#include "ble_cps.h"
#include "ble_dis.h"
#include "ble_cscs.h"
#include "ble_bas.h"
#include "ble_ftms.h"
#include "ble_fec.h"
// #include "ble_boot_custom.h" // Not needed for Tacx!

#include "ble_atom.h"
#include "ble_uds.h"

// Backend services
#include "ble_hrs_c.h"
#include "ble_cscs_c.h"
#include "ble_bas_c.h"
#include "ble_dis_c.h"


#include "ble_conn_state.h"
#include "nrf_fstorage.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "calculations.h"
#include "counter.h"
#include "math.h"

#include "oled_controller.h"

#include "nrf_drv_gpiote.h"
#include "app_button.h"
#include "app_error.h"
#include "app_scheduler.h"

#include <ble_gap.h>
#include "helper.h"
#include "assert.h"

#include "gear_buttons.h"
#include "shimano.h"
#include "cryptoutil.h"

// S-DFU related
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "ble_dfu.h"

#include "nrf_bootloader_info.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "nrf_power.h"

#include "bond_mgr.h"

#include "nrf_strerror.h"

#define PERIPHERAL_ADVERTISING_LED      BSP_BOARD_LED_2
#define PERIPHERAL_CONNECTED_LED        BSP_BOARD_LED_3
#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1

#define SIM_MODE_ENABLED 0
// check sdk_config.h's define for NRF_LOG_ENABLED to enable/disable logging in DEBUG mode. 


#define EMULATE_TACX 1 
// Setting it to 1 will include the DIS values and adv. values from TACX
// and will establish the FE-C handling

#define BENCHMARK 0

#if EMULATE_TACX
    #define DEVICE_NAME                     "Tacx Neo"                                   /**< Name of device used for advertising. */
    #define MANUFACTURER_NAME               "Tacx"                                    /**< Manufacturer. Passed to Device Information Service. */
    #define MODEL_NUMBER                    "2875"
    #define SERIAL_NUMBER "000001"
    #define HW_REVISION "2.0"

    

#else
    #define DEVICE_NAME                     "nRF CPS"                                   /**< Name of device used for advertising. */
    #define MANUFACTURER_NAME               "Daubsi"                                    /**< Manufacturer. Passed to Device Information Service. */
    #define MODEL_NUMBER                    "1"
#endif

#define BUILD "8.9"
#define VERSION "Version: " BUILD

#define APP_ADV_INTERVAL                300                                         /**< The advertising interval (in units of 0.625 ms). This value corresponds to 187.5 ms. */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG            1                                           /**< Tag that identifies the SoftDevice BLE configuration. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to the first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define RESISTANCE_REEVALUTATION_MS        300
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size in octets. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN  

#define TRAINER_INITIAL_RESISTANCE_LEVEL 10

#define APP_TIMER_PRESCALER 0
#define APP_TIMER_OP_QUEUE_SIZE 2

/**@brief   Priority of the application BLE event handler.
 * @note    You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO           3

#define DB_DISCOVERY_INSTANCE_CNT       3  /**< Number of DB Discovery instances. */

// Controls which services we want to offer
typedef enum {
  RESISTANCE_CHANGE_NEEDED_RESULT_BUSY,
  RESISTANCE_CHANGE_NEEDED_RESULT_NO_CHANGE_NEEDED,
  RESISTANCE_CHANGE_NEEDED_RESULT_PLUS_NEEDED,
  RESISTANCE_CHANGE_NEEDED_RESULT_MINUS_NEEDED
} resistance_change_needed_t;

/* Normal mode */

bool ble_ftms_active = false;
bool ble_cps_active = true;
bool ble_cscs_active = true;
volatile bool ble_fec_active = true;
bool ble_hrs_active = true;

bool ble_dis_active = true;
bool ble_bas_active = true;
bool ble_atom_active = true;
bool ble_uds_active = false;

/* Debug mode */
/*
bool ble_ftms_active = false;
bool ble_cps_active = true;
bool ble_cscs_active = false;
bool ble_fec_active = true;
bool ble_hrs_active = false;

bool ble_dis_active = true;
bool ble_bas_active = false;
bool ble_atom_active = false;
*/


static ble_hrs_t m_hrs;                                             /**< Heart Rate Service instance. */
static ble_cps_t m_cps;                                             /**< Cycling Power Service instance. */
static ble_cscs_t m_cscs;                                           /**< Cycling Speed and Cadence Service instance. */
static ble_bas_t m_bas;                                             /**< Cycling Speed and Cadence Service instance. */
static ble_ftms_t m_ftms;                                             /**< Fitness Machine Service instance. */

static ble_atom_t m_atom;
static ble_uds_t m_uds;

// static ble_dis_t m_dis;

static ble_hrs_c_t m_hrs_c;                                         /**< Heart Rate Service client instance. */
static ble_cscs_c_t m_cscs_c;                                       /**< Cycling Speed and Cadence Service client instance. */
static ble_bas_c_t m_bas_c[2];                                      /**< Battery Service client instances. */
static ble_dis_c_t m_dis_c[2];                                      /**< Device Information client instances. */

static bool need_peer_delete = false;

NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                    /**< BLE GATT Queue instance. */
               6, // Increased from 
                  // NRF_SDH_BLE_CENTRAL_LINK_COUNT and,
               8  // NRF_BLE_GQ_QUEUE_SIZE - because my additional services might need a larger queue/size
               );
NRF_BLE_GATT_DEF(m_gatt);                                              /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);                 /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                    /**< Advertising module instance. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_discovery, 4);                         /**< Database discovery module instances. */
NRF_BLE_SCAN_DEF(m_scan);                                           /**< Scanning module instance. */

// DO NOT DO THIS!! It will cause all kind of event havoc
#if EMULATE_TACX
/******************************************************************
* Our BLE_FEC_DEF does not include the BLE_SDH_OBSERVER_MACRO!    *
* Therefore it's safe to use it like that! (see ble_fec.h) *
******************************************************************/
BLE_FEC_DEF(m_fec, NRF_SDH_BLE_TOTAL_LINK_COUNT); 
// Not needed for Tacx! BLE_BOOT_DEF(m_boot, NRF_SDH_BLE_TOTAL_LINK_COUNT);    
#endif


APP_TIMER_DEF(m_setup_hometrainer_timer_id); 

static uint16_t m_conn_handle_hrs_c  = BLE_CONN_HANDLE_INVALID;     /**< Connection handle for the HRS central application */
static uint16_t m_conn_handle_cscs_c = BLE_CONN_HANDLE_INVALID;     /**< Connection handle for the CSCP central application */
static uint16_t m_conn_handle_bas_c[2] =  { BLE_CONN_HANDLE_INVALID, BLE_CONN_HANDLE_INVALID};     /**< Connection handle for the BAS central application */
static uint16_t m_conn_handle_dis_c[2] =  { BLE_CONN_HANDLE_INVALID, BLE_CONN_HANDLE_INVALID};     /**< Connection handle for the DIS central application */
static uint16_t m_central_conn_handle = BLE_CONN_HANDLE_INVALID;

void setupHomeTrainerTimer();
void delete_bonds(void);
static void ble_dis_c_pnp_id_log(uint16_t conn_handle, ble_dis_pnp_id_t const * const p_pnp_id);
static void ble_dis_c_cert_list_log(uint16_t conn_handle, ble_dis_reg_cert_data_list_t const * const p_cert_list);
static void ble_dis_c_system_id_log(uint16_t conn_handle, ble_dis_sys_id_t const * const p_sys_id);
static void ble_dis_c_string_char_log(uint16_t conn_handle, ble_dis_c_char_type_t            char_type,
                                      ble_dis_c_string_t const * const p_string);

static uint32_t ble_dis_c_all_chars_read(uint8_t instance);
static void advertising_init(void);
/**@brief Names that the central application scans for, and that are advertised by the peripherals.
 *  If these are set to empty strings, the UUIDs defined below are used.
 */
static char const m_target_periph_name[] = "";

/**@brief UUIDs that the central application scans for if the name above is set to an empty string,
 * and that are to be advertised by the peripherals.
 */

#if EMULATE_TACX

// It's the same as in the "custom_nus" service -> Refactor. Note: UUID is in reverse order! ("6E40FEC1-B5A3-F393-E0A9-E50E24DCCA9E")
#define BLE_UUID_FEC_BRAKE_SERVICE {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC1, 0xFE, 0x40, 0x6E}} // Tacx Neo 2 specific brake service

ble_uuid128_t brake_uuid = BLE_UUID_FEC_BRAKE_SERVICE;

static ble_uuid_t m_adv_uuids[] =
{    
     {BLE_UUID_CYCLING_SPEED_AND_CADENCE, BLE_UUID_TYPE_BLE},    
     //{BLE_UUID_HEART_RATE_SERVICE,        BLE_UUID_TYPE_BLE},   
     //{BLE_UUID_CYCLING_POWER,             BLE_UUID_TYPE_BLE},    
     {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},    // Tacx Detection absolutely needs this!
    //{BLE_UUID_BATTERY_SERVICE,           BLE_UUID_TYPE_BLE},
    //{BLE_UUID_FITNESS_MACHINE,           BLE_UUID_TYPE_BLE}
    
};

static ble_uuid_t m_adv_sr_uuids[] =
{
    { BLE_UUID_TACX_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN},     
       // // Tacx Detection absolutely needs this!    
    
};

// Tacx also needs the Secure DFU from Nordic names "TXN2_DFU"

#else
static ble_uuid_t m_adv_uuids[] =
{        
    {BLE_UUID_HEART_RATE_SERVICE,        BLE_UUID_TYPE_BLE},   
    {BLE_UUID_CYCLING_POWER,             BLE_UUID_TYPE_BLE},    
   
    //{BLE_UUID_BATTERY_SERVICE,           BLE_UUID_TYPE_BLE},
    {BLE_UUID_FITNESS_MACHINE,           BLE_UUID_TYPE_BLE}
};
#endif

//#define CPS_SERVICE_UUID_ADV_IDX            0                                           /**< CPS service UUID index in array. */
//#define CSCS_SERVICE_UUID_ADV_IDX           1                                           /**< CSCS service UUID index in array. */
//#define HART_RATE_SERVICE_UUID_ADV_IDX      1                                           /**< Hart Rate service UUID index in array. */
//#define BATTERY_SERVICE_UUID_ADV_IDX        3                                           /**< Battery Service UUID index in array. */
//#define FTMS_SERVICE_UUID_ADV_IDX           4                                           /**< Fitness Machine Service UUID index in array. */
//#define FTMS_SERVICE_UUID_ADV_IDX           2                                           /**< Fitness Machine Service UUID index in array. */


static ble_uuid_t m_scan_uuids[] =
{
    {BLE_UUID_CYCLING_SPEED_AND_CADENCE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_HEART_RATE_SERVICE,        BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,           BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,BLE_UUID_TYPE_BLE}
};

#define CSCS_SERVICE_UUID_SCAN_IDX           0                                           /**< CSCS service UUID index in array. */
#define HART_RATE_SERVICE_UUID_SCAN_IDX      1                                           /**< Hart Rate service UUID index in array. */
#define BATTERY_SERVICE_UUID_SCAN_IDX        2    // Do we need this?                    /**< Battery Service UUID index in arrya. */
#define DEVICE_INFORMATION_SERVICE_UUID_SCAN_IDX 3 // Do we need this?

static ble_gap_scan_params_t m_scan_param =                 /**< Scan parameters requested for scanning and connection. */
{
    .active        = 0x01,
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = NRF_BLE_SCAN_SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .extended      = true,
};

static ble_sensor_location_t supported_locations[] =                                /**< Supported location for the sensor location. */
{
    BLE_SENSOR_LOCATION_FRONT_WHEEL,
    BLE_SENSOR_LOCATION_LEFT_CRANK,
    BLE_SENSOR_LOCATION_RIGHT_CRANK,
    BLE_SENSOR_LOCATION_LEFT_PEDAL,
    BLE_SENSOR_LOCATION_RIGHT_PEDAL,
    BLE_SENSOR_LOCATION_FRONT_HUB,
    BLE_SENSOR_LOCATION_REAR_DROPOUT,
    BLE_SENSOR_LOCATION_CHAINSTAY,
    BLE_SENSOR_LOCATION_REAR_WHEEL,
    BLE_SENSOR_LOCATION_REAR_HUB
};

/**@brief Strings to display data retrieved by Device Information Service client module. */
static char const * const m_dis_char_names[] =
{
    "Manufacturer Name String",
    "Model Number String     ",
    "Serial Number String    ",
    "Hardware Revision String",
    "Firmware Revision String",
    "Software Revision String",
    "System ID",
    "IEEE 11073-20601 Regulatory Certification Data List",
    "PnP ID"
};


volatile uint8_t resistance_level = 1;
volatile uint8_t target_resistance_level = 1;
volatile int8_t gear_offset = 0;

volatile bool requireResistanceMinus;
volatile bool requireResistancePlus;
volatile bool doCalcResistance;
volatile bool is_advertising = false;

extern oled_data_t oled_data;
extern volatile bool doUpdateOLED;
extern volatile bool doInjectSimEvent;
extern volatile bool doTriggerFECEvent;
extern volatile bool doCalcModel;
extern volatile bool doSteer;
extern volatile bool gpio_busy;
extern volatile bool gpio_finished;
extern volatile gear_button_state_t gear_button_state;
/*
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch (pin)
    {
        case TRAINER_BUTTON_INCLINE_UP:
            NRF_LOG_INFO("Set incline up!");

        break;
        case TRAINER_BUTTON_INCLINE_DOWN:
            NRF_LOG_INFO("Set incline down!");
        break;
        default:
        break;
    }
    // NRF_LOG_FLUSH();
}
*/
/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling the Heart Rate Service Client
 *        Cycling Speed and Cadence Service Client.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    ble_gap_evt_adv_report_t const * p_adv = 
                   p_scan_evt->params.filter_match.p_adv_report;
    ble_gap_scan_params_t    const * p_scan_param = 
                   p_scan_evt->p_scan_params;
                   p_scan_evt->p_scan_params;
    
    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
        {
            // Initiate connection.
            err_code = sd_ble_gap_connect(&p_adv->peer_addr,
                                          p_scan_param,
                                          &m_scan.conn_params,
                                          APP_BLE_CONN_CFG_TAG);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}

static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}


static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}


//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}



static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};


// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}


/**@brief Function for initialization the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.p_scan_param = &m_scan_param;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    if (strlen(m_target_periph_name) != 0)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan, 
                                           SCAN_NAME_FILTER, 
                                           m_target_periph_name);
        APP_ERROR_CHECK(err_code);
    }

    
    err_code = nrf_ble_scan_filter_set(&m_scan, 
                                       SCAN_UUID_FILTER, 
                                       &m_scan_uuids[CSCS_SERVICE_UUID_SCAN_IDX]);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, 
                                       SCAN_UUID_FILTER, 
                                       &m_scan_uuids[HART_RATE_SERVICE_UUID_SCAN_IDX]);
    APP_ERROR_CHECK(err_code);
    
   

    /*
    // Would need to increase NRF_BLE_SCAN_UUID_CNT in sdk_config.h here!
    err_code = nrf_ble_scan_filter_set(&m_scan, 
                                       SCAN_UUID_FILTER, 
                                       &m_scan_uuids[BATTERY_SERVICE_UUID_IDX]);
    APP_ERROR_CHECK(err_code);
    */

    err_code = nrf_ble_scan_filters_enable(&m_scan, 
                                           NRF_BLE_SCAN_ALL_FILTER, 
                                           false);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;    
    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the advertising and the scanning.
 */
static void adv_scan_start(void)
{
    ret_code_t err_code;

    //check if there are no flash operations in progress
    if (!nrf_fstorage_is_busy(NULL))
    {
        // Start scanning for peripherals and initiate connection to devices which
        // advertise Heart Rate or Cycling speed and cadence UUIDs.
        scan_start();

        // Turn on the LED to signal scanning.
        bsp_board_led_on(CENTRAL_SCANNING_LED);

        // Start advertising.
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
ret_code_t err_code;
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            need_peer_delete = false;
            //adv_scan_start();
            // Deletion succeeded! Remove it from the deletion table.
            //remove_peer_from_deletion_table(p_evt->peer_id);
            NRF_LOG_INFO("PM_EVT_PEERS_DELETE_SUCCEEDED received");
            break;
        }

        case PM_EVT_PEER_DELETE_SUCCEEDED:
        {
            need_peer_delete = false;
            //adv_scan_start();
            // Deletion succeeded! Remove it from the deletion table.
            //remove_peer_from_deletion_table(p_evt->peer_id);
            NRF_LOG_INFO("PM_EVT_PEER_DELETE_SUCCEEDED received: Peer-ID: %u", p_evt->peer_id);
            // Try to re-eanble adv and scan
            scan_init();
            advertising_init();

            break;
        }
        case PM_EVT_PEER_DELETE_FAILED:
        {
        // Again, for this design, if we fail to delete the peer,
           // were giving up and removing it from the deletion table,
           // but your application may need to handle it differently.
            //remove_peer_from_deletion_table(p_evt->peer_id);
            NRF_LOG_INFO("PM_EVT_PEER_DELETE_FAILED received: Peer-ID: %u", p_evt->peer_id);
        }
        case PM_EVT_PEERS_DELETE_FAILED:
        {
        // Again, for this design, if we fail to delete the peer,
           // were giving up and removing it from the deletion table,
           // but your application may need to handle it differently.
            //remove_peer_from_deletion_table(p_evt->peer_id);
            NRF_LOG_INFO("PM_EVT_PEERS_DELETE_FAILED received");
        }
        break;
        case PM_EVT_CONN_SEC_FAILED:
        {
            //NRF_LOG_INFO("Establishing a secure link by Peer Manager failed! Please press Button 2 + RESET on the DK to delete bonding data");            
            pm_conn_secure_failed_evt_t e = p_evt->params.conn_sec_failed;
            pm_sec_error_code_t error           = e.error;
            uint8_t error_src                   = e.error_src;
            pm_conn_sec_procedure_t procedure   = e.procedure;
            switch (procedure)
            {
                case PM_CONN_SEC_PROCEDURE_ENCRYPTION:
                    NRF_LOG_INFO("PM_CONN_SEC_PROCEDURE_ENCRYPTION  procedure failed");
                    break;
                case PM_CONN_SEC_PROCEDURE_BONDING:
                    NRF_LOG_INFO("PM_CONN_SEC_PROCEDURE_BONDING  procedure failed");
                    break;
                case PM_CONN_SEC_PROCEDURE_PAIRING:
                    NRF_LOG_INFO("PM_CONN_SEC_PROCEDURE_PAIRING  procedure failed");
                    break;
                default:
                    NRF_LOG_INFO("Unknown peer manager procedure failed");
                    break;
            }
            switch (error)
            {
                case PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING:
                    NRF_LOG_INFO("Error PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING");
                    break;
                case PM_CONN_SEC_ERROR_MIC_FAILURE:
                    NRF_LOG_INFO("Error PM_CONN_SEC_ERROR_MIC_FAILURE");
                    break;
                case PM_CONN_SEC_ERROR_DISCONNECT:
                    NRF_LOG_INFO("Error PM_CONN_SEC_ERROR_DISCONNECT");
                    break;
                case PM_CONN_SEC_ERROR_SMP_TIMEOUT :
                     NRF_LOG_INFO("Error PM_CONN_SEC_ERROR_SMP_TIMEOUT");
                    break;
                default:
                    NRF_LOG_INFO("Unknown error");
                    break;
            }
            NRF_LOG_INFO("Establishing a secure link by Peer Manager failed! Trying to remove bonds and rescan");
            err_code = bond_mgr_queue_peer_for_deletion(p_evt->peer_id);
            APP_ERROR_CHECK(err_code);
            need_peer_delete = true;
            //delete_bonds();
            break;
            }
        case PM_EVT_CONN_SEC_CONFIG_REQ:
          {
            // Allow pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};            
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
          }break;
        default:
            break;
    }
}


/**@brief Function for changing filter settings after establishing the connection.
 */
static void filter_settings_change(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_all_filter_remove(&m_scan);
    APP_ERROR_CHECK(err_code);

    if (strlen(m_target_periph_name) != 0)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan, 
                                           SCAN_NAME_FILTER, 
                                           m_target_periph_name);
        APP_ERROR_CHECK(err_code);
    }

    if (m_conn_handle_cscs_c == BLE_CONN_HANDLE_INVALID)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan, 
                                           SCAN_UUID_FILTER, 
                                           &m_scan_uuids[CSCS_SERVICE_UUID_SCAN_IDX]);
    }
    /*else if ((m_conn_handle_cscs_c != BLE_CONN_HANDLE_INVALID) && 
         (m_conn_handle_bas_c == BLE_CONN_HANDLE_INVALID))
    {
        err_code = nrf_ble_scan_filter_set(&m_scan, 
                                           SCAN_UUID_FILTER, 
                                           &m_scan_uuids[BATTERY_SERVICE_UUID_IDX]);
    }*/
    else if ((m_conn_handle_cscs_c != BLE_CONN_HANDLE_INVALID) && 
       //  (m_conn_handle_bas_c != BLE_CONN_HANDLE_INVALID) &&
         (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID))
    {
        err_code = nrf_ble_scan_filter_set(&m_scan, 
                                           SCAN_UUID_FILTER, 
                                           &m_scan_uuids[HART_RATE_SERVICE_UUID_SCAN_IDX]);
    }
    

    APP_ERROR_CHECK(err_code);
}


/**@brief Handles events coming from the Heart Rate central module.
 */
 
static void hrs_c_evt_handler(ble_hrs_c_t * p_hrs_c, ble_hrs_c_evt_t * p_hrs_c_evt)
{
    switch (p_hrs_c_evt->evt_type)
    {
        case BLE_HRS_C_EVT_DISCOVERY_COMPLETE:
        {
            if (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID)
            {
                ret_code_t err_code;

                m_conn_handle_hrs_c = p_hrs_c_evt->conn_handle;
                NRF_LOG_INFO("HRS discovered on conn_handle 0x%x", m_conn_handle_hrs_c);

                filter_settings_change();

                err_code = ble_hrs_c_handles_assign(p_hrs_c,
                                                    m_conn_handle_hrs_c,
                                                    &p_hrs_c_evt->params.peer_db);
                APP_ERROR_CHECK(err_code);
                // Initiate bonding.
                err_code = pm_conn_secure(m_conn_handle_hrs_c, false);
                if (err_code != NRF_ERROR_BUSY)
                {
                    APP_ERROR_CHECK(err_code);
                }

                // Heart rate service discovered. Enable notification of Heart Rate Measurement.
                err_code = ble_hrs_c_hrm_notif_enable(p_hrs_c);
                APP_ERROR_CHECK(err_code);
            }
        } break; // BLE_HRS_C_EVT_DISCOVERY_COMPLETE

        case BLE_HRS_C_EVT_HRM_NOTIFICATION:
        {
            ret_code_t err_code;

            // NRF_LOG_INFO("Heart Rate = %d", p_hrs_c_evt->params.hrm.hr_value);

            err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, p_hrs_c_evt->params.hrm.hr_value);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
            {
                APP_ERROR_HANDLER(err_code);
            }
        } break; // BLE_HRS_C_EVT_HRM_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Handles events coming from the Battery central module.
 */
 
static void bas_c_evt_handler(ble_bas_c_t * p_bas_c, ble_bas_c_evt_t * p_bas_c_evt)
{
    switch (p_bas_c_evt->evt_type)
    {
        case BLE_BAS_C_EVT_DISCOVERY_COMPLETE:
        {
            if (m_conn_handle_bas_c[0] == BLE_CONN_HANDLE_INVALID)
            {
                ret_code_t err_code;

                m_conn_handle_bas_c[0] = p_bas_c_evt->conn_handle;
                NRF_LOG_INFO("BAS 1 discovered on conn_handle 0x%x", m_conn_handle_bas_c[0]);

                filter_settings_change();

                err_code = ble_bas_c_handles_assign(&p_bas_c[0],
                                                    m_conn_handle_bas_c[0],
                                                    &p_bas_c_evt->params.bas_db);
                APP_ERROR_CHECK(err_code);
		
		NRF_LOG_INFO("Battery Service 1 discovered. Reading battery level.");

	        err_code = ble_bas_c_bl_read(&p_bas_c[0]);
	        APP_ERROR_CHECK(err_code);
        
		// Battery service discovered. Enable notification of Battery Measurement.
        	//NRF_LOG_INFO("Enabling Battery Level Notification.");
		//err_code = ble_bas_c_bl_notif_enable(p_bas_c);
                //APP_ERROR_CHECK(err_code);
            }
            else if (m_conn_handle_bas_c[1] == BLE_CONN_HANDLE_INVALID)
            {
                ret_code_t err_code;

                m_conn_handle_bas_c[1] = p_bas_c_evt->conn_handle;
                NRF_LOG_INFO("BAS 2 discovered on conn_handle 0x%x", m_conn_handle_bas_c[1]);

                filter_settings_change();

                err_code = ble_bas_c_handles_assign(&p_bas_c[1],
                                                    m_conn_handle_bas_c[1],
                                                    &p_bas_c_evt->params.bas_db);
                APP_ERROR_CHECK(err_code);
		
		NRF_LOG_INFO("Battery Service 2 discovered. Reading battery level.");

	        err_code = ble_bas_c_bl_read(&p_bas_c[1]);
	        APP_ERROR_CHECK(err_code);
        
		// Battery service discovered. Enable notification of Battery Measurement.
        	//NRF_LOG_INFO("Enabling Battery Level Notification.");
		//err_code = ble_bas_c_bl_notif_enable(p_bas_c);
                //APP_ERROR_CHECK(err_code);
            }
        } break; // BLE_BAS_C_EVT_DISCOVERY_COMPLETE

        case BLE_BAS_C_EVT_BATT_NOTIFICATION:
        {
            ret_code_t err_code;
            // We can only report one battery!
            NRF_LOG_INFO("Battery = %d", p_bas_c_evt->params.battery_level);
            
            err_code = ble_bas_battery_level_update(&m_bas, p_bas_c_evt->params.battery_level,BLE_CONN_HANDLE_ALL);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
            {
                APP_ERROR_HANDLER(err_code);
            }
        } break; // BLE_BAS_C_EVT_BAM_NOTIFICATION
        
        case BLE_BAS_C_EVT_BATT_READ_RESP:
            NRF_LOG_INFO("Battery Level Read as %d %%.", p_bas_c_evt->params.battery_level);
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for reading all characteristics that can be present in DIS.
 *
 * @return    The number of discovered characteristics.
 */
static uint32_t ble_dis_c_all_chars_read(uint8_t instance)
{
    ret_code_t err_code;
    uint32_t   disc_char_num = 0;

    for (ble_dis_c_char_type_t char_type = (ble_dis_c_char_type_t) 0;
         char_type < BLE_DIS_C_CHAR_TYPES_NUM;
         char_type++)
    {
        err_code = ble_dis_c_read(&m_dis_c[instance], char_type);

        // The NRF_ERROR_INVALID_STATE error code means that the characteristic is not present in DIS.
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
            disc_char_num++;
        }
    }

    return disc_char_num;
}



/**@brief Handles events coming from the Battery central module.
 */

static void dis_c_evt_handler(ble_dis_c_t * p_ble_dis_c, ble_dis_c_evt_t const * p_ble_dis_evt)
{
    ret_code_t      err_code;
    static uint32_t disc_chars_num     = 0;
    static uint32_t disc_chars_handled = 0;    

    switch (p_ble_dis_evt->evt_type)
    {
        case BLE_DIS_C_EVT_DISCOVERY_COMPLETE:
        {
            if (m_conn_handle_dis_c[0] == BLE_CONN_HANDLE_INVALID)
            {
              err_code = ble_dis_c_handles_assign(p_ble_dis_c,
                                                  p_ble_dis_evt->conn_handle,
                                                  p_ble_dis_evt->params.disc_complete.handles);
              APP_ERROR_CHECK(err_code);
              m_conn_handle_dis_c[0] = p_ble_dis_evt->conn_handle;
              disc_chars_num     = ble_dis_c_all_chars_read(m_conn_handle_dis_c[0]);
              disc_chars_handled = 0;
              NRF_LOG_INFO("Device Information Service 1 discovered.");
            }
            else if (m_conn_handle_dis_c[1] == BLE_CONN_HANDLE_INVALID)
            {
              err_code = ble_dis_c_handles_assign(p_ble_dis_c,
                                                  p_ble_dis_evt->conn_handle,
                                                  p_ble_dis_evt->params.disc_complete.handles);
              APP_ERROR_CHECK(err_code);
              m_conn_handle_dis_c[1] = p_ble_dis_evt->conn_handle;
              disc_chars_num     = ble_dis_c_all_chars_read(m_conn_handle_dis_c[1]);
              disc_chars_handled = 0;
              NRF_LOG_INFO("Device Information Service 2 discovered.");
        
            }
        } break; // BLE_BAS_C_EVT_DISCOVERY_COMPLETE
        
        case BLE_DIS_C_EVT_DIS_C_READ_RSP:
        {    
            ble_dis_c_evt_read_rsp_t const * p_read_rsp = &p_ble_dis_evt->params.read_rsp;

            //Print header log.
            if ((disc_chars_handled == 0) && (disc_chars_num != 0))
            {
                if (m_conn_handle_dis_c[0] != BLE_CONN_HANDLE_INVALID && m_conn_handle_dis_c[0] == p_ble_dis_evt->conn_handle)
                {
                    NRF_LOG_INFO("");
                    NRF_LOG_INFO("Device Information of first connected device:");
                }
                else if (m_conn_handle_dis_c[1] != BLE_CONN_HANDLE_INVALID && m_conn_handle_dis_c[1] == p_ble_dis_evt->conn_handle)
                {
                    NRF_LOG_INFO("");
                    NRF_LOG_INFO("Device Information of second connected device:");
                }

            }

            switch (p_read_rsp->char_type)
            {
                case BLE_DIS_C_MANUF_NAME:
                case BLE_DIS_C_MODEL_NUM:
                case BLE_DIS_C_SERIAL_NUM:
                case BLE_DIS_C_HW_REV:
                case BLE_DIS_C_FW_REV:
                case BLE_DIS_C_SW_REV:
                    ble_dis_c_string_char_log(p_ble_dis_evt->conn_handle, p_read_rsp->char_type, &p_read_rsp->content.string);
                    break;

                case BLE_DIS_C_SYS_ID:
                    ble_dis_c_system_id_log(p_ble_dis_evt->conn_handle, &p_read_rsp->content.sys_id);
                    break;

                case BLE_DIS_C_CERT_LIST:
                    ble_dis_c_cert_list_log(p_ble_dis_evt->conn_handle, &p_read_rsp->content.cert_list);
                    break;

                case BLE_DIS_C_PNP_ID:
                    ble_dis_c_pnp_id_log(p_ble_dis_evt->conn_handle, &p_read_rsp->content.pnp_id);
                    break;

                default:
                    break;
            }

            disc_chars_handled++;
            if(disc_chars_handled == disc_chars_num)
            {
                NRF_LOG_INFO("");
                disc_chars_handled = 0;
                disc_chars_num     = 0;
            }
         }
         break;
        case BLE_DIS_C_EVT_DIS_C_READ_RSP_ERROR:
        {
        NRF_LOG_ERROR("Read request for: %s characteristic failed with gatt_status: 0x%04X.",
                      m_dis_char_names[p_ble_dis_evt->params.read_rsp.char_type],
                      p_ble_dis_evt->params.read_rsp_err.gatt_status);
        break;
          }
          
        case BLE_DIS_C_EVT_DISCONNECTED:
            break;

    }
}

/**@brief Function for logging string characteristics that can be present in Device Information Service.
 *
 * @param[in] char_type Type of string characteristic.
 * @param[in] p_string  Response data of the characteristic that has been read.
 */
static void ble_dis_c_string_char_log(uint16_t conn_handle, 
                                      ble_dis_c_char_type_t            char_type,
                                      ble_dis_c_string_t const * const p_string)
{
    char response_data_string[BLE_DIS_C_STRING_MAX_LEN] = {0};

    if (sizeof(response_data_string) > p_string->len)
    {
        int deviceid = -1;
        memcpy(response_data_string, p_string->p_data, p_string->len);
        if (conn_handle == m_conn_handle_dis_c[0])
        {
            deviceid = 1;
        }
        else if (conn_handle == m_conn_handle_dis_c[1])
        {
            deviceid = 2;
        }

        NRF_LOG_INFO("Device %d: %s: %s",
                     deviceid,
                     m_dis_char_names[char_type],
                     nrf_log_push((char *) response_data_string));
    }
    else
    {
        NRF_LOG_ERROR("String buffer for DIS characteristics is too short.")
    }
}


/**@brief Function for logging PnP ID characteristic data that can be present in Device Information Service.
 *
 * @param[in] p_pnp_id Pointer to structure that describes the content of PnP ID characteristic.
 */
static void ble_dis_c_pnp_id_log(uint16_t conn_handle, ble_dis_pnp_id_t const * const p_pnp_id)
{
    NRF_LOG_INFO("%s:", m_dis_char_names[BLE_DIS_C_PNP_ID]);
    NRF_LOG_INFO(" Vendor ID Source: 0x%02X", p_pnp_id->vendor_id_source);
    NRF_LOG_INFO(" Vendor ID:        0x%04X", p_pnp_id->vendor_id);
    NRF_LOG_INFO(" Product ID:       0x%04X", p_pnp_id->product_id);
    NRF_LOG_INFO(" Product Version:  0x%04X", p_pnp_id->product_version);
}

static void timer_resistance_handler(void *p_context)
{
    if (!doCalcResistance)
    {
        doCalcResistance = true;
    }
}

static resistance_change_needed_t checkResistanceChangeNeeded()
{
    //uint8_t final_resistance_level = 
    //      (int8_t) ((int8_t) target_resistance_level + 2*gear_offset) < 1 ? 1 : (int8_t) target_resistance_level + 2*gear_offset;
    
    // TODO: Experiment: Does a gear shift not impose new resistance but is it covered by the FE-C formulas??
//    uint8_t final_resistance_level =
  //          (int8_t) ((int8_t) target_resistance_level + 0*gear_offset) < 1 ? 1 : (int8_t) target_resistance_level + 0*gear_offset;
    

    if (gpio_busy)
    {
        return RESISTANCE_CHANGE_NEEDED_RESULT_BUSY;
    }
    uint8_t final_resistance_level = target_resistance_level;

    

    if ((target_resistance_level < 1) || (target_resistance_level > 32))
    {
      uint8_t trl = target_resistance_level;
      NRF_LOG_ERROR("Error invalid target_resistance_level: %d", target_resistance_level);
      stopOLEDUpdates();          
      
      char buf[32];
      sprintf(buf, "TR ASSERTION: %u", trl);
      oled_printStringAt(30,10, buf, true, true);
      assert(true==false);
    }

    if (final_resistance_level > NUM_RESISTANCE_LEVELS)
    {
      final_resistance_level = NUM_RESISTANCE_LEVELS;
    }

    
    if (resistance_level>=1 && resistance_level<=NUM_RESISTANCE_LEVELS && resistance_level!=final_resistance_level)
    {   
        // NRF_LOG_INFO("Resistance level change needed: Current = %u, Should %u (Target: %u + Gear offset: %d)", resistance_level, final_resistance_level, target_resistance_level, gear_offset);
        if (resistance_level < final_resistance_level) 
        {
            requireResistancePlus = true;
            // triggerResistancePlus();
            // resistance_level++;
        }
        else
        {
            requireResistanceMinus = true;
            // triggerResistanceMinus();
            // resistance_level--;
        }                
    }
    else
    {
        requireResistancePlus = false;
        requireResistanceMinus = false;

        //NRF_LOG_INFO("Initial incline %d reached. Stopping timer", incline_level);
        // NRF_LOG_INFO("No change of incline needed (current incline = %d", incline_level);
        //app_timer_stop(m_setup_hometrainer_timer_id);
    }
    
    oled_data.sim_resistance = target_resistance_level;

    if (requireResistancePlus) return RESISTANCE_CHANGE_NEEDED_RESULT_PLUS_NEEDED;
    if (requireResistanceMinus) return RESISTANCE_CHANGE_NEEDED_RESULT_MINUS_NEEDED;
    return RESISTANCE_CHANGE_NEEDED_RESULT_NO_CHANGE_NEEDED;
}


/**@brief Handles events coming from  Cycling Speed and Cadence central module.
 */
static void cscs_c_evt_handler(ble_cscs_c_t * p_cscs_c, ble_cscs_c_evt_t * p_cscs_c_evt)
{
    switch (p_cscs_c_evt->evt_type)
    {        
        case BLE_CSCS_C_EVT_DISCOVERY_COMPLETE:
        {
            if (m_conn_handle_cscs_c == BLE_CONN_HANDLE_INVALID)
            {
                ret_code_t err_code;

                m_conn_handle_cscs_c = p_cscs_c_evt->conn_handle;
                NRF_LOG_INFO("Cycling Speed and Cadence service discovered on conn_handle 0x%x",
                             m_conn_handle_cscs_c);

                filter_settings_change();                
                err_code = ble_cscs_c_handles_assign(p_cscs_c,
                                                    m_conn_handle_cscs_c,
                                                    &p_cscs_c_evt->params.cscs_db);
                APP_ERROR_CHECK(err_code);
                
                // Initiate bonding.
                err_code = pm_conn_secure(m_conn_handle_cscs_c, false);
                
                if (err_code != NRF_ERROR_BUSY)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                // Cycling Speed Cadence Service discovered. Enable notifications.
                err_code = ble_cscs_c_csc_notif_enable(p_cscs_c);
                                
                APP_ERROR_CHECK(err_code);

                // Setting of the initialze of the difficulty to 10
                NRF_LOG_INFO("Setting of timer to initalize the trainer in 2000ms");
                target_resistance_level = TRAINER_INITIAL_RESISTANCE_LEVEL;
                setupHomeTrainerTimer();                

            }
        } break; // BLE_CSCS_C_EVT_DISCOVERY_COMPLETE:
        
        case BLE_CSCS_C_EVT_CSC_NOTIFICATION:
        {
            ret_code_t      err_code;
            // ble_cps_meas_t  cps_measurment;
            ble_cscs_meas_t  cscs_measurment;            


            // NRF_LOG_INFO("Speed      = %d", p_cscs_c_evt->params.csc.inst_speed);
            // NRF_LOG_INFO("Proxying measurements!");

            /*
            cps_measurment.cumulative_wheel_revs      = p_cscs_c_evt->params.csc.cumulative_wheel_revs;
            cps_measurment.last_wheel_event_time      = p_cscs_c_evt->params.csc.last_wheel_event_time;
            cps_measurment.cumulative_crank_revs      = p_cscs_c_evt->params.csc.cumulative_crank_revs;
            cps_measurment.last_crank_event_time      = p_cscs_c_evt->params.csc.last_crank_event_time;
            cps_measurment.is_wheel_rev_data_present  = p_cscs_c_evt->params.csc.is_wheel_rev_data_present;
            cps_measurment.is_crank_rev_data_present  = p_cscs_c_evt->params.csc.is_crank_rev_data_present;
            */

            // Fill our CSCS measurement as well!
            cscs_measurment.cumulative_wheel_revs      = getCumulativeWheelRevs(); //p_cscs_c_evt->params.csc.cumulative_wheel_revs;
            cscs_measurment.last_wheel_event_time      = getLastWheelEventTime(); // p_cscs_c_evt->params.csc.last_wheel_event_time;
            cscs_measurment.cumulative_crank_revs      = getCumulativeCrankRevs(); //p_cscs_c_evt->params.csc.cumulative_crank_revs;
            cscs_measurment.last_crank_event_time      = getLastCrankEventTime(); // p_cscs_c_evt->params.csc.last_crank_event_time;
            cscs_measurment.is_wheel_rev_data_present  = p_cscs_c_evt->params.csc.is_wheel_rev_data_present;
            cscs_measurment.is_crank_rev_data_present  = p_cscs_c_evt->params.csc.is_crank_rev_data_present;
            
            
            /*
            // Prepare calculation basis
            if (cps_measurment.cumulative_crank_revs != cscs_calculation_helper.cumulative_crank_revs)
            {
                cscs_calculation_helper.prev_cumulative_crank_revs = cscs_calculation_helper.cumulative_crank_revs;
                cscs_calculation_helper.cumulative_crank_revs = cps_measurment.cumulative_crank_revs;

                cscs_calculation_helper.prev_last_crank_event_time = cscs_calculation_helper.last_crank_event_time;
                cscs_calculation_helper.last_crank_event_time = cps_measurment.last_crank_event_time;

                cscs_calculation_helper.zPower = 0.0;                
                cscs_calculation_helper.i_cadence = cps_calc_instantaneous_cadence();
                
                // char s_i_cadence[16];
                // sprintf(s_i_cadence, "%f", cscs_calculation_helper.i_cadence);                
                // NRF_LOG_INFO("i_cadence: %s", s_i_cadence);

                cscs_calculation_helper.a_cadence = cps_calc_average_cadence(cscs_calculation_helper.i_cadence);
                // char s_a_cadence[16];
                // sprintf(s_a_cadence, "%f", cscs_calculation_helper.a_cadence);
                // NRF_LOG_INFO("a_cadence: %s", s_a_cadence);

                if (cscs_calculation_helper.a_cadence >= 80.0)
                {
                    cscs_calculation_helper.zPower = 178.0 + (cscs_calculation_helper.a_cadence - 80) * 3.0;
                }
                else
                {
                    cscs_calculation_helper.zPower = 178.0 - (80 - cscs_calculation_helper.a_cadence) * 3.0;
                }
                if (cscs_calculation_helper.zPower < 0.0)
                {
                    cscs_calculation_helper.zPower = 0;
                }
                char s_zpower[16];
                sprintf(s_zpower, "%f", cscs_calculation_helper.zPower);
                NRF_LOG_INFO("zPower: %s", s_zpower);
                cps_measurment.instantaneous_power = (int16_t) round(cscs_calculation_helper.zPower);
            }
            

            
            NRF_LOG_INFO("Cumulative wheel revs: %d", cscs_measurment.cumulative_wheel_revs);
            NRF_LOG_INFO("Last wheel event time: %d", cscs_measurment.last_wheel_event_time);
            NRF_LOG_INFO("Cumulative crank revs: %d", cscs_measurment.cumulative_crank_revs);
            NRF_LOG_INFO("Last crank event time: %d", cscs_measurment.last_crank_event_time);
            NRF_LOG_INFO("Wheel data present: %d", cscs_measurment.is_wheel_rev_data_present);
            NRF_LOG_INFO("Crank data present: %d", cscs_measurment.is_crank_rev_data_present);
            
            err_code = ble_cps_measurement_send(&m_cps, &cps_measurment);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
            {
                 APP_ERROR_HANDLER(err_code);
            }
            */

            if (ble_cps_active)
            {
                ble_cps_on_cscs_evt(&m_cps, p_cscs_c_evt); // Dispatch to cps calculations
            }

            if (ble_ftms_active)
            {
                ble_ftms_on_cscs_evt(&m_ftms, p_cscs_c_evt);
            }

            if (ble_fec_active)
            {
                ble_fec_on_cscs_evt(&m_fec, p_cscs_c_evt);
            }
            

            // Update our CSCS service at the same time!
            if (ble_cscs_active)
            {
                err_code = ble_cscs_measurement_send(&m_cscs, &cscs_measurment);
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
        } break; // BLE_CSCS_C_EVT_CSP_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}

void setupHomeTrainerTimer()
{
    ret_code_t err_code;
    err_code = app_timer_create(&m_setup_hometrainer_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                //checkResistanceChangeNeeded);
                                timer_resistance_handler);
                APP_ERROR_CHECK(err_code);
                err_code = app_timer_start(m_setup_hometrainer_timer_id, APP_TIMER_TICKS(RESISTANCE_REEVALUTATION_MS), NULL);
                //err_code = app_timer_start(m_setup_hometrainer_timer_id, APP_TIMER_TICKS(1000), NULL);
                APP_ERROR_CHECK(err_code);
}

/**@brief Function for assigning new connection handle to available instance of QWR module.
 *
 * @param[in] conn_handle New connection handle.
 */
static void multi_qwr_conn_handle_assign(uint16_t conn_handle)
{
    for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }
}


/**@brief Function for logging System ID characteristic data that can be present in Device Information Service.
 *
 * @param[in] p_sys_id Pointer to structure that describes the content of the System ID characteristic.
 */
static void ble_dis_c_system_id_log(uint16_t conn_handle, ble_dis_sys_id_t const * const p_sys_id)
{
    NRF_LOG_INFO("%s:", m_dis_char_names[BLE_DIS_C_SYS_ID]);
    NRF_LOG_INFO(" Manufacturer Identifier:            0x%010X", p_sys_id->manufacturer_id);
    NRF_LOG_INFO(" Organizationally Unique Identifier: 0x%06X", p_sys_id->organizationally_unique_id);
}


/**@brief Function for logging IEEE 11073-20601 Regulatory Certification Data List characteristic
 *        data that can be present in DIS.
 *
 * @param[in] p_cert_list  Pointer to structure that describes the content of Certification Data List
 *                         characteristic.
 */
static void ble_dis_c_cert_list_log(uint16_t conn_handle, ble_dis_reg_cert_data_list_t const * const p_cert_list)
{
    NRF_LOG_INFO("%s:", m_dis_char_names[BLE_DIS_C_CERT_LIST]);
    NRF_LOG_HEXDUMP_INFO(p_cert_list->p_list, p_cert_list->list_len);
}

/**@brief   Function for handling BLE events from the central application.
 *
 * @details This function parses scanning reports and initiates a connection to peripherals when a
 *          target UUID is found. It updates the status of LEDs used to report the central application
 *          activity.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral is connected (HR or RSC), initiate DB
        // discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("We connected to peripheral");
            // If no Heart Rate sensor or CSCS sensor or Battery sensor is currently connected, try to find them on this peripheral.
            if (   (m_conn_handle_hrs_c  == BLE_CONN_HANDLE_INVALID)
                || (m_conn_handle_cscs_c == BLE_CONN_HANDLE_INVALID)
                //|| (m_conn_handle_bas_c == BLE_CONN_HANDLE_INVALID)
                )
            {
                NRF_LOG_INFO("Attempt to find peripheral services on conn_handle 0x%x", p_gap_evt->conn_handle);
                // m_central_conn_handle = p_gap_evt->conn_handle; // This is not our connection handle!
                err_code = ble_db_discovery_start(&m_db_discovery[0], p_gap_evt->conn_handle);
                if (err_code == NRF_ERROR_BUSY)
                {
                    err_code = ble_db_discovery_start(&m_db_discovery[1], p_gap_evt->conn_handle);
                    APP_ERROR_CHECK(err_code);
                
               		 if (err_code == NRF_ERROR_BUSY)
                	{
                    		err_code = ble_db_discovery_start(&m_db_discovery[2], p_gap_evt->conn_handle);
                    	APP_ERROR_CHECK(err_code);
                	}
                	else
                	{
                    APP_ERROR_CHECK(err_code);
                	}
		}
		else
		{
			APP_ERROR_CHECK(err_code);
		}
                
            }

            // Assign connection handle to the QWR module.
            multi_qwr_conn_handle_assign(p_gap_evt->conn_handle);

            // Update LEDs status, and check whether to look for more peripherals to connect to.
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                bsp_board_led_off(CENTRAL_SCANNING_LED);
            }
            else
            {
                // Resume scanning.
                bsp_board_led_on(CENTRAL_SCANNING_LED);
                scan_start();
            }
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer that disconnected,
        // update the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("BLE_GAP_EVT_DISCONNECTED");
            if (p_gap_evt->conn_handle == m_conn_handle_hrs_c)
            {
                NRF_LOG_INFO("HRS peripheral disconnected from central (reason: %s)",
                             nrf_strerror_get(p_gap_evt->params.disconnected.reason));

                m_conn_handle_hrs_c = BLE_CONN_HANDLE_INVALID;
                
                err_code = nrf_ble_scan_filter_set(&m_scan, 
                                                   SCAN_UUID_FILTER, 
                                                   &m_adv_uuids[HART_RATE_SERVICE_UUID_SCAN_IDX]);
                APP_ERROR_CHECK(err_code);
            }
            
            if (p_gap_evt->conn_handle == m_conn_handle_cscs_c)
            {
                NRF_LOG_INFO("CSCS peripheral disconnected from central (reason: %s)",
                             nrf_strerror_get(p_gap_evt->params.disconnected.reason));

                m_conn_handle_cscs_c = BLE_CONN_HANDLE_INVALID;

                err_code = nrf_ble_scan_filter_set(&m_scan, 
                                                   SCAN_UUID_FILTER, 
                                                   &m_adv_uuids[CSCS_SERVICE_UUID_SCAN_IDX]);
                APP_ERROR_CHECK(err_code);
            }

            if (bond_mgr_deletion_pending)
            {
                // Disable scanning/advertising
                nrf_ble_scan_stop();
                if (is_advertising)
                {
                    err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
                    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                    is_advertising = false;
                }
                bond_mgr_delete_queued_peers(); // This one might error out with NRF_ERROR_INVALID_STATE
                // Wait for pm deletion completed event
            }
            /*if ((p_gap_evt->params.disconnected.reason == BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION) && need_peer_delete)
            {
                    NRF_LOG_INFO("Disconnected due to sec failure. Deleting peers.");
                    delete_bonds();
                    need_peer_delete = false;
                    adv_scan_start(); // Try by nRF support (moved from PM_EVT_PEERS_DELETE_SUCCEEDED)
            }
            */
            /*
            if (p_gap_evt->conn_handle == m_conn_handle_bas_c)
            {
                NRF_LOG_INFO("BAS central disconnected (reason: %d)",
                             p_gap_evt->params.disconnected.reason);

                m_conn_handle_bas_c = BLE_CONN_HANDLE_INVALID;

                err_code = nrf_ble_scan_filter_set(&m_scan, 
                                                   SCAN_UUID_FILTER, 
                                                   &m_adv_uuids[BATTERY_SERVICE_UUID_IDX]);
                APP_ERROR_CHECK(err_code);
            }
            */
            if (   (m_conn_handle_cscs_c == BLE_CONN_HANDLE_INVALID)
                || (m_conn_handle_hrs_c  == BLE_CONN_HANDLE_INVALID))
            
            {
                // Start scanning.
                scan_start();

                // Update LEDs status.
                bsp_board_led_on(CENTRAL_SCANNING_LED);
            }

            if (ble_conn_state_central_conn_count() == 0)
            {
                bsp_board_led_off(CENTRAL_CONNECTED_LED);
            }
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_TIMEOUT:
        {
            // No timeout for scanning is specified, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief   Function for handling BLE events from peripheral applications.
 * @details Updates the status LEDs used to report the activity of the peripheral applications.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            is_advertising = false;
            NRF_LOG_INFO("Client connected to us using us as server (peripheral)");
            if (m_central_conn_handle == BLE_CONN_HANDLE_INVALID && p_ble_evt->evt.gap_evt.conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                m_central_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                NRF_LOG_INFO("Stored central connection handle for disconnect handling");
            }
            bsp_board_led_off(PERIPHERAL_ADVERTISING_LED);
            bsp_board_led_on(PERIPHERAL_CONNECTED_LED);

            // Assign connection handle to the QWR module.
            multi_qwr_conn_handle_assign(p_ble_evt->evt.gap_evt.conn_handle);
#if NRF_SDH_BLE_SERVICE_CHANGED == 1
            NRF_LOG_INFO("Sending pm_local_database_has_changed() as NRF_SDH_BLE_SERVICE_CHANGED == 1");
            pm_local_database_has_changed();
#endif
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Client disconnected from us when we were acting as server (peripheral): connection handle: 0x%x, reason: 0x%x (%s)",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason, nrf_strerror_get(p_gap_evt->params.disconnected.reason));

            bsp_board_led_off(PERIPHERAL_CONNECTED_LED);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_TIMEOUT:
            // We'e no longer advertising after timeout
            is_advertising = false;
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
        {
            NRF_LOG_INFO("Fast advertising.");
            bsp_board_led_on(PERIPHERAL_ADVERTISING_LED);
        } break;

        case BLE_ADV_EVT_IDLE:
        {
            ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for checking whether a bluetooth stack event is an advertising timeout.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static bool ble_evt_is_advertising_timeout(ble_evt_t const * p_ble_evt)
{
    return (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role        = ble_conn_state_role(conn_handle);


    /***************************************************************************************
    * Make sure to not have any calls to BLE_SDH_OBSERVER or the like (BLE_STERZO_DEF, ..) *
    * in the code!!                                                                        *
    * This will cause duplicate events and all kind of havoc as the events are dispatched  *
    * in central and peripheral mode at the same time!!!                                   *
    ***************************************************************************************/

    // Based on the role this device plays in the connection, dispatch to the right handler.
    if (role == BLE_GAP_ROLE_PERIPH || ble_evt_is_advertising_timeout(p_ble_evt))
    {       
        if (ble_hrs_active)
        {
            ble_hrs_on_ble_evt(p_ble_evt, &m_hrs);
        }
        //ble_dis_on_ble_evt(p_ble_evt, &m_dis);
#if EMULATE_TACX 
        if (ble_fec_active)
        { 
            ble_fec_on_ble_evt(p_ble_evt, &m_fec);
        }
//#else
        
#endif
        if (ble_cps_active)
        {
            ble_cps_on_ble_evt(p_ble_evt, &m_cps);
        }

        if (ble_bas_active)
        {
            ble_bas_on_ble_evt(p_ble_evt, &m_bas);
        }

        if (ble_atom_active)
        {
            ble_atom_on_ble_evt(p_ble_evt, &m_atom);
        }

        if (ble_cscs_active)
        {
            ble_cscs_on_ble_evt(p_ble_evt, &m_cscs); // This one needs to be enabled?              
        }

        if (ble_ftms_active)
        {
            ble_ftms_on_ble_evt(p_ble_evt, &m_ftms);        
        }

        if (ble_uds_active)
        {
            ble_uds_on_ble_evt(p_ble_evt, &m_uds);        
        }
        
        on_ble_peripheral_evt(p_ble_evt);
    }
    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
    {
        ble_hrs_c_on_ble_evt(p_ble_evt, &m_hrs_c);
        ble_cscs_c_on_ble_evt(p_ble_evt, &m_cscs_c);
        // ble_bas_c_on_ble_evt(p_ble_evt, &m_bas_c);
        // ble_dis_c_on_ble_evt(p_ble_evt, &m_dis_c);
        on_ble_central_evt(p_ble_evt);
    }
}


/**@brief Heart Rate Collector initialization.
 */

static void hrs_c_init(void)
{
    ret_code_t       err_code;
    ble_hrs_c_init_t hrs_c_init_obj;

    hrs_c_init_obj.evt_handler   = hrs_c_evt_handler;
    hrs_c_init_obj.error_handler = service_error_handler;
    hrs_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_hrs_c_init(&m_hrs_c, &hrs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Battery Service initialization.
 */

static void bas_c_init(void)
{
    ret_code_t       err_code;
    ble_bas_c_init_t bas_c_init_obj, bas_c_init_obj2;

    bas_c_init_obj.evt_handler   = bas_c_evt_handler;
    bas_c_init_obj.error_handler = service_error_handler;
    bas_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_bas_c_init(&m_bas_c[0], &bas_c_init_obj);
    APP_ERROR_CHECK(err_code);
    // Init a second instance
    err_code = ble_bas_c_init(&m_bas_c[1], &bas_c_init_obj);
    APP_ERROR_CHECK(err_code);
}

static void dis_c_init(void)
{
    ret_code_t       err_code;
    ble_dis_c_init_t dis_c_init_obj;

    dis_c_init_obj.evt_handler   = dis_c_evt_handler;
    dis_c_init_obj.error_handler = service_error_handler;
    dis_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_dis_c_init(&m_dis_c[0], &dis_c_init_obj);
    APP_ERROR_CHECK(err_code);
    // Init a second instance
    err_code = ble_dis_c_init(&m_dis_c[1], &dis_c_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief RSC collector initialization.
 */
static void cscs_c_init(void)
{
    ret_code_t        err_code;
    ble_cscs_c_init_t cscs_c_init_obj;

    cscs_c_init_obj.evt_handler   = cscs_c_evt_handler;
    cscs_c_init_obj.error_handler = service_error_handler;
    cscs_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;
    cscs_c_init_obj.sim_mode      = SIM_MODE_ENABLED;
    // Initialize our incline_level to level TRAINER_INITIAL_INCLINE_LEVEL
    // cscs_c_init_obj.incline_level = &resistance_level; 
    
    err_code = ble_cscs_c_init(&m_cscs_c, &cscs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the Peer Manager.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;
    ble_gap_addr_t          p_addr;    
    ble_gap_irk_t           peer_gap_id_key;
    char s_addr[18];
    char key[33];
    pm_peer_id_t            current_peer_id;
    pm_peer_data_id_t       data_id;
    pm_peer_data_bonding_t  bonding_data;
    pm_peer_data_bonding_t  *p_data;
    uint32_t p_len;
    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;
    
    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = pm_id_addr_get(&p_addr);
    APP_ERROR_CHECK(err_code);

    switch(p_addr.addr_type)
    {        
        case BLE_GAP_ADDR_TYPE_PUBLIC:
            NRF_LOG_INFO("GAP addr type: Public");
            break;
        case BLE_GAP_ADDR_TYPE_RANDOM_STATIC:
            NRF_LOG_INFO("GAP addr type: Random static");
            break;     
        case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE:
            NRF_LOG_INFO("GAP addr type: Random private resolvable");
            break;
        case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
            NRF_LOG_INFO("GAP addr type: Random private non-resolvable");
            break;
        case BLE_GAP_ADDR_TYPE_ANONYMOUS:
            NRF_LOG_INFO("GAP addr type: Anonymous");
            break;
        default:
            NRF_LOG_INFO("GAP addr type: Unknown");
            break;
        break;
    }
        
    sprintf(s_addr, "%02x:%02x:%02x:%02x:%02x:%02x", p_addr.addr[5], p_addr.addr[4], p_addr.addr[3], p_addr.addr[2], p_addr.addr[1], p_addr.addr[0]);
    NRF_LOG_INFO("Our address: %s", s_addr); 
    NRF_LOG_FLUSH();
    uint32_t cnt = pm_peer_count();
    NRF_LOG_INFO("%d peers stored in persistent flash", cnt);
    
    current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (current_peer_id != PM_PEER_ID_INVALID)
    {        
        p_data = &bonding_data;
        p_len = sizeof(bonding_data);
        err_code = pm_peer_data_load(current_peer_id, PM_PEER_DATA_ID_BONDING, p_data, &p_len);
        APP_ERROR_CHECK(err_code);
        switch (bonding_data.own_role)
        {
            case BLE_GAP_ROLE_PERIPH:
                NRF_LOG_INFO("Own role: BLE_GAP_ROLE_PERIPH");
                break;
            case BLE_GAP_ROLE_CENTRAL:
                NRF_LOG_INFO("Own role: BLE_GAP_ROLE_CENTRAL");
                break;
            default:
                NRF_LOG_INFO("Own role: BLE_GAP_ROLE_INVALID");
                break;
        }
        peer_gap_id_key = bonding_data.peer_ble_id.id_info;
        p_addr = bonding_data.peer_ble_id.id_addr_info;
        sprintf(s_addr, "%02x:%02x:%02x:%02x:%02x:%02x", p_addr.addr[0], p_addr.addr[1], p_addr.addr[2], p_addr.addr[3], p_addr.addr[4], p_addr.addr[5]);
        NRF_LOG_INFO("Peer address: %s", s_addr); 
        NRF_LOG_FLUSH();

        // Get Identity recovering key
        for (int idx=0; idx<=15; idx+=1)
        {
            sprintf(&key[idx*2], "%02x", peer_gap_id_key.irk[idx]);
        }
        
        NRF_LOG_INFO("Identity recovery key of peer: 0x%s", key); 
        NRF_LOG_FLUSH();
        current_peer_id = pm_next_peer_id_get(current_peer_id);
    }
    NRF_LOG_FLUSH(); // needed because s_addr is a local variable and get' out of scope https://devzone.nordicsemi.com/f/nordic-q-a/22647/nrf_log_info-how-to-print-log-with-string-parameter
}


/**@brief Clear bond information from persistent storage.
 */
void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");
    // https://punchthrough.com/ble-bond-deletion-constraints/
    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to
 *                            wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the GAP.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device, including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_CYCLING_POWER_SENSOR);    
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = m_scan.conn_params.min_conn_interval;
    gap_conn_params.max_conn_interval = m_scan.conn_params.max_conn_interval;
    gap_conn_params.slave_latency     = m_scan.conn_params.slave_latency;
    gap_conn_params.conn_sup_timeout  = m_scan.conn_params.conn_sup_timeout;
    
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_CONN_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;  // Ignore events.
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_db_discovery_t const * p_db = (ble_db_discovery_t *)p_evt->params.p_db_instance;

    ble_hrs_on_db_disc_evt(&m_hrs_c, p_evt);
    ble_cscs_on_db_disc_evt(&m_cscs_c, p_evt);
    if (m_dis_c[0].conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        ble_dis_c_on_db_disc_evt(&m_dis_c[0], p_evt);
    } 
    else if (m_dis_c[1].conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        ble_dis_c_on_db_disc_evt(&m_dis_c[1], p_evt);
    }

    if (m_bas_c[0].conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        ble_bas_on_db_disc_evt(&m_bas_c[0], p_evt);
    } 
    else if (m_bas_c[1].conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        ble_bas_on_db_disc_evt(&m_bas_c[1], p_evt);
    }     
    
    if (p_evt->evt_type == BLE_DB_DISCOVERY_AVAILABLE) {
        NRF_LOG_INFO("DB Discovery instance %p available on conn handle: %d",
                     p_db,
                     p_evt->conn_handle);
        NRF_LOG_INFO("Found %d services on conn_handle: %d",
                     p_db->srv_count,
                     p_evt->conn_handle);
    }
}


/**
 * @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write module errors.
 *
 * @details A pointer to this function is passed to each service that may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code that contains information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for handling Power Control point events
 *
 * @details Function for handling Power Control point events.
 *          This function parses the event and in case the "set cumulative value" event is received,
 *          sets the wheel cumulative value to the received value.
 */
ble_scpt_response_t sc_ctrlpt_event_handler(ble_sc_ctrlpt_t     * p_sc_ctrlpt,
                                            ble_sc_ctrlpt_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_SC_CTRLPT_EVT_SET_CUMUL_VALUE:
            
            // m_cumulative_wheel_revs = p_evt->params.cumulative_value;
            break;

        case BLE_SC_CTRLPT_EVT_START_CALIBRATION:
            // m_auto_calibration_in_progress = true;
            break;

        default:
            // No implementation needed.
            break;
    }
    return (BLE_SCPT_SUCCESS);
}

ble_cpspt_response_t cps_ctrlpt_event_handler(ble_cps_ctrlpt_t     * p_cps_ctrlpt,
                                            ble_cps_ctrlpt_evt_t * p_evt)
{

    switch (p_evt->evt_type)
    {
        case BLE_CPS_CTRLPT_EVT_SET_CUMUL_VALUE:
            // m_cumulative_wheel_revs = p_evt->params.cumulative_value;
            break;

        case BLE_CPS_CTRLPT_EVT_START_CALIBRATION:
            // m_auto_calibration_in_progress = true;
            break;

        default:
            // No implementation needed.
            break;
    }
    return (BLE_CPSPT_SUCCESS);
}

void atom_event_handler(ble_atom_t *p_atom, ble_atom_evt_t *p_evt)
{
}



void ftms_event_handler(ble_ftms_t * p_ftms, ble_ftms_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_FTMS_EVT_INDICATION_ENABLED:
            NRF_LOG_INFO("Enabled INDICATION on ftms control point characteristic");
            break;
        case BLE_FTMS_EVT_INDICATION_DISABLED:
            NRF_LOG_INFO("Disabled INDICATION on ftms control point characteristic");
        case BLE_FTMS_EVT_NOTIFICATION_ENABLED: // Does this exist at all?
            NRF_LOG_INFO("Enabled NOTIFICATION on ftms control point characteristic");
            break;
        case BLE_FTMS_EVT_NOTIFICATION_DISABLED:
            NRF_LOG_INFO("Disabled NOTIFICATION on ftms control point characteristic");
                
            break;
        case BLE_FTMS_EVT_MACHINESTATUS_NOTIFICATION_ENABLED:
            NRF_LOG_INFO("Enabled NOTIFICATION on ftms machine status characteristic");
            
            break;
        case BLE_FTMS_EVT_MACHINESTATUS_NOTIFICATION_DISABLED:
           NRF_LOG_INFO("Disabled NOTIFICATION on ftms machine status characteristic");
            
            break;
        case BLE_FTMS_EVT_INDOORBIKEDATA_NOTIFICATION_ENABLED:
           NRF_LOG_INFO("Enabled NOTIFICATION on ftms indoor bike data characteristic");
            
            break;
        case BLE_FTMS_EVT_INDOORBIKEDATA_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("Disabled NOTIFICATION on ftms indoor bike data characteristic");
            
            break;
        case BLE_FTMS_EVT_TRAININGSTATUS_NOTIFICATION_ENABLED:
        NRF_LOG_INFO("Enabled NOTIFICATION on ftms training status characteristic");
            
            break;
        case BLE_FTMS_EVT_TRAININGSTATUS_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("Disabled NOTIFICATION on ftms training status characteristic");
            
            break;
        default:
            break;
    }
}

ble_ftmspt_response_t ftms_ctrlpt_event_handler(ble_ftms_ctrlpt_t *p_ftms_ctrlpt,
                                            ble_ftms_ctrlpt_evt_t * p_evt)
{
    // NRF_LOG_INFO("In ftms_ctrlpt_event_handler()")
    switch (p_evt->evt_type)
    {
        case BLE_FTMS_CTRLPT_EVT_REQUEST_CONTROL:
            // NRF_LOG_INFO("Received event BLE_FTMS_CTRLPT_EVT_REQUEST_CONTROL. Acquiring control...");            
            ftms_op_acquire_control(&m_ftms);
            // Do we need to return a return value?
            break;

        case BLE_FTMS_CTRLPT_EVT_RESET:
            // NRF_LOG_INFO("Received event BLE_FTMS_CTRLPT_EVT_RESET. Resetting...");
            ftms_op_reset(&m_ftms);
            // Do we need to return a return value?
            break;

        case BLE_FTMS_CTRLPT_EVT_SET_TARGET_INCLINATION:
            // NRF_LOG_INFO("Received event BLE_FTMS_CTRLPT_EVT_SET_TARGET_INCLINATION. Setting inclination...");   
            ftms_op_set_target_inclination(&m_ftms, p_evt->params.target_inclination);            
            break;

        case BLE_FTMS_CTRLPT_EVT_SET_TARGET_POWER:
            // NRF_LOG_INFO("Received event BLE_FTMS_CTRLPT_EVT_SET_TARGET_POWER. Setting power...");    
            ftms_op_set_target_power(&m_ftms, p_evt->params.target_power);
            break;

        case BLE_FTMS_CTRLPT_EVT_SET_TARGET_RESISTANCE:
            // NRF_LOG_INFO("Received event BLE_FTMS_CTRLPT_EVT_SET_TARGET_RESISTANCE. Setting resistance...");                
            ftms_op_set_target_resistance(&m_ftms, p_evt->params.target_resistance);
            break;
        case BLE_FTMS_CTRLPT_EVT_SET_TARGETED_CADENCE:
            // NRF_LOG_INFO("Received event BLE_FTMS_CTRLPT_EVT_SET_TARGETED_CADENCE. Setting cadence...");                
            ftms_op_set_targeted_cadence(&m_ftms, p_evt->params.target_cadence);
            break;
        case BLE_FTMS_CTRLPT_EVT_START_RESUME:
            // NRF_LOG_INFO("Received event BLE_FTMS_CTRLPT_EVT_START_RESUME. Starting/Resuming..."); 
            ftms_op_start_resume_training(&m_ftms);            
            break;

        case BLE_FTMS_CTRLPT_EVT_STOP_PAUSE:
            // NRF_LOG_INFO("Received event BLE_FTMS_CTRLPT_EVT_STOP_PAUSE. Stopping/Pausing..."); 
            ftms_op_stop_pause_training(&m_ftms, p_evt->params.stop_pause_value);
            break;
        case BLE_FTMS_CTRLPT_EVT_SET_INDOOR_TRAINING_BIKE_PARAMETERS:
            // NRF_LOG_INFO("Received event BLE_FTMS_CTRLPT_EVT_SET_INDOOR_TRAINING_BIKE_PARAMETERS. Setting...");
            ftms_op_set_indoor_training_bike_parameters(&m_ftms, p_evt->params.indoor_bike_simulation_parameters);
            break;       
        default:
            // No implementation needed.
            break;
    }
    return (BLE_FTMSPT_SUCCESS);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void fec_data_handler(ble_fec_t *p_fec, ble_fec_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case BLE_FEC_EVT_RX_DATA:                                
            fec_handle_command(p_fec, p_evt);
            break;
        case BLE_FEC_EVT_TX_RDY:    
            // Do something too
            // Logs all events not only NUS
            // NRF_LOG_INFO("FE_C BLE_NUS_EVT_TX_RDY");            
            break;
        case BLE_FEC_EVT_COMM_STARTED:
            NRF_LOG_INFO("Enabled NOTIFICATIONS on FE-C over BLE (FEC2)")   
            break;
        case BLE_FEC_EVT_COMM_STOPPED:
             NRF_LOG_INFO("Disabled NOTIFICATIONS on FE-C over BLE (FEC2)")   
             break;
        default:
            NRF_LOG_INFO("Unknown event");
            break;
    }
}

void uds_event_handler(ble_uds_t *p_uds, ble_uds_evt_t *p_evt)
{
     switch (p_evt->evt_type)
    {
        case BLE_UDS_CHGINCREMENT_NOTIFICATION_ENABLED:
            NRF_LOG_INFO("Enabled NOTIFICATION on UDS change indicator characteristic");
            
            break;
         case BLE_UDS_CHGINCREMENT_NOTIFICATION_DISABLED:
            NRF_LOG_INFO("Disabled NOTIFICATION on UDS change indicator characteristic");            
            break;
         case BLE_UDS_EVT_INDICATION_ENABLED:
NRF_LOG_INFO("Enabled INDICATION on UDS control point characteristic");            
            break;
         case BLE_UDS_EVT_INDICATION_DISABLED:
         NRF_LOG_INFO("Disabled INDICATION on UDS control point characteristic");            
            break;
         case BLE_UDS_CTRLPT_EVT_REGISTER_NEW_USER:
          NRF_LOG_INFO("Stub - registering new user");
          break;
case BLE_UDS_CTRLPT_EVT_CONSENT:
          NRF_LOG_INFO("Stub - send consent");
          break;
case BLE_UDS_CTRLPT_EVT_DELETE_USER:
          NRF_LOG_INFO("Stub - delete user");
          break;
          
case BLE_UDS_CTRLPT_EVT_DELETE_USER_DATA:
          NRF_LOG_INFO("Stub - delete user data");
          break;
          
case BLE_UDS_CTRLPT_EVT_LIST_ALL_USERS:
          NRF_LOG_INFO("Stub - list all users");
          break;
case BLE_UDS_CTRLPT_EVT_RESPONSE_CODE:
          NRF_LOG_INFO("Stub - response code");
          break;
          
          
          
        default:
            NRF_LOG_INFO("Received another event: %u", p_evt->evt_type);
            break;
    }    
}
void cscs_event_handler(ble_cscs_t * p_cscs, ble_cscs_evt_t * p_evt)
{
   switch (p_evt->evt_type)
    {
        case BLE_CSCS_EVT_NOTIFICATION_ENABLED:
            NRF_LOG_INFO("Enabled NOTIFICATION on CSCS measurement characteristic");
            
            break;
         case BLE_CSCS_EVT_NOTIFICATION_DISABLED:
            NRF_LOG_INFO("Disabled NOTIFICATION on CSCS measurement characteristic");
            
            break;
        default:
            break;
    }
}


void cps_event_handler(ble_cps_t * p_cscs, ble_cps_evt_t * p_evt)
{
   switch (p_evt->evt_type)
    {
        case BLE_CPS_EVT_NOTIFICATION_ENABLED:
            NRF_LOG_INFO("Enabled NOTIFICATION on CPS measurement characteristic");
            
            break;
         case BLE_CPS_EVT_NOTIFICATION_DISABLED:
            NRF_LOG_INFO("Disabled NOTIFICATION on CPS measurement characteristic");
            
            break;
        default:
            break;
    }
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that are be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hrs_init_t     hrs_init;
    ble_cps_init_t     cps_init;
    ble_cscs_init_t    cscs_init;
    ble_bas_init_t     bas_init;
    ble_ftms_init_t    ftms_init;
    
    ble_atom_init_t    atom_init;
    ble_uds_init_t     uds_init;
#if EMULATE_TACX
    ble_fec_init_t     fec_init;
 //   ble_boot_init_t     boot_init; // NOT needed for Tacx
#endif
    ble_dis_init_t     dis_init;
    ble_sensor_location_t sensor_location;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dfu_buttonless_init_t dfus_init = {0};
    uint8_t            body_sensor_location;


    // Initialize Queued Write module instances (needed for FE-C / NUS service).
    qwr_init.error_handler = nrf_qwr_error_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }

    ////////////////////////////////////////////////////////////////////////
    //
    // If ERR_NO_MEM, try increasing NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE in sdk_config to a (much bigger size) or play with Properties->Common->Linker->RAM_START + RAM_SIZE    
    //
    ////////////////////////////////////////////////////////////////////////
    if (ble_bas_active)
    {
        // Initialize the Battery Service.
        memset(&bas_init, 0, sizeof(bas_init));

        bas_init.evt_handler          = NULL;
        bas_init.support_notification = true;
        bas_init.p_report_ref         = NULL;
        bas_init.initial_batt_level   = 100;

        // Here the sec level for the Battery Service can be changed or increased.
        bas_init.bl_cccd_wr_sec   = SEC_OPEN;
        bas_init.bl_report_rd_sec = SEC_OPEN;
        bas_init.bl_rd_sec        = SEC_OPEN;

        err_code = ble_bas_init(&m_bas, &bas_init);
        APP_ERROR_CHECK(err_code);
    }

    if (ble_hrs_active)
    {
        // Initialize the Heart Rate Service.
        body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_CHEST;

        memset(&hrs_init, 0, sizeof(hrs_init));

        hrs_init.evt_handler                 = NULL;
        hrs_init.is_sensor_contact_supported = true;
        hrs_init.p_body_sensor_location      = &body_sensor_location;

        // Here the sec level for the Heart Rate Service can be changed or increased.
        hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
        hrs_init.bsl_rd_sec      = SEC_OPEN;

        err_code = ble_hrs_init(&m_hrs, &hrs_init);
        APP_ERROR_CHECK(err_code);
    }

    if (ble_cscs_active)
    {
        // Initialize Cycling Speed and Cadence Service.
        memset(&cscs_init, 0, sizeof(cscs_init));

        cscs_init.evt_handler = cscs_event_handler;
        cscs_init.feature     = BLE_CSCS_FEATURE_WHEEL_REV_BIT | BLE_CSCS_FEATURE_CRANK_REV_BIT;

        // Here the sec level for the Cycling Speed and Cadence Service can be changed/increased.
        cscs_init.csc_meas_cccd_wr_sec  = SEC_OPEN;
        cscs_init.csc_feature_rd_sec    = SEC_OPEN;
        cscs_init.csc_location_rd_sec   = SEC_OPEN;
        cscs_init.sc_ctrlpt_cccd_wr_sec = SEC_OPEN;
        cscs_init.sc_ctrlpt_wr_sec      = SEC_OPEN;

        cscs_init.ctrplt_supported_functions = BLE_SRV_SC_CTRLPT_CUM_VAL_OP_SUPPORTED
                                               | BLE_SRV_SC_CTRLPT_SENSOR_LOCATIONS_OP_SUPPORTED
                                               | BLE_SRV_SC_CTRLPT_START_CALIB_OP_SUPPORTED;
        cscs_init.ctrlpt_evt_handler            = sc_ctrlpt_event_handler;
        cscs_init.list_supported_locations      = supported_locations;
        cscs_init.size_list_supported_locations = sizeof(supported_locations) /
                                                  sizeof(ble_sensor_location_t);

        sensor_location           = BLE_SENSOR_LOCATION_FRONT_WHEEL;                 // initializes the sensor location to add the sensor location characteristic.
        cscs_init.sensor_location = &sensor_location;

        err_code = ble_cscs_init(&m_cscs, &cscs_init);
        APP_ERROR_CHECK(err_code);
    }
   
    if (ble_cps_active)
    {
        // Initialize the Cycling Power Service.
        memset(&cps_init, 0, sizeof(cps_init));

        cps_init.evt_handler = cps_event_handler;
        cps_init.feature     =  BLE_CPS_FEATURE_CRANK_REV_BIT | 
                                BLE_CPS_FEATURE_WHEEL_REV_BIT;
    
        // Here the sec level for the Cycling Power Service can be changed or increased.    
        cps_init.cps_feature_rd_sec    = SEC_OPEN;
        cps_init.cps_meas_cccd_wr_sec  = SEC_OPEN;
        cps_init.cps_location_rd_sec   = SEC_OPEN;
        cps_init.cps_ctrlpt_cccd_wr_sec = SEC_OPEN;
        cps_init.cps_ctrlpt_wr_sec      = SEC_OPEN;

        cps_init.ctrplt_supported_functions = 0; //BLE_SRV_SC_CTRLPT_CUM_VAL_OP_SUPPORTED
                                              // | BLE_SRV_SC_CTRLPT_SENSOR_LOCATIONS_OP_SUPPORTED
                                               //| BLE_SRV_SC_CTRLPT_START_CALIB_OP_SUPPORTED;
        cps_init.ctrlpt_evt_handler            = cps_ctrlpt_event_handler;
        cps_init.list_supported_locations      = supported_locations;
        cps_init.size_list_supported_locations = sizeof(supported_locations) /
                                                  sizeof(ble_sensor_location_t);

        sensor_location           = BLE_SENSOR_LOCATION_RIGHT_CRANK;                 // initializes the sensor location to add the sensor location characteristic.
        cps_init.sensor_location = &sensor_location;
        cps_init.resistance_level = &resistance_level; // Preset out difficulty to level TRAINER_INITIAL_INCLINE_LEVEL
        err_code = ble_cps_init(&m_cps, &cps_init);
        APP_ERROR_CHECK(err_code);

    }
        
    if (ble_ftms_active)
    {        
        // Initialize the Fitness Machine Service.
        memset(&ftms_init, 0, sizeof(ftms_init));

        ftms_init.evt_handler = ftms_event_handler;

        ftms_init.fitness_machine_feature.fitness_machine_features = BLE_FTMS_FEATURE_AVERAGE_SPEED_SUPPORTED_BIT | BLE_FTMS_FEATURE_CADENCE_SUPPORTED_BIT | BLE_FTMS_FEATURE_RESISTANCE_SUPPORTED_BIT |  BLE_FTMS_FEATURE_INCLINATION_SUPPORTED_BIT | BLE_FTMS_FEATURE_POWER_MEASUREMENT_SUPPORTED_BIT;
        ftms_init.fitness_machine_feature.target_setting_features       = BLE_FTMS_TARGET_INCLINATION_SUPPORTED_BIT | BLE_FTMS_TARGET_POWER_SUPPORTED_BIT | BLE_FTMS_TARGETED_CADENCE_CONFIGURATION_SUPPORTED_BIT | BLE_FTMS_TARGET_RESISTANCE_SUPPORTED_BIT;
        ftms_init.training_status = BLE_FTMS_TRAINING_STATUS_IDLE;
        ftms_init.resistance_level = &resistance_level; // Preset out difficulty to level TRAINER_INITIAL_INCLINE_LEVEL - - no pointer here!
        // Here the sec level for the Fitness Machine Service can be changed or increased.    
        ftms_init.ftms_feature_rd_sec    = SEC_OPEN; // We only use this one for the moment
        ftms_init.ftms_machine_status_rd_sec = SEC_OPEN;
        ftms_init.ftms_indoor_bike_rd_sec = SEC_OPEN;
        ftms_init.ftms_training_status_rd_sec = SEC_OPEN;
        ftms_init.ftms_sup_power_range_rd_sec = SEC_OPEN;
        ftms_init.ftms_sup_inclination_range_rd_sec = SEC_OPEN;
        ftms_init.ftms_sup_resistance_range_rd_sec = SEC_OPEN;


        ftms_init.ftms_cccd_wr_sec  = SEC_OPEN;
        ftms_init.ftms_ctrlpt_cccd_wr_sec = SEC_OPEN;
        ftms_init.ftms_ctrlpt_wr_sec      = SEC_OPEN;

        ftms_init.ctrplt_supported_functions = BLE_SRV_FTMS_CTRLPT_REQUEST_CONTROL_SUPPORTED | BLE_SRV_FTMS_CTRLPT_RESET_SUPPORTED 
                                             | BLE_SRV_FTMS_CTRLPT_SET_TARGET_INCLINATION_SUPPORTED 
                                             | BLE_SRV_FTMS_CTRLPT_SET_TARGET_POWER_SUPPORTED
                                             | BLE_SRV_FTMS_CTRLPT_SET_TARGET_RESISTANCE_SUPPORTED
                                             | BLE_SRV_FTMS_CTRLPT_START_RESUME_SUPPORTED
                                             | BLE_SRV_FTMS_CTRLPT_STOP_PAUSE_SUPPORTED 
                                             | BLE_SRV_FTMS_CTRLPT_SET_INDOOR_TRAINING_BIKE_PARAMETERS_SUPPORTED;   
        ftms_init.ctrlpt_evt_handler            = ftms_ctrlpt_event_handler;
    
        err_code = ble_ftms_init(&m_ftms, &ftms_init);
        APP_ERROR_CHECK(err_code);    
    }

 
#if EMULATE_TACX
    if (ble_fec_active)
    {
        // Initialize the BLE FEC Service for ANT-FEC
        // Initialize FEC.
    
        memset(&fec_init, 0, sizeof(fec_init));

        fec_init.data_handler = fec_data_handler;
    
        err_code = ble_fec_init(&m_fec, &fec_init);
        APP_ERROR_CHECK(err_code);  

        // Initialize "Boot" service
        //memset(&boot_init, 0, sizeof(boot_init));

        //boot_init.data_handler = 0L;
        //err_code = ble_boot_init(&m_boot, &boot_init);
        //APP_ERROR_CHECK(err_code);  
    }
// Removed reversed 3rd party module...
#endif  

    if (ble_atom_active)
    {
        // Initialize atom
        memset(&atom_init, 0, sizeof(atom_init));
        atom_init.evt_handler = atom_event_handler;
        atom_init.atom_readings_rd_sec = SEC_OPEN;
        atom_init.atom_readings_cccd_wr_sec = SEC_OPEN;
        atom_init.gear_offset = &gear_offset;
        err_code = ble_atom_init(&m_atom, &atom_init);
        APP_ERROR_CHECK(err_code);
    }

   


    if (ble_dis_active)
    {
        // Initialize Device Information Service.
        memset(&dis_init, 0, sizeof(dis_init));

        ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
        ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char *) SERIAL_NUMBER);
        ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *) HW_REVISION);
        ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, (char *) SW_REVISION);
        //ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *) "FTMS Xtreme");
        ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUMBER);


        dis_init.dis_char_rd_sec = SEC_OPEN;

        err_code = ble_dis_init(&dis_init);

        APP_ERROR_CHECK(err_code);
    }

     if (ble_uds_active)
    {
        // Initialize UDS
        memset(&uds_init, 0, sizeof(uds_init));
        uds_init.evt_handler = uds_event_handler;
        uds_init.uds_tx_cccd_rd_sec = SEC_OPEN;
        uds_init.uds_rx_cccd_wr_sec = SEC_OPEN;
        uds_init.uds_ctrlpt_cccd_wr_sec = SEC_OPEN;
        uds_init.uds_ctrlpt_wr_sec = SEC_OPEN;
        
        uds_init.uds_weight_wr_sec = SEC_OPEN;
        uds_init.uds_height_wr_sec = SEC_OPEN;
        uds_init.uds_change_increment_wr_sec = SEC_OPEN;
        uds_init.uds_user_index_rd_sec = SEC_OPEN;
        uds_init.supported_functions = BLE_SRV_UDS_CTRLPT_REGISTER_NEW_USER | BLE_SRV_UDS_CTRLPT_CONSENT 
                                             | BLE_SRV_UDS_CTRLPT_DELETE_USER_SUPPORTED 
                                             | BLE_SRV_UDS_CTRLPT_DELETE_USER_DATA_SUPPORTED
                                             | BLE_SRV_UDS_CTRLPT_LIST_ALL_USERS_SUPPORTED
                                             | BLE_SRV_UDS_CTRLPT_RESPONSE_CODE_SUPPORTED;                                             
        err_code = ble_uds_init(&m_uds, &uds_init);
        APP_ERROR_CHECK(err_code);
    }
    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;
    ble_advdata_service_data_t service_data;
#if EMULATE_TACX
    ble_advdata_manuf_data_t manuf_data;
#endif
    //    
    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
    
#if EMULATE_TACX
    uint8_t mdata[] = "";
    manuf_data.company_identifier = 0x0689; // Tacx
    manuf_data.data.p_data                    = mdata;
    manuf_data.data.size                      = sizeof(mdata);
    init.advdata.p_manuf_specific_data   = &manuf_data;
    
    init.srdata.name_type               = BLE_ADVDATA_NO_NAME;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_sr_uuids) / sizeof(m_adv_sr_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_sr_uuids;
#endif

    // Service data AD type
    // Service Data AD Type: UINT8 = 16 Bit UUID
    // Fitness Machine Service UUID = UINT16 
    // Flags: UINT8, Bit 0 = 1, Value = 0x01
    // Fitness Machine Type: UINT16, 1<<5 => Value = 0x32
    
    
    uint8_t data[3] = { 0x01, 0x32, 0x00 }; // 0x01 = Available, 0x32 0x00 = Fitness Machine Type 
    service_data.service_uuid = BLE_UUID_FITNESS_MACHINE;  
    service_data.data.p_data = data;
    service_data.data.size = sizeof(data);

    init.srdata.service_data_count = 1;
    init.srdata.p_service_data_array = &service_data;
    
    // We should add:
    /*
    3.1.1.5 Service Data AD Type
    In order to facilitate the connection between a Fitness Machine and a Collector, a Fitness Machine should
    support the Service Data AD Type.
    When the Fitness Machine is using Connectable Undirected Advertising, it should include the Service
    Data AD Type in its Advertising Data to reduce unwanted connection requests by unintended Collectors.
    The Service Data payload (defined in [1]) includes a Flags field and a Fitness Machine Type field
    */

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing logging.
 */
static void log_init(void)
{
    #if NRF_LOG_USES_TIMESTAMP==1
    counter_init();
    counter_start();
    
    ret_code_t err_code = NRF_LOG_INIT(counter_get);
    #else
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    #endif

    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop). If there is no pending log operation,
          then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code = 0;
    /*
    // GPIOTE is already initialized when we'e here!
    if(!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
    }
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    //err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    //APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(TRAINER_BUTTON_INCLINE_UP, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(TRAINER_BUTTON_INCLINE_DOWN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(TRAINER_BUTTON_INCLINE_UP, true);
    nrf_drv_gpiote_in_event_enable(TRAINER_BUTTON_INCLINE_DOWN, true);
    */
   // APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

	
	
   /*
    //init app_button module, 50ms detection delay (button debouncing)
    err_code = app_button_init((app_button_cfg_t *)app_buttons,
                                   sizeof(app_buttons)/sizeof(app_buttons[0]),
                                   APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
    */
}

void hometrainer_control_init()
{
    // Initialize my 2 GPIOs for remote control of trainer
    nrf_gpio_cfg_output(TRIGGER_RESISTANCE_UP_GPIO);
    nrf_gpio_cfg_output(TRIGGER_RESISTANCE_DOWN_GPIO);
}

void triggerFECEvent()
{
    if (doTriggerFECEvent)
    {
        ble_fec_page_evt_t ble_fec_page_evt;

        ble_fec_page_evt.type = BLE_FEC_REQUEST_TYPE_BROADCAST;
        ble_fec_page_evt.page = getFECPage();

        ble_fec_page_handler_t p_fec_page_handler = getFECPageHandler();
        ble_fec_t *fec_handle = getFECHandle();
        
        p_fec_page_handler(fec_handle, &ble_fec_page_evt);
    }
    doTriggerFECEvent = false;
}

void injectSimEvent()
{
static uint16_t cum_c_revs = 0;
static uint16_t last_c_evt_time = 0;
ble_cscs_c_evt_t ble_cscs_c_evt;

    if (doInjectSimEvent)
    {
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
            
        ble_cscs_c_evt.evt_type    = BLE_CSCS_C_EVT_CSC_NOTIFICATION;
    
        ble_cscs_c_evt.params.csc.is_crank_rev_data_present = true;
        ble_cscs_c_evt.params.csc.is_wheel_rev_data_present = true;
        
        ble_cscs_c_evt.params.csc.cumulative_crank_revs = getCumulativeCrankRevs(); 
        ble_cscs_c_evt.params.csc.last_crank_event_time = getLastCrankEventTime(); 
      
        ble_cscs_c_evt.params.csc.cumulative_wheel_revs = getCumulativeWheelRevs(); 
        ble_cscs_c_evt.params.csc.last_wheel_event_time = getLastWheelEventTime(); 
        m_cscs_c.evt_handler(&m_cscs_c, &ble_cscs_c_evt);    
    }

    doInjectSimEvent = false;
}

// Overwritten app_error_fault_handler. We will send data to oled and assert

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    static volatile struct
    {
        uint32_t        fault_id;
        uint32_t        pc;
        uint32_t        error_info;
        assert_info_t * p_assert_info;
        error_info_t  * p_error_info;
        ret_code_t      err_code;
        uint32_t        line_num;
        const uint8_t * p_file_name;
    } m_error_data = {0};

    uint8_t offset = 0;
    
    stopOLEDUpdates();          
    
    /*********************************************
     * WITHOUT DEBUG SYMBOLS NO LOC,PC,Filename! *
     * --> Release build won't log!              *
     *********************************************/

    // Check https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.s132.api.v7.2.0%2Fgroup__nrf__error.html 
    // for the actual error code!

    char buf[32];
     
    // Copied from app_error.c
      

    // The following variable helps Keil keep the call stack visible, in addition, it can be set to
    // 0 in the debugger to continue executing code after the error check.
    volatile bool loop = true;
    UNUSED_VARIABLE(loop);

    m_error_data.fault_id   = id;
    m_error_data.pc         = pc;
    m_error_data.error_info = info;

    switch (id)
    {
        case NRF_FAULT_ID_SDK_ASSERT:
            m_error_data.p_assert_info = (assert_info_t *)info;
            m_error_data.line_num      = m_error_data.p_assert_info->line_num;
            m_error_data.p_file_name   = m_error_data.p_assert_info->p_file_name;
            
            oled_printStringAt(30,2, "SDK Assert:", true, true);
            sprintf(buf, "ID: 0x%08x", id);
            oled_printStringAt(45,2, buf, false, true);
            sprintf(buf, "PC: 0x%08x", pc);
            oled_printStringAt(60,2, buf, false, true);
            sprintf(buf, "LoC: %lu", m_error_data.line_num);
            oled_printStringAt(75,2, buf, false, true);
            
            offset = strlen(m_error_data.p_file_name);
            if (offset > 0)
            {
                char *ptr_filename = (char *) m_error_data.p_file_name;                
                ptr_filename += offset;
                while (*ptr_filename != '\\' && *ptr_filename != '/' && ptr_filename > (char *) m_error_data.p_file_name) 
                {
                    ptr_filename--;                
                }
                ptr_filename++;
                oled_printStringAt(90,2, ptr_filename, false, true);
            }                      
             NRF_LOG_ERROR("App Error! ID: 0x%08x, PC: 0x%08x, LoC: %lu, File: %s", id, pc, m_error_data.line_num, m_error_data.p_file_name);
            
            break;

        case NRF_FAULT_ID_SDK_ERROR:
            m_error_data.p_error_info = (error_info_t *)info;
            m_error_data.err_code     = m_error_data.p_error_info->err_code;
            m_error_data.line_num     = m_error_data.p_error_info->line_num;
            m_error_data.p_file_name  = m_error_data.p_error_info->p_file_name;
            
            oled_printStringAt(30,2, "SDK Error:", true, true);
            sprintf(buf, "Err: 0x%08x", m_error_data.err_code);
            oled_printStringAt(45,2, buf, false, true);
            sprintf(buf, "PC: 0x%08x", pc);
            oled_printStringAt(60,2, buf, false, true);
            
            char const * p_desc = nrf_strerror_find(m_error_data.err_code);
            sprintf(buf, "..._%s", p_desc+10); /// strip the NRF_ERROR prefix
            oled_printStringAt(75,2, buf, false, true);

            sprintf(buf, "LoC: %lu", m_error_data.line_num);
            oled_printStringAt(90,2, buf, false, true);

            offset = strlen(m_error_data.p_file_name);
            if (offset > 0)
            {
                char *ptr_filename = (char *) m_error_data.p_file_name;                
                ptr_filename += offset;
                while (*ptr_filename != '\\' && *ptr_filename != '/' && ptr_filename > (char *) m_error_data.p_file_name) 
                {
                    ptr_filename--;                
                }
                ptr_filename++;
                oled_printStringAt(105,2, ptr_filename, false, true);
            }     
            NRF_LOG_ERROR("App Error! ID: 0x%08x, PC: 0x%08x, Loc: %lu, Error: %s, File: %s", id, pc, m_error_data.line_num, p_desc, m_error_data.p_file_name);
            break;
        default:
            NRF_LOG_ERROR("App Error - but I don' know from where :-(");
            break;
    }

    UNUSED_VARIABLE(m_error_data);
    
    // If printing is disrupted, remove the irq calls, or set the loop variable to 0 in the debugger.
    __disable_irq();
    while (loop);

    __enable_irq();
      assert(true==false);
}

void HardFault_Handler(void)
{
    uint32_t *sp = (uint32_t *) __get_MSP(); // Get stack pointer
    uint32_t ia = sp[12]; // Get instruction address from stack
    char buf[32];

    stopOLEDUpdates();
    sprintf(buf, "Hardfault: 0x%08x", (unsigned int)ia);
    oled_printStringAt(40,2, buf, true, true);
    printf("Hard Fault at address: 0x%08x\r\n", (unsigned int)ia);
    while(1)
        ;
}

void disconnectCentralForcefully(void)
{
    ret_code_t err_code = 0;
    
    if (m_central_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_INFO("Forcefully terminating connection to central/client");
        err_code = sd_ble_gap_disconnect(m_central_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code == BLE_ERROR_INVALID_CONN_HANDLE)
        {
            NRF_LOG_INFO("Connection handle was already invalid on sd_ble_gap_disconnect!");
        }
        else if (err_code == NRF_ERROR_INVALID_STATE)
        {
            NRF_LOG_INFO("Connection was in invalid state on sd_ble_gap_disconnect!");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

void self_configure()
{
// Startup config

    if (app_button_is_pushed(6)) // left steer, if pressed then DEactive FTMS, else (default) activate it
    {
        ;
    }
    else
    {
       
       ;
       
    }

    if (app_button_is_pushed(7)) // Right steer on startup, if pressed then active FTMS, else (default) FE-C
    {
        ble_ftms_active = false;
        ble_fec_active = true;
        oled_data.ftms_active = false;         
    }
    else
    {      
        ble_ftms_active = true;
        ble_fec_active = false;
        oled_data.ftms_active = true;           
    }
}

/**@brief Function for initializing the application main entry.
 */
int main(void)
{
    bool erase_bonds;
    ret_code_t err_code = 0;

    // Initialize.
    
    log_init();
    /*

    Check logging settings if no logging!

    // <e> NRF_LOG_ENABLED - nrf_log - Logger
    //==========================================================
    #ifndef NRF_LOG_ENABLED
    #ifdef DEBUG
        #define NRF_LOG_ENABLED 1
    #else
        #define NRF_LOG_ENABLED 0
    #endif
    #endif

    // Also check: #define NRF_LOG_DEFAULT_LEVEL 4
    */
        
    // Init DFU - must come AFTER DIRECTLY AFER log_init()!
    // https://devzone.nordicsemi.com/f/nordic-q-a/59881/advertising-rename-feature-not-working
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);

    // All other code here after
    cryptoutil_init();
    NRF_LOG_INFO("Using RTC prescaler of 1 leading to a rollover in timestamps after 17 min 03 seconds.");
       
    timer_init();    
    buttons_leds_init(&erase_bonds);
    hometrainer_control_init();
    power_management_init();
    ble_stack_init();
    
    oled_initialize();
   
    scan_init();
    gap_params_init();
    gatt_init();
    conn_params_init();
    db_discovery_init();
    peer_manager_init();
    
    self_configure();
    // bas_c_init(); // the multi link to two peripherals still doesn't work correctly.
    // dis_c_init();
    hrs_c_init();
    cscs_c_init();
    services_init();   
    advertising_init();

    // Start execution.
    NRF_LOG_INFO("nRF CPS relay started.");
    oled_printStringAt(FIRST_ROW,10, "Ride on!", true, true);

    oled_printStringAt(SECOND_ROW,10, "Go!", false, true);

    oled_printStringAt(THIRD_ROW,10, "Connecting CSCS...", false, true);

    oled_printStringAt(FOURTH_ROW,10, VERSION, false, true);

    if (ble_ftms_active)
    {
        oled_printStringAt(FIFTH_ROW,10, "FTMS: on, FE-C: off", false, true);
    }
    else
    {
        oled_printStringAt(FIFTH_ROW,10, "FTMS: off, FE-C: on", false, true);
    }

    
    oled_data.gear_ratio = getGearRatio();
    //gpio_init();
    
    // APP_ERROR_CHECK(1); // Test error handler app_error_fault_handler()

    if (erase_bonds == true)
    {
        // Scanning and advertising is done upon PM_EVT_PEERS_DELETE_SUCCEEDED event.
        delete_bonds();
    }
    else
    {
        adv_scan_start();
    }

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
#if BENCHMARK
        uint32_t execution_start, execution_end;
        execution_start = app_timer_cnt_get(); 
#endif // BENCHMARK

        if (gpio_finished)
        { 
          // NRF_LOG_INFO("GPIO sequence done");
          gpio_finished = false;
        }

        // Handle button presses
        if (gear_button_state != GEAR_BUTTON_PRESSED_NONE)
        {
          
            switch (gear_button_state)
            {
              case GEAR_BUTTON_PRESSED_RESET:
                NRF_LOG_INFO("GEAR_BUTTON_PRESSED_RESET");
                break;
              case GEAR_BUTTON_PRESSED_GEAR_UP:
                gear_offset++;
                checkGearBoundaries();
                NRF_LOG_INFO("Button TRAINER_BUTTON_GEAR_UP got pressed! New gear offset: %d", gear_offset);
                oled_data.gear = gear_offset;
                oled_data.gear_ratio = getGearRatio();
                oled_data.gear_up_counter = 2; // Display for 2s                
                ble_atom_send_gear_shift_notification(&m_atom, BLE_ATOM_GEAR_UP);
                // oled_printGear();
                break;
              case GEAR_BUTTON_PRESSED_GEAR_DOWN:
                gear_offset--;
                checkGearBoundaries();
                NRF_LOG_INFO("Button TRAINER_BUTTON_GEAR_DOWN got pressed! New gear offset: %d", gear_offset);               
                oled_data.gear = gear_offset;
                oled_data.gear_ratio = getGearRatio();
                oled_data.gear_down_counter = 2; // Display for 2s
                // notify Atom characteristic                
                ble_atom_send_gear_shift_notification(&m_atom, BLE_ATOM_GEAR_DOWN);                
                break;
              case GEAR_BUTTON_PRESSED_FRONT_GEAR_UP:
                NRF_LOG_INFO("Button TRAINER_BUTTON_FRONT_GEAR_UP got pressed! No action yet");               
                break;
              case GEAR_BUTTON_PRESSED_FRONT_GEAR_DOWN:
                NRF_LOG_INFO("Button TRAINER_BUTTON_FRONT_GEAR_UP got pressed! No action yet");               
                break;
              default:
                break;
            }
          
          // We purposefully overwrite the state, even if another key press happened
          // while we were handling this! Makes no sense to stack them further?
          gear_button_state = GEAR_BUTTON_PRESSED_NONE;
        }

        if (doCalcResistance)
        {
            resistance_change_needed_t res = checkResistanceChangeNeeded();
            /*
            switch (res)
            {
              case RESISTANCE_CHANGE_NEEDED_RESULT_BUSY:
                // NRF_LOG_INFO("BUSY");
                break;
              case RESISTANCE_CHANGE_NEEDED_RESULT_NO_CHANGE_NEEDED:
                // NRF_LOG_INFO("NO_CHANGE_NEEDED");
                break;
              case RESISTANCE_CHANGE_NEEDED_RESULT_PLUS_NEEDED:
                // NRF_LOG_INFO("PLUS_NEEDED"); 
                break;
              case RESISTANCE_CHANGE_NEEDED_RESULT_MINUS_NEEDED:
                NRF_LOG_INFO("MINUS_NEEDED");
                break;
              default:
                break;
            }
            */
            // NRF_LOG_INFO("Calced");
            if (requireResistancePlus)
            {
                triggerResistanceChange(TRIGGER_UP);
                requireResistancePlus = false;        
            }

            if (requireResistanceMinus)
            {
                //NRF_LOG_INFO("Resistance Level: %d", resistance_level);
                //NRF_LOG_INFO("Target Resistance Level: %d", target_resistance_level);
                triggerResistanceChange(TRIGGER_DOWN);
            
                requireResistanceMinus = false;
            }
            doCalcResistance = false;
        }

        

        if (doSteer)
        {
            //
            doSteer = false;
        }

        if (doUpdateOLED)
        {
            updateOLEDDisplay();
        }

        if (doInjectSimEvent)
        {
            injectSimEvent();
        }

        if (doCalcModel)
        {
            performModelCalculations();
            doCalcModel = false;
        }

        if (doTriggerFECEvent)
        {
            triggerFECEvent();
        }
#if BENCHMARK
        execution_end = app_timer_cnt_get();
        uint32_t timer_ticks = app_timer_cnt_diff_compute(execution_end,execution_start);
        char buf[16];
        snprintf(buf, sizeof(buf), "%.3f", (double) timer_ticks / 32.0);
        //NRF_LOG_INFO("main() loop (ms): %s", buf);
#endif
    }
}

// Pins in use:
//
// Bank P2
// P0.03 - DC (SPI)
// P0.04 - CS (SPI)
// P0.28 - CLK (SPI)
// P0.29 - DI (SPI)
// P0.31 - Reset (SPI)

// Bank P3
// P0.11  - Steer left (incoming)
// P0.12  - Steer right (incoming)

// Bank P4
// P0.22 - Gear shift UP (right side) (signal "up" to MCU, incoming)
// P0.23 - Gear shift DOWN (right side) (signal "down" to MCU, incoming)
// P0.24 - TRIGGER_RESISTANCE_UP_GPIO (send trigger up to hometrainer, outgoing)
// P0.25 - TRIGGER_RESISTANCE_DOWN_GPIO (send trigger down to hometrainer, outgoing)
// GAP HERE ON PCB!
// P0.26 - Front gear shift UP
// P0.27 - Front gear shift DOWN

// Potentially free on DK
// P0.2
// P0.30

 
// NOT free on DK
// P0.0 - GPIO physically disconnected, needs solder bridges
// P0.1 - GPIO physically disconnected, needs solder bridges
// P0.5 - used by NRF_LOG_BACKEND_SERIAL_USES_UART, free if not set
// P0.6 - used by NRF_LOG_BACKEND_SERIAL_USES_UART, free if not set
// P0.7 - used by NRF_LOG_BACKEND_SERIAL_USES_UART, free if not set
// P0.8 - used by NRF_LOG_BACKEND_SERIAL_USES_UART, free if not set
// P0.9  // In use by NFC, https://devzone.nordicsemi.com/f/nordic-q-a/20640/cannot-get-p0-09-to-operate-as-a-gpio
// P0.10 // In use by NFC, https://devzone.nordicsemi.com/f/nordic-q-a/20640/cannot-get-p0-09-to-operate-as-a-gpio

