
#ifndef BLE_CSCS_C_H__
#define BLE_CSCS_C_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_ble_gq.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "oled_controller.h"


#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_cscs_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CSCS_C_DEF(_name)                                                                       \
static ble_cscs_c_t _name;                                                                          \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_CSCS_C_BLE_OBSERVER_PRIO,                                                  \
                     ble_cscs_c_on_ble_evt, &_name)

/** @brief Macro for defining multiple ble_cscs_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 * @hideinitializer
 */
#define BLE_CSCS_C_ARRAY_DEF(_name, _cnt)                 \
static ble_cscs_c_t _name[_cnt];                          \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                      \
                      BLE_CSCS_C_BLE_OBSERVER_PRIO,       \
                      ble_cscs_c_on_ble_evt, &_name, _cnt)

#define BLE_CSCS_WHEEL_REV_DATA_PRESENT     0x00  /**< Instantaneous Stride Length Measurement Supported bit. */
#define BLE_CSCS_CRANK_REV_DATA_PRESENT     0x01  /**< Total Distance Measurement Supported bit. */

// Shall we include free roll calculations?
#ifndef HANDLE_FREEROLL
#define HANDLE_FREEROLL 0
#endif

/**@brief   Structure containing the handles related to the Cycling Speed and Cadence Service found on the peer. */
typedef struct
{
    uint16_t csc_cccd_handle;                /**< Handle of the CCCD of the Cycling Speed and Cadence characteristic. */
    uint16_t csc_handle;                     /**< Handle of the Cycling Speed and Cadence characteristic as provided by the SoftDevice. */
} ble_cscs_c_db_t;

/**@brief   CSCS Client event type. */
typedef enum
{
    BLE_CSCS_C_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the Cycling Speed and Cadence Service has been discovered at the peer. */
    BLE_CSCS_C_EVT_CSC_NOTIFICATION         /**< Event indicating that a notification of the Cycling Speed and Cadence measurement characteristic has been received from the peer. */
} ble_cscs_c_evt_type_t;

/**@brief   Structure containing the Cycling Speed and Cadence measurement received from the peer. */
typedef struct
{
    bool        is_wheel_rev_data_present;                              /**< True if Wheel Revolution Data is present in the measurement. */
    bool        is_crank_rev_data_present;                              /**< True if Crank Revolution Data is present in the measurement. */
    uint32_t    cumulative_wheel_revs;                                  /**< Cumulative Wheel Revolutions. */
    uint16_t    last_wheel_event_time;                                  /**< Last Wheel Event Time. */
    uint16_t    cumulative_crank_revs;                                  /**< Cumulative Crank Revolutions. */
    uint16_t    last_crank_event_time;                         /**< Total Distance. */
} ble_csc_t;

/**@brief   Cycling Speed and Cadence Event structure. */
typedef struct
{
    ble_cscs_c_evt_type_t evt_type;  /**< Type of the event. */
    uint16_t  conn_handle;           /**< Connection handle on which the cscs_c event  occured.*/
    union
    {
        ble_cscs_c_db_t cscs_db;           /**< Cycling Speed and Cadence Service related handles found on the peer device. This is filled if the evt_type is @ref BLE_CSCS_C_EVT_DISCOVERY_COMPLETE.*/
        ble_csc_t       csc;               /**< Cycling Speed and Cadence measurement received. This is filled if the evt_type is @ref BLE_CSCS_C_EVT_RSC_NOTIFICATION. */
    } params;
} ble_cscs_c_evt_t;

// Forward declaration of the ble_cscs_c_t type.
typedef struct ble_cscs_c_s ble_cscs_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that is to be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_cscs_c_evt_handler_t) (ble_cscs_c_t * p_ble_cscs_c, ble_cscs_c_evt_t * p_evt);

/**@brief   Cycling Speed and Cadence client structure. */
struct ble_cscs_c_s
{
    uint16_t                 conn_handle;   /**< Connection handle as provided by the SoftDevice. */
    ble_cscs_c_db_t          peer_db;       /**< Handles related to CSCS on the peer*/
    ble_cscs_c_evt_handler_t evt_handler;   /**< Application event handler to be called when there is an event related to the Cycling Speed and Cadence service. */
    ble_srv_error_handler_t  error_handler; /**< Function to be called in case of an error. */
    nrf_ble_gq_t           * p_gatt_queue;  /**< Pointer to BLE GATT Queue instance. */
    bool                    sim_mode;
};

/**@brief   Cycling Speed and Cadence client initialization structure. */
typedef struct
{
    ble_cscs_c_evt_handler_t evt_handler;   /**< Event handler to be called by the Cycling Speed and Cadence Client module whenever there is an event related to the Cycling Speed and Cadence Service. */
    ble_srv_error_handler_t  error_handler; /**< Function to be called in case of an error. */
    nrf_ble_gq_t           * p_gatt_queue;  /**< Pointer to BLE GATT Queue instance. */
    bool                    sim_mode;
} ble_cscs_c_init_t;

/**@brief      Function for initializing the Cycling Speed and Cadence Service Client module.
 *
 * @details    This function will initialize the module and set up Database Discovery to discover
 *             the Cycling Speed and Cadence Service. After calling this function, call @ref ble_db_discovery_start
 *             to start discovery once a link with a peer has been established.
 *
 * @param[out] p_ble_cscs_c      Pointer to the CSC Service Client structure.
 * @param[in]  p_ble_cscs_c_init Pointer to the CSC Service initialization structure containing
 *                               the initialization information.
 *
 * @retval     NRF_SUCCESS      Operation success.
 * @retval     NRF_ERROR_NULL   A parameter is NULL.
 * @retval     err_code       	Otherwise, this function propagates the error code returned by @ref ble_db_discovery_evt_register.
 */
uint32_t ble_cscs_c_init(ble_cscs_c_t * p_ble_cscs_c, ble_cscs_c_init_t * p_ble_cscs_c_init);


/**@brief   Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack that are of interest to the Cycling Speed and Cadence
 *          Service Client.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Cycling Speed and Cadence Service Client structure.
 */
void ble_cscs_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


uint32_t ble_cscs_c_csc_notif_enable(ble_cscs_c_t * p_ble_cscs_c);


/**@brief   Function for handling events from the Database Discovery module.
 *
 * @details Call this function when you get a callback event from the Database Discovery module.
 *          This function handles an event from the Database Discovery module, and determines
 *          whether it relates to the discovery of Cycling Speed and Cadence Service at the peer.
 *          If it does, the function calls the application's event handler to indicate that the RSC Service was
 * 			discovered at the peer. The function also populates the event with service-related
 *          information before providing it to the application.
 *
 * @param     p_ble_cscs_c Pointer to the Cycling Speed and Cadence Service Client structure.
 * @param[in] p_evt Pointer to the event received from the Database Discovery module.
 */
void ble_cscs_on_db_disc_evt(ble_cscs_c_t * p_ble_cscs_c, ble_db_discovery_evt_t const * p_evt);


/**@brief   Function for assigning handles to this instance of cscs_c.
 *
 * @details Call this function when a link has been established with a peer to
 *          associate the link to this instance of the module. This makes it
 *          possible to handle several links and associate each link to a particular
 *          instance of this module. The connection handle and attribute handles are
 *          provided from the discovery event @ref BLE_CSCS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in]   p_ble_cscs_c    Pointer to the CSC client structure instance for associating the link.
 * @param[in]   conn_handle     Connection handle to associated with the given CSCS Client Instance.
 * @param[in]   p_peer_handles  Attribute handles on the CSCS server that you want this csc client
 *                              to interact with.
 */
uint32_t ble_cscs_c_handles_assign(ble_cscs_c_t    * p_ble_cscs_c,
                                   uint16_t          conn_handle,
                                   ble_cscs_c_db_t * p_peer_handles);

static void pid_handler(void * p_context);
//double cscs_calc_instantaneous_cadence();
//double cscs_calc_average_cadence();

//double cscs_calc_average_kmh();

// Shall we fake wheel data if non delivered by the sensor
#ifndef FAKE_WHEEL_DATA
#define FAKE_WHEEL_DATA 1
#endif

// Shall we approach precise measurement
#ifndef PRECISE_MEASUREMENT
#define PRECISE_MEASUREMENT 1
#endif

// Shall we do extensive logging
#ifndef DEBUG_LOGGING
#define DEBUG_LOGGING 0
#endif

#ifndef CALC_LOGGING
#define CALC_LOGGING 1
#endif

// Shall we do CSV logging
#ifndef CSV_LOGGING
#define CSV_LOGGING 1
#define CSV_LINE_LENGTH (10+5+5+5)*3+3+6+5+1+15+1 // Measurements: 10 (cumwheelrev), 5 (lastweheeleventtime), 5(cumcrankrev), 5(lastcranktime) - 3 times, 3 (cadence), 6 (km/h, ddd.dd), 5 gear ratio (dd.dd), 1 cached, 15 (fieldseperators), 1(\n)
#endif

//#ifndef GEAR_RATIO
//#define GEAR_RATIO 2.58
//#endif


#define PID_CONTROLLER_INTERVAL 1000

#ifndef HONOR_ZERO_DELTA
#define HONOR_ZERO_DELTA 0
#endif


extern oled_data_t oled_data;
#ifdef __cplusplus
}
#endif

#endif // BLE_CSCS_C_H__

/** @} */ // End tag for the file.
