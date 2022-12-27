

#ifndef BLE_CPS_H__
#define BLE_CPS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_cps_ctrlpt.h"
#include "ble_sensor_location.h"
#include "nrf_sdh_ble.h"
#include "ble_cscs_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_cscs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CPS_DEF(_name)                                                                         \
static ble_cps_t _name;                                                                            \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_CPS_BLE_OBSERVER_PRIO,                                                    \
                     ble_cps_on_ble_evt, &_name)

#define BLE_UUID_CYCLING_POWER 0x1818
#define BLE_UUID_CYCLING_POWER_MEASUREMENT 0x2A63
#define BLE_UUID_CYCLING_POWER_VECTOR 0x2A64 
#define BLE_UUID_CYCLING_POWER_FEATURE 0x2A65
#define BLE_UUID_CYCLING_POWER_CONTROLPOINT 0x2A66



/** @defgroup BLE_CPS_FEATURES Cycling Power Service feature bits
 * @{ */
#define BLE_CPS_FEATURE_PEDAL_POWER_BALANCE_BIT (0x01 << 0)     /**< Pedal Power Balance Data Supported bit. */
#define BLE_CPS_FEATURE_ACCUMULATED_TORQUE_BIT  (0x01 << 1)     /**< Accumulated Torque Data Supported bit. */
#define BLE_CPS_FEATURE_WHEEL_REV_BIT           (0x01 << 2)     /**< Wheel Revolution Data Supported bit. */
#define BLE_CPS_FEATURE_CRANK_REV_BIT           (0x01 << 3)     /**< Crank Revolution Data Supported bit. */
#define BLE_CPS_FEATURE_EXTREME_MAGNITUDES_BIT  (0x01 << 4)     /**< Extreme Magnitudes Data Supported bit. */
#define BLE_CPS_FEATURE_EXTREME_ANGLES_BIT      (0x01 << 5)     /**< Extreme Angles Data Supported bit. */
#define BLE_CPS_FEATURE_TOP_BOTTOM_DEAD_SPOT_BIT      (0x01 << 6)     /**< Top and Bottom dead spot Data Supported bit. */
#define BLE_CPS_FEATURE_ACCUMULATED_ENERGY_BIT      (0x01 << 7)     /**< Accumulated energy Data Supported bit. */
#define BLE_CPS_FEATURE_OFFSET_COMPENSATION_INDICATOR_BIT      (0x01 << 8)     /**< Offset compensation indicator Data Supported bit. */
#define BLE_CPS_FEATURE_OFFSET_COMPENSATION_BIT      (0x01 << 9)     /**< Offset compensation Data Supported bit. */
#define BLE_CPS_FEATURE_CONTENT_MASKING_BIT      (0x01 << 10)     /**< Content masking Supported bit. */
#define BLE_CPS_FEATURE_MULTIPLE_SENSOR_SUPPORTED_BIT      (0x01 << 11)     /**< Multiple sensor Supported bit. */
#define BLE_CPS_FEATURE_CRANK_LENGTH_ADJUSTMENT_BIT      (0x01 << 12)     /**< Crank Length adjustment Supported bit. */
#define BLE_CPS_FEATURE_CHAIN_LENGTH_ADJUSTMENT_BIT      (0x01 << 13)     /**< ChainLength adjustment Supported bit. */
// And more! https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.cycling_power_feature.xml










//#define BLE_CPS_FEATURE_MULTIPLE_SENSORS_BIT   (0x01 << 2)     /**< Multiple Sensor Locations Supported bit. */
/** @} */


/**@brief Cycling Speed and Cadence Service event type. */
typedef enum
{
    BLE_CPS_EVT_NOTIFICATION_ENABLED,                                  /**< Cycling Speed and Cadence value notification enabled event. */
    BLE_CPS_EVT_NOTIFICATION_DISABLED                                  /**< Cycling Speed and Cadence value notification disabled event. */
} ble_cps_evt_type_t;

/**@brief Cycling Power Service event. */
typedef struct
{
    ble_cps_evt_type_t evt_type;                                       /**< Type of event. */
} ble_cps_evt_t;

// Forward declaration of the ble_cps_t type.
typedef struct ble_cps_s ble_cps_t;

/**@brief Cycling Power Service event handler type. */
typedef void (*ble_cps_evt_handler_t) (ble_cps_t * p_cps, ble_cps_evt_t * p_evt);

/**@brief Cycling Power Service init structure. This contains all options and data
*         needed for initialization of the service. */
typedef struct
{
    ble_cps_evt_handler_t        evt_handler;                           /**< Event handler to be called for handling events in the Cycling Power Service. */
    security_req_t               cps_meas_cccd_wr_sec;                  /**< Security requirement for writing cycling power measurement characteristic CCCD. */
    security_req_t               cps_feature_rd_sec;                    /**< Security requirement for reading cycling power feature characteristic. */
    security_req_t               cps_location_rd_sec;                   /**< Security requirement for reading cycling power location characteristic. */
    security_req_t               cps_ctrlpt_cccd_wr_sec;                 /**< Security requirement for writing power control point characteristic CCCD. */
    security_req_t               cps_ctrlpt_wr_sec;                      /**< Security requirement for writing power control point characteristic. */
    uint16_t                     feature;                               /**< Initial value for features of sensor @ref BLE_CPS_FEATURES. */
    uint8_t                      ctrplt_supported_functions;            /**< Supported control point functionalities see @ref BLE_SRV_SC_CTRLPT_SUPP_FUNC. */
    ble_cps_ctrlpt_evt_handler_t  ctrlpt_evt_handler;                    /**< Event handler */
    ble_sensor_location_t        *list_supported_locations;             /**< List of supported sensor locations.*/
    uint8_t                      size_list_supported_locations;         /**< Number of supported sensor locations in the list.*/
    ble_srv_error_handler_t      error_handler;                         /**< Function to be called in case of an error. */
    ble_sensor_location_t        *sensor_location;                      /**< Initial Sensor Location, if NULL, sensor_location characteristic is not added*/
    volatile uint8_t                      *resistance_level;                         /**< Difficulty level for power calculation */
} ble_cps_init_t;

/**@brief Cycling Power Service structure. This contains various status information for
 *        the service. */
struct ble_cps_s
{
    ble_cps_evt_handler_t        evt_handler;                           /**< Event handler to be called for handling events in the Cycling Power Service. */
    uint16_t                     service_handle;                        /**< Handle of Cycling Power Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     meas_handles;                          /**< Handles related to the Cycling Power Measurement characteristic. */
    ble_gatts_char_handles_t     feature_handles;                       /**< Handles related to the Cycling Power feature characteristic. */
    ble_gatts_char_handles_t     sensor_loc_handles;                    /**< Handles related to the Cycling Power Sensor Location characteristic. */
    uint16_t                     conn_handle;                           /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t                     feature;                               /**< Bit mask of features available on sensor. */
    ble_cps_ctrlpt_t             ctrl_pt;                               /**< data for power control point */
    volatile uint8_t                      *resistance_level;                         /**< Current Incline level for power calculation */
};

/**@brief Cycling Power Service measurement structure. This contains a Cycling Power
 *        Service measurement. */
typedef struct ble_cps_meas_s
{
    bool        is_wheel_rev_data_present;                              /**< True if Wheel Revolution Data is present in the measurement. */
    bool        is_crank_rev_data_present;                              /**< True if Crank Revolution Data is present in the measurement. */
    int16_t     instantaneous_power;                                    /**< Instantaneous power */
    uint32_t    cumulative_wheel_revs;                                  /**< Cumulative Wheel Revolutions. */
    uint16_t    last_wheel_event_time;                                  /**< Last Wheel Event Time. */
    uint16_t    cumulative_crank_revs;                                  /**< Cumulative Crank Revolutions. */
    uint16_t    last_crank_event_time;                                  /**< Last Crank Event Time. */
} ble_cps_meas_t;

/*
typedef struct {
  ble_cscs_c_calculation_helper_t cscs_c;  
  double zPower;
} ble_cps_calculation_helper_t;
*/
/**@brief Function for initializing the Cycling Power Service.
 *
 * @param[out]  p_cps       Cycling Power Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_cps_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_cps_init(ble_cps_t * p_cps, ble_cps_init_t const * p_cps_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Cycling Power
 *          Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Cycling Power Service structure.
 */
void ble_cps_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending cycling power measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a Cycling Power
 *          Service measurement. If notification has been enabled, the measurement data is encoded
 *          and sent to the client.
 *
 * @param[in]   p_cps          Cycling Power Service structure.
 * @param[in]   p_measurement  Pointer to new cycling power measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_cps_measurement_send(ble_cps_t * p_cps, ble_cps_meas_t * p_measurement);


void ble_cps_on_cscs_evt(ble_cps_t * m_cps, ble_cscs_c_evt_t * p_cscs_c_evt);
//double cps_calc_instantaneous_cadence();
//double calc_instantaneous_cadence(ble_cscs_c_calculation_helper_t *h);
//double cps_calc_average_cadence();
//double calc_average_cadence(ble_cscs_c_calculation_helper_t *h);
//double cps_calc_average_kmh();
//double calc_average_kmh(ble_cscs_c_calculation_helper_t *h);


#ifdef __cplusplus
}
#endif

#endif // BLE_CPS_H__

/** @} */
