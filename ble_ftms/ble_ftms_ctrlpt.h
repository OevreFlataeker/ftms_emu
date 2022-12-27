

#ifndef BLE_FTMS_CTRLPT_H__
#define BLE_FTMS_CTRLPT_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_sensor_location.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_FTMS_CTRLPT_MAX_LEN                                      19                     /**< maximum lenght for Speed and cadence control point characteristic value. */
#define BLE_FTMS_CTRLPT_MIN_LEN                                      1                      /**< minimum length for Speed and cadence control point characteristic value. */
#define BLE_UUID_FTMS_CTRLPT_CHAR                                  0x2AD9 
// Forward declaration of the ble_sc_ctrlpt_t type.
typedef struct ble_ftms_ctrlpt_s ble_ftms_ctrlpt_t;

typedef enum {
    BLE_FTMSPT_STOP_PAUSE_PARAM_STOP        = 0x01,
    BLE_FTMSPT_STOP_PAUSE_PARAM_PAUSE       = 0x02
} ble_ftmspt_stop_pause_param_t;
/**@brief Speed and Cadence Control Point event type. */
typedef enum
{
    BLE_FTMS_CTRLPT_EVT_REQUEST_CONTROL,                                                    /**< rcvd update location opcode (the control point handles the change of location automatically, the event just informs the application in case it needs to adjust its algorithm). */
    BLE_FTMS_CTRLPT_EVT_RESET,                                                    /**< rcvd set cumulative value opcode, it is then up to the application to use the new cumulative value. */
    BLE_FTMS_CTRLPT_EVT_SET_TARGET_INCLINATION,                                                  /**< rcvd start calibration opcode, the application needs, at the end ot the calibration to call ble_sc_ctrlpt_send_rsp. */
    BLE_FTMS_CTRLPT_EVT_SET_TARGET_POWER,                                                  /**< rcvd start calibration opcode, the application needs, at the end ot the calibration to call ble_sc_ctrlpt_send_rsp. */
    BLE_FTMS_CTRLPT_EVT_SET_TARGET_RESISTANCE,                                                  /**< rcvd start calibration opcode, the application needs, at the end ot the calibration to call ble_sc_ctrlpt_send_rsp. */
    BLE_FTMS_CTRLPT_EVT_START_RESUME,
    BLE_FTMS_CTRLPT_EVT_STOP_PAUSE,
    BLE_FTMS_CTRLPT_EVT_SET_INDOOR_TRAINING_BIKE_PARAMETERS,
    BLE_FTMS_CTRLPT_EVT_SET_TARGETED_CADENCE
} ble_ftms_ctrlpt_evt_type_t;

//

#define BLE_SRV_FTMS_CTRLPT_REQUEST_CONTROL_SUPPORTED  0x01                             /**< Support for sensor location related operations */
#define BLE_SRV_FTMS_CTRLPT_RESET_SUPPORTED           0x02                             /**< Support for setting cumulative value related operations */
#define BLE_SRV_FTMS_CTRLPT_SET_TARGET_INCLINATION_SUPPORTED       0x04                             /**< Support for starting calibration related operations */
#define BLE_SRV_FTMS_CTRLPT_SET_TARGET_POWER_SUPPORTED       0x08                             /**< Support for starting calibration related operations */
#define BLE_SRV_FTMS_CTRLPT_SET_TARGET_RESISTANCE_SUPPORTED       0x10                             /**< Support for starting calibration related operations */
#define BLE_SRV_FTMS_CTRLPT_START_RESUME_SUPPORTED       0x20                             /**< Support for starting calibration related operations */
#define BLE_SRV_FTMS_CTRLPT_STOP_PAUSE_SUPPORTED       0x40                             /**< Support for starting calibration related operations */
#define BLE_SRV_FTMS_CTRLPT_SET_TARGETED_CADENCE_SUPPORTED       0x80                             /**< Support for starting calibration related operations */

//


typedef struct{
    int16_t wind_speed;
    int16_t grade;
    uint8_t crr;
    uint8_t cw;
} ble_ftms_indoor_bike_simulation_parameters_t;


/**@brief Speed and Cadence Control point event. */
typedef struct
{
    ble_ftms_ctrlpt_evt_type_t evt_type;                                                    /**< Type of event. */
    union
    {
        int16_t target_inclination;
        int16_t target_power;
        uint8_t target_resistance;
        uint16_t target_cadence;
        ble_ftmspt_stop_pause_param_t stop_pause_value;
        ble_ftms_indoor_bike_simulation_parameters_t indoor_bike_simulation_parameters;
    }params;
} ble_ftms_ctrlpt_evt_t;


/** Finess Machine Control Point operator code  (see FTMS service specification)*/
typedef enum {
    BLE_FTMSPT_REQUEST_CONTROL                        = 0x00,
    BLE_FTMSPT_RESET                                    = 0x01,                               /**< Operator to set a given cumulative value. */
    BLE_FTMSPT_SET_TARGET_SPEED            = 0x02,                               /**< Operator to start automatic calibration. */
    BLE_FTMSPT_SET_TARGET_INCLINATION                 = 0x03,                               /**< Operator to update the sensor location. */
    BLE_FTMSPT_SET_TARGET_RESISTANCE     = 0x04,                               /**< Operator to request the supported sensor locations. */
    BLE_FTMSPT_SET_TARGET_POWER                          = 0x05,                               /**< Response Code. */
    BLE_FTMSPT_SET_TARGET_HEARTRATE                          = 0x06,
    BLE_FTMSPT_START_RESUME                     = 0x07,
    BLE_FTMSPT_STOP_PAUSE                     = 0x08,
    BLE_FTMSPT_SET_TARGETED_EXPENDED_ENERGY = 0x09,
    BLE_FTMSPT_SET_TARGETED_NUMBER_OF_STEPS = 0x0A,
    BLE_FTMSPT_SET_TARGETED_NUMBER_OF_STRIDES = 0x0B,
    BLE_FTMSPT_SET_TARGETED_DISTANCE = 0x0C,
    BLE_FTMSPT_SET_TARGETED_TRAINING_TIME = 0x0D,
    BLE_FTMSPT_SET_TARGETED_TIME_IN_TWO_HEART_RATE_ZONES = 0x0E,
    BLE_FTMSPT_SET_TARGETED_TIME_IN_THREE_HEART_RATE_ZONES = 0x0F,
    BLE_FTMSPT_SET_TARGETED_TIME_IN_FIVE_HEART_RATE_ZONES = 0x10,
    BLE_FTMSPT_SET_INDOOR_TRAINING_BIKE_SIMULATION_PARAMETERS = 0x11,
    BLE_FTMSPT_SET_WHEEL_CIRCUMFENCE = 0x12,
    BLE_FTMSPT_SPIN_DOWN_CONTROL = 0x13,
    BLE_FTMSPT_SET_TARGETED_CADENCE = 0x14,
    BLE_FTMSPT_RESPONSE_CODE = 0x80
} ble_ftmspt_operator_t;


/** Speed and Cadence Control Point response parameter  (see RSC service specification)*/
typedef enum {
    BLE_FTMSPT_SUCCESS                                = 0x01,                               /**< Sucess Response. */
    BLE_FTMSPT_OP_CODE_NOT_SUPPORTED                  = 0x02,                               /**< Error Response received opcode not supported. */
    BLE_FTMSPT_INVALID_PARAMETER                      = 0x03,                               /**< Error Response received parameter invalid. */
    BLE_FTMSPT_OPERATION_FAILED                       = 0x04,                               /**< Error Response operation failed. */
    BLE_FTMSPT_CONTROL_NOT_PERMITTED                  = 0x05
} ble_ftmspt_response_t;


    

// TODO Didn' find yet the proper values for FTMS

/** Speed and Cadence Control Point procedure status (indicates is a procedure is in progress or not and which procedure is in progress*/
typedef enum {
    BLE_FTMSPT_NO_PROC_IN_PROGRESS                    = 0x00,                               /**< No procedure in progress. */
    BLE_FTMSPT_AUTOMATIC_CALIB_IN_PROGRESS            = 0x01,                               /**< Automatic Calibration is in progress. */
    BLE_FTMSPT_INDICATION_PENDING                     = 0x02,                               /**< Control Point Indication is pending. */
    BLE_FTMSPT_IND_CONFIRM_PENDING                    = 0x03,                               /**< Waiting for the indication confirmation. */
}ble_ftmspt_procedure_status_t;

/**@brief Speed and Cadence Control point event handler type. */
typedef ble_ftmspt_response_t (*ble_ftms_ctrlpt_evt_handler_t) (ble_ftms_ctrlpt_t * p_ftms_ctrlpt,
                                             ble_ftms_ctrlpt_evt_t * p_evt);



typedef struct{
    ble_ftmspt_operator_t opcode;
    int16_t              target_inclination;
    int16_t              target_power;
    uint8_t              target_resistance;
    uint16_t             target_cadence;
    ble_ftmspt_stop_pause_param_t              stop_pause_value;    
    ble_ftms_indoor_bike_simulation_parameters_t simulation_parameters;
}ble_ftms_ctrlpt_val_t;


typedef struct{
    ble_ftmspt_operator_t   opcode;
    ble_ftmspt_response_t   status;
    //ble_sensor_location_t location_list[BLE_NB_MAX_SENSOR_LOCATIONS];
}ble_ftms_ctrlpt_rsp_t;


/**
 * \defgroup BLE_SRV_SC_CTRLPT_SUPP_FUNC Control point functionalities.
 *@{
 */
#define BLE_SRV_FTMS_CTRLPT_REQUEST_CONTROL_SUPPORTED  0x01                             /**< Support for sensor location related operations */
#define BLE_SRV_FTMS_CTRLPT_RESET_SUPPORTED           0x02                             /**< Support for setting cumulative value related operations */
#define BLE_SRV_FTMS_CTRLPT_SET_TARGET_INCLINATION_SUPPORTED       0x04                             /**< Support for starting calibration related operations */
#define BLE_SRV_FTMS_CTRLPT_SET_TARGET_POWER_SUPPORTED       0x08                             /**< Support for starting calibration related operations */
#define BLE_SRV_FTMS_CTRLPT_SET_TARGET_RESISTANCE_SUPPORTED       0x10                             /**< Support for starting calibration related operations */
#define BLE_SRV_FTMS_CTRLPT_START_RESUME_SUPPORTED       0x20                             /**< Support for starting calibration related operations */
#define BLE_SRV_FTMS_CTRLPT_STOP_PAUSE_SUPPORTED       0x40                             /**< Support for starting calibration related operations */
#define BLE_SRV_FTMS_CTRLPT_SET_INDOOR_TRAINING_BIKE_PARAMETERS_SUPPORTED 0x80
// ...



/**
  *@}
  */

/**@brief Speed and Cadence Control Point init structure. This contains all options and data
*         needed for initialization of the Speed and Cadence Control Point module. */
typedef struct
{
    security_req_t               ftms_ctrlpt_cccd_wr_sec;                                   /**< Security requirement for writing speed and cadence control point characteristic CCCD. */
    security_req_t               ftms_ctrlpt_wr_sec;                                        /**< Security requirement for writing speed and cadence control point characteristic. */
    uint8_t                      supported_functions;                                     /**< supported control point functionalities see @ref BLE_SRV_SC_CTRLPT_SUPP_FUNC. */
    uint16_t                     service_handle;                                          /**< Handle of the parent service (as provided by the BLE stack). */
    ble_ftms_ctrlpt_evt_handler_t  evt_handler;                                             /**< event handler */
    uint16_t                    trainer_status_handle;
    ble_srv_error_handler_t      error_handler;  
                                             uint8_t *                        trainer_started;/**< Function to be called in case of an error. */
                                             uint8_t * control_acquired;
} ble_ftms_ctrlpt_init_t;


/**@brief Speed and Cadence Control Point response indication structure. */
typedef struct
{
    ble_ftmspt_response_t          status;                                                  /**< control point response status .*/
    uint8_t                      len;                                                     /**< control point response length .*/
    uint8_t                      encoded_ctrl_rsp[BLE_FTMS_CTRLPT_MAX_LEN];                 /**< control point encoded response.*/
}ble_ftms_ctrlpt_resp_t;


/**@brief Speed and Cadence Control Point structure. This contains various status information for
 *        the Speed and Cadence Control Point behavior. */
struct ble_ftms_ctrlpt_s
{
    uint8_t                      supported_functions;                                     /**< supported control point functionalities see @ref BLE_SRV_SC_CTRLPT_SUPP_FUNC. */
    uint16_t                     service_handle;                                          /**< Handle of the parent service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     ftms_ctrlpt_handles;                                       /**< Handles related to the Speed and Cadence Control Point characteristic. */
    uint16_t                     conn_handle;                                             /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    ble_ftms_ctrlpt_evt_handler_t  evt_handler;                                             /**< Handle of the parent service (as provided by the BLE stack). */
    uint16_t                     trainer_status_handle;
    ble_ftmspt_procedure_status_t  procedure_status;                                        /**< status of possible procedure*/
    ble_srv_error_handler_t        error_handler;                                           /**< Function to be called in case of an error. */
    ble_ftms_ctrlpt_resp_t         response;                                                /**< pending response data.*/    
    uint8_t *                      control_acquired;
    uint8_t *                        trainer_started;
};

#define FTMSPT_OPCODE_POS                   0                                               /**< Request opcode position. */
#define FTMSPT_PARAMETER_POS                1                                               /**< Request parameter position. */

#define FTMSPT_RESPONSE_REQUEST_OPCODE_POS  1                                               /**< Response position of requested opcode. */
#define FTMSPT_RESPONSE_CODE_POS            2                                               /**< Response position of response code. */
#define FTMSPT_RESPONSE_PARAMETER           3                                               /**< Response position of response parameter. */

#define FTMSPT_MIN_RESPONSE_SIZE            3                                               /**< Minimum size for control point response. */
#define FTMSPT_MAX_RESPONSE_SIZE  (FTMSPT_MIN_RESPONSE_SIZE + NB_MAX_SENSOR_LOCATIONS)        /**< Maximum size for control point response. */


/**@brief Function for Initializing the Speed and Cadence Control Point.
 *
 * @details Function for Initializing the Speed and Cadence Control Point.
 * @param[in]   p_sc_ctrlpt   Speed and Cadence Control Point structure.
 * @param[in]   p_sc_ctrlpt_init   Information needed to initialize the control point behavior.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_ftms_ctrlpt_init(ble_ftms_ctrlpt_t            * p_ftms_ctrlpt,
                            ble_ftms_ctrlpt_init_t const * p_ftms_ctrlpt_init);


/**@brief Function for sending a control point response.
 *
 * @details Function for sending a control point response when the control point received was
 *          BLE_SCPT_START_AUTOMATIC_CALIBRATION. To be called after the calibration procedure is finished.
 *
 * @param[in]   p_sc_ctrlpt      Speed and Cadence Control Point structure.
 * @param[in]   response_status  status to include in the control point response.
 */
uint32_t ble_ftms_ctrlpt_rsp_send(ble_ftms_ctrlpt_t * p_ftms_ctrlpt, ble_ftmspt_response_t response_status);


/**@brief Speed and Cadence Control Point BLE stack event handler.
 *
 * @details Handles all events from the BLE stack of interest to the Speed and Cadence Control Point.
 *
 * @param[in]   p_sc_ctrlpt   Speed and Cadence Control Point structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_ftms_ctrlpt_on_ble_evt(ble_ftms_ctrlpt_t * p_ftms_ctrlpt, ble_evt_t const * p_ble_evt);



#ifdef __cplusplus
}
#endif

#endif // BLE_FTMS_CTRLPT_H__

/** @} */
