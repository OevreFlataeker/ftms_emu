
#ifndef BLE_ATOM_CTRLPT_H__
#define BLE_ATOM_CTRLPT_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_ATOM_CTRLPT_MAX_LEN                                      19                     /**< maximum lenght for Speed and cadence control point characteristic value. */
#define BLE_ATOM_CTRLPT_MIN_LEN                                      1                      /**< minimum length for Speed and cadence control point characteristic value. */

// Forward declaration of the ble_sc_ctrlpt_t type.
typedef struct ble_atom_ctrlpt_s ble_atom_ctrlpt_t;

/**@brief Speed and Cadence Control Point event type. */
typedef enum
{
    BLE_ATOM_CTRLPT_EVT_AUTHENTICATE,
    BLE_ATOM_CTRLPT_EMIT_RESPONSE,
    BLE_ATOM_START_SENDING_STEERING_DATA
} ble_atom_ctrlpt_evt_type_t;

//

#define BLE_SRV_STERZO_CTRLPT_AUTHENTICATE_SUPPORTED  0x01                             /**< Support for sensor location related operations */

/**@brief Speed and Cadence Control point event. */
typedef struct
{
    ble_atom_ctrlpt_evt_type_t evt_type;                                                    /**< Type of event. */
    union
    {
        int16_t some_value;}params;
} ble_atom_ctrlpt_evt_t;



/** Speed and Cadence Control Point response parameter  (see RSC service specification)*/
typedef enum {
    BLE_ATOMPT_SUCCESS                                = 0x01,                               /**< Sucess Response. */
    BLE_ATOMPT_OP_CODE_NOT_SUPPORTED                  = 0x02,                               /**< Error Response received opcode not supported. */
    BLE_ATOMPT_INVALID_PARAMETER                      = 0x03,                               /**< Error Response received parameter invalid. */
    BLE_ATOMPT_OPERATION_FAILED                       = 0x04,                               /**< Error Response operation failed. */
    BLE_ATOMPT_CONTROL_NOT_PERMITTED                  = 0x05,
    BLE_ATOMPT_ALREADY_AUTHENTICATED                  = 0x06, // Needs change
} ble_atompt_response_t;


    

// TODO Didn' find yet the proper values for FTMS

/** Speed and Cadence Control Point procedure status (indicates is a procedure is in progress or not and which procedure is in progress*/
typedef enum {
    BLE_ATOMPT_NO_PROC_IN_PROGRESS                    = 0x00,                               /**< No procedure in progress. */
    BLE_ATOMPT_AUTHENTICATION_IN_PROGRESS            = 0x01,                               /**< Automatic Calibration is in progress. */
    BLE_ATOMPT_INDICATION_PENDING                     = 0x02,                               /**< Control Point Indication is pending. */
    BLE_ATOMPT_IND_CONFIRM_PENDING                    = 0x03,                               /**< Waiting for the indication confirmation. */
}ble_atompt_procedure_status_t;

/**@brief Speed and Cadence Control point event handler type. */
typedef ble_atompt_response_t (*ble_atom_ctrlpt_evt_handler_t) (ble_atom_ctrlpt_t * p_atom_ctrlpt,
                                             ble_atom_ctrlpt_evt_t * p_evt);

typedef enum {
    BLE_ATOMPT_AUTHENTICATE                        = 0x01,
    BLE_ATOMPT_RESPONSE_CODE = 0x80 // Needs change!
} ble_atompt_operator_t;

typedef struct{
    ble_atompt_operator_t opcode;
    int16_t              someval;
}ble_atom_ctrlpt_val_t;


typedef struct{
    ble_atompt_operator_t   opcode;
    ble_atompt_response_t   status;
    //ble_sensor_location_t location_list[BLE_NB_MAX_SENSOR_LOCATIONS];
}ble_atom_ctrlpt_rsp_t;


/**
 * \defgroup BLE_SRV_SC_CTRLPT_SUPP_FUNC Control point functionalities.
 *@{
 */
#define BLE_SRV_STERZO_CTRLPT_AUTHENTICATION_SUPPORTED  0x01                             /**< Support for sensor location related operations */

/**
  *@}
  */

/**@brief Speed and Cadence Control Point init structure. This contains all options and data
*         needed for initialization of the Speed and Cadence Control Point module. */
typedef struct
{
    security_req_t               sterzo_ctrlpt_cccd_wr_sec;                                   /**< Security requirement for writing speed and cadence control point characteristic CCCD. */
    security_req_t               sterzo_ctrlpt_wr_sec;                                        /**< Security requirement for writing speed and cadence control point characteristic. */
    uint8_t                      supported_functions;                                     /**< supported control point functionalities see @ref BLE_SRV_SC_CTRLPT_SUPP_FUNC. */
    uint16_t                     service_handle;                                          /**< Handle of the parent service (as provided by the BLE stack). */
    ble_atom_ctrlpt_evt_handler_t  evt_handler;                                             /**< event handler */
    ble_srv_error_handler_t      error_handler; 
     uint8_t authenticated;
     uint8_t service_uuid_type;
} ble_atom_ctrlpt_init_t;

/**@brief Speed and Cadence Control Point response indication structure. */
typedef struct
{
    ble_atompt_response_t          status;                                                  /**< control point response status .*/
    uint8_t                      len;                                                     /**< control point response length .*/
    uint8_t                      encoded_ctrl_rsp[BLE_ATOM_CTRLPT_MAX_LEN];                 /**< control point encoded response.*/
}ble_atom_ctrlpt_resp_t;


/**@brief Speed and Cadence Control Point structure. This contains various status information for
 *        the Speed and Cadence Control Point behavior. */
struct ble_atom_ctrlpt_s
{
    uint8_t                      supported_functions;                                     /**< supported control point functionalities see @ref BLE_SRV_SC_CTRLPT_SUPP_FUNC. */
    uint16_t                     service_handle;                                          /**< Handle of the parent service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     sterzo_ctrlpt_handles;                                       /**< Handles related to the Speed and Cadence Control Point characteristic. */
    uint16_t                     conn_handle;                                             /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    ble_atom_ctrlpt_evt_handler_t  evt_handler;                                             /**< Handle of the parent service (as provided by the BLE stack). */
    ble_atompt_procedure_status_t  procedure_status;                                        /**< status of possible procedure*/
    ble_srv_error_handler_t        error_handler;                                           /**< Function to be called in case of an error. */
    ble_atom_ctrlpt_resp_t         response;                                                /**< pending response data.*/    
    uint8_t *                     authenticated;
    ble_atom_t *                 p_atom;
};

#define STERZOPT_OPCODE_POS                   0                                               /**< Request opcode position. */
#define STERZOPT_PARAMETER_POS                1                                               /**< Request parameter position. */

#define STERZOPT_RESPONSE_REQUEST_OPCODE_POS  1                                               /**< Response position of requested opcode. */
#define STERZOPT_RESPONSE_CODE_POS            2                                               /**< Response position of response code. */
#define STERZOPT_RESPONSE_PARAMETER           3                                               /**< Response position of response parameter. */

#define STERZOPT_MIN_RESPONSE_SIZE            3                                               /**< Minimum size for control point response. */
#define STERZOPT_MAX_RESPONSE_SIZE  (STERZOPT_MIN_RESPONSE_SIZE + NB_MAX_SENSOR_LOCATIONS)        /**< Maximum size for control point response. */


/**@brief Function for Initializing the Speed and Cadence Control Point.
 *
 * @details Function for Initializing the Speed and Cadence Control Point.
 * @param[in]   p_sc_ctrlpt   Speed and Cadence Control Point structure.
 * @param[in]   p_sc_ctrlpt_init   Information needed to initialize the control point behavior.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_atom_ctrlpt_init(ble_atom_ctrlpt_t            * p_atom_ctrlpt,
                            ble_atom_ctrlpt_init_t const * p_atom_ctrlpt_init);


/**@brief Function for sending a control point response.
 *
 * @details Function for sending a control point response when the control point received was
 *          BLE_SCPT_START_AUTOMATIC_CALIBRATION. To be called after the calibration procedure is finished.
 *
 * @param[in]   p_sc_ctrlpt      Speed and Cadence Control Point structure.
 * @param[in]   response_status  status to include in the control point response.
 */
uint32_t ble_atom_ctrlpt_rsp_send(ble_atom_ctrlpt_t * p_atom_ctrlpt, ble_atompt_response_t response_status);


/**@brief Speed and Cadence Control Point BLE stack event handler.
 *
 * @details Handles all events from the BLE stack of interest to the Speed and Cadence Control Point.
 *
 * @param[in]   p_sc_ctrlpt   Speed and Cadence Control Point structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_atom_ctrlpt_on_ble_evt(ble_atom_ctrlpt_t * p_atom_ctrlpt, ble_evt_t const * p_ble_evt);

//void on_ctrlpt_write(ble_atom_ctrlpt_t * p_atom_ctrlpt,ble_gatts_evt_write_t const * p_ble_evt);

typedef ble_atompt_response_t (*ble_atom_ctrlpt_evt_handler_t) (ble_atom_ctrlpt_t * p_atom_ctrlpt,
                                             ble_atom_ctrlpt_evt_t * p_evt);



#ifdef __cplusplus
}
#endif

#endif // BLE_FTMS_CTRLPT_H__

/** @} */
