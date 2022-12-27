/**
 * Copyright (c) 2020 daubsi
*/

#ifndef BLE_FTMS_H__
#define BLE_FTMS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_ftms_ctrlpt.h"
#include "ble_sensor_location.h"
#include "nrf_sdh_ble.h"
#include "ble_cscs_c.h"
#include "ble_cps.h"
#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_cscs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_FTMS_DEF(_name)                                                                         \
static ble_ftms_t _name;                                                                            \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_FTMS_BLE_OBSERVER_PRIO,                                                    \
                     ble_ftms_on_ble_evt, &_name)


#define BLE_UUID_FITNESS_MACHINE 0x1826  // The service itself
#define BLE_UUID_FITNESS_MACHINE_FEATURE 0x2acc // The features
#define BLE_UUID_FITNESS_MACHINE_STATUS 0x2ada  // This only delivers the status
#define BLE_UUID_FITNESS_MACHINE_CONTROLPOINT 0x2ad9 // This is our interface for control
#define BLE_UUID_FITNESS_MACHINE_INDOOR_BIKE 0x2ad2  // This is our "usual" measurement
#define BLE_UUID_FITNESS_MACHINE_TRAINING_STATUS 0x2ad3  
#define BLE_UUID_SUPPORTED_POWER_RANGE 0x2ad8 
#define BLE_UUID_SUPPORTED_RESISTANCE_RANGE 0x2ad6
#define BLE_UUID_SUPPORTED_INCLINATION_RANGE 0x2ad5


/*
 Also
   Indoor Bike Data Characteristic/Type, subscriptable

   Training Status Characteristic, check/ Same "level" as Indoof Bike Data, subscriptable

   Supported Inclination Range https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.supported_inclination_range.xml
   Supported Resistance Range https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.supported_resistance_level_range.xml
   Supported Power Range https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.supported_power_range.xml
   Fitness Machine Control Point - check
   Fitness Machine Status - check
         In the context of an indoor bike, a Data Record will be sent with one or more notifications of the Indoor 
         Bike Data characteristic defined in Section 4.9. 
         For devices that support the low energy feature of Bluetooth, if a Data Record exceeds the ATT_MTU 
        size, it shall be transmitted in several notifications. Refer to Section 4.18 for additional requirements on 
         time-sensitive data. 

 Training session:
 Training Session is started via Control Point -> Start or Resume
 Training Session is stopped via Control Point -> Pause or Stop
 -> 4.10.1

 Fitness Machine Feature Characteristic:, 4.3.1
UUID: 0x2ACC
https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.fitness_machine_feature.xml
2x32bit
 Sends Fitness Machine Features and Target Setting Features

Indoor BIke:
https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.indoor_bike_data.xml
2AD2

 Features
 0 = False, 1 = True
 Bit 0: Average Speed Supported
 Bit 1: Cadence Supported
 Bit 2: Total Distance Supported
 Bit 3: Inclination supported --> Needs to be advertised in service characteristics!
 ..
 Bit 7: Resistance Level Supported --> --> Needs to be advertised in service characteristics!
 Bit 10: HRS supported
 ..
 Bit 14: Power Measurement supported

 Target Settigns:
 0 = False, 1 = True
 Bit 0: Speed Target supported
 Bit 1: Inclination Target supported
 Bit 2: Resistance Target supported
 Bit 3: Power Target supported
 ..
 Bit 13: Indoor Bike Simulation Parameters Supported
 ...

Indoor Bike Data 4.9
The Indoor Bike Data characteristic is used to send training-related data to the Client from an indoor bike 
(Server). Included in the characteristic value is a Flags field (for showing the presence of optional fields), 
and depending upon the contents of the Flags field, it may include one or more optional fields as defined 

4.9.1 Characteristic Behavior 
When the Indoor Bike Data characteristic is configured for notification via the Client Characteristic 
Configuration descriptor and training-related data is available, this characteristic shall be notified. The 
Server should notify this characteristic at a regular interval, typically once per second while in a 
connection and the interval is not configurable by the Client. 
For low energy, all the fields of this characteristic cannot be present simultaneously if using a default 
ATT_MTU size. Refer to Sections 4.1 and 4.19 for additional requirements on the transmission of a Data 
Record in multiple notifications. Refer to Section 4.18 for additional requirements on time-sensitive data. 

Flags Field, 4.9.1.1 (16 bit?)
Has 1/0 bits depending on whether the info is present (nothing new)

4.9.1.2 Instantaneous Speed Field 
The Instantaneous Speed field SHALL be included in the indoor bike-related Data Record. If the Data 
Record is split into several notifications of the Indoor Bike Data, this field shall only be included in the 
Indoor Bike Data characteristic when the More Data bit of the Flags field is set to 0. Refer to Section 4.19 
for additional information related to the transmission of a Data Record. 

4.9.1.4 Instantaneous Cadence Field 
The Instantaneous Cadence field MAY be included
supports the Cadence feature (see Table 4.10). 

Rest all MAY!

---

Training Status characteristic:
https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.training_status.xml
Included in the characteristic value is a Flags field (for showing the presence of optional fields), 
a Training Status field, and depending upon the contents of the Flags field, also a Training Status String. 

When the Training Status characteristic is configured Trfor notification
Configuration descriptor and a new training status is available
training program), this characteristic shall be notified. 

Flags Field:
    Bit 0: Training Status Present
    Bit 1: Ext Training Status Present
 

Training Status:
    1 Byte Field to identify type (Warmup, etc.)

Training Status String: MAY be sent with additional description

--

4.12 Supported Inclination Range 
The Supported Inclination Range characteristic shall be exposed by the Server if the Inclination Target 
Setting feature is supported. 
The Supported Inclination Range characteristic is used to send the supported inclination range as well as 
the minimum inclination increment supported by the Server. Included in the characteristic value are a 
Minimum Inclination field, a Maximum Inclination field, and a Minimum Increment field as defined on the 
Bluetooth SIG Assigned Numbers webpage [2]. 
4.12.1 Characteristic Behavior 
When read, the Supported Inclination Range characteristic returns a value that is used by a Client to 
determine the valid range that can be used in order to control the inclination of the Server. 

--
Same for supported resistance range, power range, etc.

---

Fitness Machine Controlpoint
https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.fitness_machine_control_point.xml
UUID: 0x2AD9

Uses to execute a function on the server

Consists of Opcodes and Parameters

Opcode:

0x00 Mandatory, Request Control, 
  Response: 0x80 followed by appropriate param value

0x01 Mandatory, Reset Settings
  Response: 0x80 followed by appropriate param value

0x03 Optional, Set Incline, SINT16 in Percent with 0.1% resolution, value as param
  Response: 0x80 followed by appropriate param value

0x04 Optional, Set Target Resistance, UINT8, 0.1 resolution, value as param
  Response: 0x80 followed by appropriate param value

0x05 Optional, Set Target Power, SINT16 in Watt 1 W resolution, value as param
  Response: 0x80 followed by appropriate param value

..

0x07 Mandatory, Start/Resume
  Response: 0x80 followed by appropriate param value

0x08 Mandatory, Stop/Pause, the param is 1 for stop, 2 for pause
  Response: 0x80 followed by appropriate param value

// Probably overkill and not needed for my simulation
0x11 Optional, Set Indoor Bike Training Params, Parameter Array
  Response: 0x80 followed by appropriate param value


Procedure:

First: Request control (opcode 0x00), Result Code should be "Success"
The response shall be indicated when the Reset Procedure is completed using the Response Code Op 
Code and the Request Op Code, along with the appropriate Result Code as defined in Section 4.16.2.22.

Procedure Complete:

Response Code Op: 0x80
Param Value:
  Request Op Code, value from client
  Result Code, 1 = Success, 2 = Not supported, 3 = Invalid Param, 4 = Op failed, 5 = Not permitted
  Respone Parameter

If an Op Code is written to the Fitness Machine Control Point
Server shall indicate the Fitness Machine Control Point with
Op Code, and the Result Code set to Success. 

If an Op Code is written to the Fitness Machine Control Point characteristic and the Client Characteristic 
Configuration descriptor of the Fitness Machine Control Point is not configured for indications, the Server 
shall return an error response with the Attribute Protocol error code set to Client Characteristic 
Configuration Descriptor Improperly Configured as defined in CSS Part B, Section 1.2 [3]. 


---

Fitness Machine Status, 4.17:
2ADA, https://github.com/oesmith/gatt-xml/blob/master/org.bluetooth.characteristic.fitness_machine_status.xml
Can be notified.

The Fitness Machine Status characteristic is used to send the status of the Server. Included in the 
characteristic value is a Fitness Machine Status field, and depending on the value of the Fitness Machine 
Status Field, a Parameter field may also be included. 

0x01 Reset
0x02 Fitness Machine Stopped
0x03 Fitness Machine Stopped Emergency
0x04 Fitness Machine Started
0x05 Target Speed Changed
0x06 Target Incline changed
0x07 Target Resistance changed
0x08 Target Power changed



*/

//extern ble_cps_calculation_helper_t cps_calculation_helper;



/** @defgroup BLE_FTMS_FEATURES Fitness Machine Service feature bits
 * @{ */
#define BLE_FTMS_FEATURE_AVERAGE_SPEED_SUPPORTED_BIT (0x01 << 0)   
#define BLE_FTMS_FEATURE_CADENCE_SUPPORTED_BIT (0x01 << 1)   
#define BLE_FTMS_FEATURE_TOTAL_DISTANCE_SUPPORTED_BIT (0x01 << 2)   
#define BLE_FTMS_FEATURE_INCLINATION_SUPPORTED_BIT (0x01 << 3)   
// ..
#define BLE_FTMS_FEATURE_RESISTANCE_SUPPORTED_BIT (0x01 << 7)   
#define BLE_FTMS_FEATURE_POWER_MEASUREMENT_SUPPORTED_BIT (0x01 << 14)   

#define BLE_FTMS_TARGET_SPEED_SUPPORTED_BIT (0x01 << 0)   
#define BLE_FTMS_TARGET_INCLINATION_SUPPORTED_BIT (0x01 << 1)   
#define BLE_FTMS_TARGET_RESISTANCE_SUPPORTED_BIT (0x01 << 2)   
#define BLE_FTMS_TARGET_POWER_SUPPORTED_BIT (0x01 << 3)   
#define BLE_FTMS_TARGETED_CADENCE_CONFIGURATION_SUPPORTED_BIT (0x01 << 16)

//...

#define BLE_FTMS_INDOOR_FLAGS_FIELD_AVERAGE_SPEED_PRESENT (1 << 1);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_INSTANTANEOUS_CADENCE_PRESENT (1 << 2);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_AVERAGE_CADENCE_PRESENT (1 << 3);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_TOTAL_DISTANCE_PRESENT (1 << 4);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_RESISTANCE_LEVEL_PRESENT (1 << 5);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_INSTANTANEOUS_POWER_PRESENT (1 << 6);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_AVERAGE_POWER_PRESENT (1 << 7);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_EXPENDED_ENERGY_PRESENT (1 << 8);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_HEARTRATE_PRESENT (1 << 9);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_METABOLIC_EQUIVALENT_PRESENT (1 << 0xa);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_ELAPSED_TIME_PRESENT (1 << 0xb);
#define BLE_FTMS_INDOOR_FLAGS_FIELD_REMAINING_TIME_PRESENT (1 << 0xc);





// Target limits
#define BLE_FTMS_INCLINE_MINIMUM -100 // -10%
#define BLE_FTMS_INCLINE_MAXIMUM 200 // +20%
#define BLE_FTMS_INCLINE_INCREMENT 10 // 1%

#define BLE_FTMS_RESISTANCE_MINIMUM 0 // 0
#define BLE_FTMS_RESISTANCE_MAXIMUM 70 // 200
#define BLE_FTMS_RESISTANCE_INCREMENT 1 // 10

#define BLE_FTMS_POWER_MINIMUM 0 // 0
#define BLE_FTMS_POWER_MAXIMUM 500 // 500
#define BLE_FTMS_POWER_INCREMENT 1 // 1
// 





//#define BLE_FTMS_FEATURE_MULTIPLE_SENSORS_BIT   (0x01 << 2)     /**< Multiple Sensor Locations Supported bit. */
/** @} */


/**@brief Fitness Machine Service event type. */
typedef enum
{
    BLE_FTMS_EVT_MACHINESTATUS_NOTIFICATION_ENABLED,                                  /**< Fitness Machine value notification enabled event. */
    BLE_FTMS_EVT_MACHINESTATUS_NOTIFICATION_DISABLED,
    BLE_FTMS_EVT_INDOORBIKEDATA_NOTIFICATION_ENABLED,
    BLE_FTMS_EVT_INDOORBIKEDATA_NOTIFICATION_DISABLED,
    BLE_FTMS_EVT_TRAININGSTATUS_NOTIFICATION_ENABLED,
    BLE_FTMS_EVT_TRAININGSTATUS_NOTIFICATION_DISABLED,
    BLE_FTMS_EVT_NOTIFICATION_ENABLED,
    BLE_FTMS_EVT_NOTIFICATION_DISABLED,                                  /**< Fitness Machine value notification disabled event. */
    BLE_FTMS_EVT_INDICATION_ENABLED,
    BLE_FTMS_EVT_INDICATION_DISABLED
} ble_ftms_evt_type_t;

/**@brief Fitness Machine Service event. */
typedef struct
{
    ble_ftms_evt_type_t evt_type;                                       /**< Type of event. */
} ble_ftms_evt_t;

// Forward declaration of the ble_ftms_t type.
typedef struct ble_ftms_s ble_ftms_t;

/**@brief Fitness Machine Service event handler type. */
typedef void (*ble_ftms_evt_handler_t) (ble_ftms_t * p_ftms, ble_ftms_evt_t * p_evt);

typedef enum {
    BLE_FTMS_TRAINING_STATUS_OTHER = 0x00,
    BLE_FTMS_TRAINING_STATUS_IDLE = 0x01,
    BLE_FTMS_TRAINING_STATUS_WARMING_UP = 0x02,
    BLE_FTMS_TRAINING_STATUS_LOW_INTENSITY_INTERVAL = 0x03,
    BLE_FTMS_TRAINING_STATUS_HIGH_INTENSITY_INTERVAL = 0x04,
    BLE_FTMS_TRAINING_STATUS_RECOVERY_INTERVAL = 0x05,
    BLE_FTMS_TRAINING_STATUS_ISOMETRIC = 0x06,
    BLE_FTMS_TRAINING_STATUS_HEARTRATE_CONTROL = 0x07,
    BLE_FTMS_TRAINING_STATUS_FITNESS_TEST = 0x08,
    BLE_FTMS_TRAINING_STATUS_SPEED_OUTSIDE_CONTROL_REGION_LOW = 0x09,
    BLE_FTMS_TRAINING_STATUS_SPEED_OUTSIDE_CONTROL_REGION_HIGH = 0x0A,
    BLE_FTMS_TRAINING_STATUS_COOL_DOWN = 0x0B,
    BLE_FTMS_TRAINING_STATUS_WATT_CONTROL = 0x0C,
    BLE_FTMS_TRAINING_STATUS_MANUAL_MODE = 0x0D,
    BLE_FTMS_TRAINING_STATUS_QUICKSTART = 0x0D,
    BLE_FTMS_TRAINING_STATUS_PRE_WORKOUT = 0x0E,
    BLE_FTMS_TRAINING_STATUS_POST_WORKOUT = 0x0F
} ble_ftms_training_status_t;

typedef struct {
    uint32_t fitness_machine_features;
    uint32_t target_setting_features;
} ble_ftms_fitness_machine_feature_t;


/**@brief Fitness Machine Service init structure. This contains all options and data
*         needed for initialization of the service. */
typedef struct
{
    ble_ftms_evt_handler_t        evt_handler;                           /**< Event handler to be called for handling events in the Fitness Machine Service. */
    security_req_t               ftms_cccd_wr_sec;                  /**< Security requirement for writing Fitness Machine measurement characteristic CCCD. */
    security_req_t               ftms_feature_rd_sec;                    /**< Security requirement for reading Fitness Machine feature characteristic. */
    security_req_t               ftms_location_rd_sec;                   /**< Security requirement for reading Fitness Machine location characteristic. */
    security_req_t               ftms_ctrlpt_cccd_wr_sec;                 /**< Security requirement for writing power control point characteristic CCCD. */
    security_req_t               ftms_ctrlpt_wr_sec;                      /**< Security requirement for writing power control point characteristic. */

    security_req_t              ftms_machine_status_rd_sec;
    security_req_t              ftms_indoor_bike_rd_sec;
    security_req_t              ftms_training_status_rd_sec;
    security_req_t              ftms_sup_power_range_rd_sec;
    security_req_t              ftms_sup_inclination_range_rd_sec;
    security_req_t              ftms_sup_resistance_range_rd_sec;


    uint16_t                            machine_status;

    ble_ftms_fitness_machine_feature_t                     fitness_machine_feature;                               /**< Initial value for features of sensor @ref BLE_FTMS_FEATURES. */   
    
    uint8_t                      ctrplt_supported_functions;            /**< Supported control point functionalities see @ref BLE_SRV_SC_CTRLPT_SUPP_FUNC. */
    ble_ftms_ctrlpt_evt_handler_t  ctrlpt_evt_handler;                    /**< Event handler */    
    
    ble_srv_error_handler_t      error_handler;                         /**< Function to be called in case of an error. */
    ble_sensor_location_t        *sensor_location;                      /**< Initial Sensor Location, if NULL, sensor_location characteristic is not added*/    
    
    volatile uint8_t                      *resistance_level;
    uint8_t                       resistance;
    uint16_t                      power;
    ble_ftms_training_status_t    training_status;    
} ble_ftms_init_t;

/**@brief Fitness Machine Service structure. This contains various status information for
 *        the service. */
struct ble_ftms_s
{
    ble_ftms_evt_handler_t       evt_handler;                           /**< Event handler to be called for handling events in the Fitness Machine Service. */
    uint16_t                     service_handle;                        /**< Handle of Fitness Machine Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     indoor_bike_handles;                          /**< Handles related to the Indoor Bike characteristic. */
    ble_gatts_char_handles_t     feature_handles;                       /**< Handles related to the Fitness Machine feature characteristic. */
    ble_gatts_char_handles_t     machine_status_handles;                    /**< Handles related to the Fitness Machine Status characteristic. */
    ble_gatts_char_handles_t     training_status_handles;                    /**< Handles related to the Fitness Machine Training Status characteristic. */

    ble_gatts_char_handles_t     sup_power_range_handles;               /**< Handles related to the Supported Power Range characteristc */
    ble_gatts_char_handles_t     sup_inclination_range_handles;         /**< Handles related to the Supported Inclination Range characteristc */
    ble_gatts_char_handles_t     sup_resistance_range_handles;               /**< Handles related to the Supported Resistance Range characteristc */
    
    uint8_t                         control_acquired;
    uint8_t                         trainer_started;
    uint16_t                      machine_status;
    ble_ftms_training_status_t   training_status;
    volatile uint8_t                      *resistance_level;
    uint16_t                        target_cadence;
    int16_t                         target_power;
    uint8_t                     target_resistance;
    uint8_t                      resistance; // Probably irrelevant
    int8_t                       incline;
    uint16_t                      power;
    ble_ftms_fitness_machine_feature_t fitness_machine_feature;                               /**< Bit mask of features available on sensor. */    
    uint16_t                      conn_handle;                           /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    
    ble_ftms_ctrlpt_t              ctrl_pt;                               /**< data for power control point */    
};

/**@brief Fitness Machine Service measurement structure. This contains a Fitness Machine
 *        Service measurement. */
typedef struct ble_ftms_meas_s
{
    bool        is_wheel_rev_data_present;                              /**< True if Wheel Revolution Data is present in the measurement. */
    bool        is_crank_rev_data_present;                              /**< True if Crank Revolution Data is present in the measurement. */
    int16_t     instantaneous_power;                                    /**< Instantaneous power */
    uint32_t    cumulative_wheel_revs;                                  /**< Cumulative Wheel Revolutions. */
    uint16_t    last_wheel_event_time;                                  /**< Last Wheel Event Time. */
    uint16_t    cumulative_crank_revs;                                  /**< Cumulative Crank Revolutions. */
    uint16_t    last_crank_event_time;                                  /**< Last Crank Event Time. */
} ble_ftms_meas_t;

typedef struct
{

} target_time_two_heart_rate_zones_t;

typedef struct
{

} target_time_three_heart_rate_zones_t;

typedef struct
{

} target_time_five_heart_rate_zones_t;

typedef enum
{
    SPINDOWN_STATUS_SPIN_DOWN_REQUESTED = 0x01,
    SPINDOWN_STATUS_SUCCESS = 0x02,
    SPINDOWN_STATUS_ERROR = 0x03,
    SPINDOWN_STATUS_STOP_PEDALING = 0x04
} spindown_status_t;

typedef struct
{
    ble_ftmspt_operator_t op_code;
    union
    {
        uint8_t control_information;
        uint16_t target_speed_kmh;
        int16_t target_incline_percent;
        uint8_t target_resistance;
        int16_t target_power_watts;
        uint8_t target_heartrate_bpm;
        uint16_t target_expended_energy_calories;
        uint16_t target_number_of_steps;
        uint16_t target_number_of_strides;
        uint32_t target_distance_meters; // TODO: Should be uint24_t!
        uint16_t target_training_time_seconds;
        target_time_two_heart_rate_zones_t target_time_two_heart_rate_zones;
        target_time_three_heart_rate_zones_t target_time_three_heart_rate_zones;
        target_time_five_heart_rate_zones_t target_time_five_heart_rate_zones;
        ble_ftms_indoor_bike_simulation_parameters_t *new_indoor_bike_simulation_parameters;
        uint16_t new_wheel_circumfence_mm;
        spindown_status_t *spindown_status_t;
        uint16_t target_cadence_rpm;
    } response;
} ble_ftms_machine_status_t;

typedef enum
{
    BLE_FTMS_MACHINE_STATUS_OP_CODE_RESET = 0x01,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_STOPPED_OR_PAUSED_BY_USER = 0x02,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_STOPPED_OR_PAUSED_BY_SAFETY_KEY = 0x03,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_STARTED_OR_RESUMED_BY_USER = 0x04,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_SPEED_CHANGED = 0x05,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_INCLINE_CHANGED = 0x06,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_RESISTANCE_LEVEL_CHANGED = 0x07,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_POWER_CHANGED = 0x08,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGET_HEARTRATE_CHANGED = 0x09,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGETED_EXPENDED_ENERGY_CHANGED = 0x0A,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGETED_NUMBER_OF_STEPS_CHANGED = 0x0B,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGETED_NUMBER_OF_STRIDES_CHANGED = 0x0C,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGETED_DISTANCE_CHANGED = 0x0D,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGETED_TRAINING_TIME_CHANGED = 0x0E,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGETED_TIME_TWO_HEARTRATE_ZONES_CHANGED = 0x0F,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGETED_TIME_THREE_HEARTRATE_ZONES_CHANGED = 0x10,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGETED_TIME_FIVE_HEARTRATE_ZONES_CHANGED = 0x11,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_INDOOR_BIKE_SIMULATION_PARAMETERS_CHANGED = 0x12,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_WHEEL_CIRCUMFENCE_CHANGED = 0x13,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_SPINDOWN_STATUS = 0x14,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_TARGETED_CADENCE_CHANGED = 0x15,
    BLE_FTMS_MACHINE_STATUS_OP_CODE_CONTROL_PERMISSION_LOST = 0xFF
} ble_ftms_machine_status_op_code_t;

/*
typedef struct {  
  ble_cscs_c_calculation_helper_t cscs_c;  
  double zPower;
} ble_ftms_calculation_helper_t;
*/
typedef struct {
    uint16_t instantaneous_speed;
    uint16_t average_speed;
    uint16_t instantaneous_cadence;
    uint16_t average_cadence;
    int16_t instantaneous_power;
    int16_t average_power;
    int16_t resistance_level;
} ble_ftms_indoor_bike_data_t;

static uint8_t ftms_machine_status_encode(ble_ftms_t      * p_ftms,
                                     ble_ftms_machine_status_t * p_cp_measurement,
                                     uint8_t        * p_encoded_buffer);

void ftms_op_reset(ble_ftms_t * p_ftms);
void ftms_op_acquire_control(ble_ftms_t * p_ftms);
void ftms_op_set_target_inclination(ble_ftms_t * p_ftms, int16_t target_inclination);
void ftms_op_set_target_power(ble_ftms_t * p_ftms, int16_t target_power);
void ftms_op_set_target_resistance(ble_ftms_t * p_ftms, uint8_t target_resistance);
void ftms_op_start_resume_training(ble_ftms_t * p_ftms);
void ftms_op_stop_pause_training(ble_ftms_t * p_ftms, uint8_t stop_pause_value);
void ftms_op_set_indoor_training_bike_parameters(ble_ftms_t * p_ftms,  ble_ftms_indoor_bike_simulation_parameters_t indoor_bike_simulation_parameters);
void ftms_op_set_targeted_cadence(ble_ftms_t * p_ftms, uint16_t target_cadence);
uint32_t ble_ftms_machine_status_send(ble_ftms_t * p_ftms, ble_ftms_machine_status_t * p_machine_status);
uint32_t ble_ftms_indoor_bike_data_send(ble_ftms_t * p_ftms, ble_ftms_indoor_bike_data_t * p_indoor_bike_data);
uint32_t ble_ftms_training_status_send(ble_ftms_t * p_ftms, ble_ftms_training_status_t * p_training_status);

/**@brief Function for initializing the Fitness Machine Service.
 *
 * @param[out]  p_ftms       Fitness Machine Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_ftms_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_ftms_init(ble_ftms_t * p_ftms, ble_ftms_init_t const * p_ftms_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Fitness Machine
 *          Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Fitness Machine Service structure.
 */
void ble_ftms_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending cycling power measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a Fitness Machine
 *          Service measurement. If notification has been enabled, the measurement data is encoded
 *          and sent to the client.
 *
 * @param[in]   p_ftms          Fitness Machine Service structure.
 * @param[in]   p_measurement  Pointer to new cycling power measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ftms_measurement_send(ble_ftms_t * p_ftms, ble_ftms_meas_t * p_measurement);
uint32_t ble_ftms_init(ble_ftms_t * p_ftms, ble_ftms_init_t const * p_ftms_init);

void ble_ftms_on_cscs_evt(ble_ftms_t * p_ftms, ble_cscs_c_evt_t * p_cscs_c_evt);
double ftms_calc_instantaneous_cadence();
double ftms_calc_average_cadence();
int8_t get_incline_from_ftms_incline(ble_ftms_t *p_ftms);
void set_ftms_incline_rough(ble_ftms_t *p_ftms, int8_t incline);
void set_ftms_incline(ble_ftms_t *p_ftms, int16_t target_incline);
void set_ftms_resistance(ble_ftms_t *p_ftms, uint8_t  target_resistance);


#ifdef __cplusplus
}
#endif

#endif // BLE_FTMS_H__

/** @} */
