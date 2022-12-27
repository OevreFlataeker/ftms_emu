
#ifndef BLE_FEC_H__
#define BLE_FEC_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#include "ble_cscs_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_fec instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _fec_max_clients Maximum number of FEC clients connected at a time.
 * @hideinitializer
 */


/*
#define BLE_NUS_DEF(_name, _nus_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_nus_max_clients),                  \
                             sizeof(ble_nus_client_context_t));   \
    static ble_nus_t _name =                                      \
    {                                                             \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
    };                                                            \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_NUS_BLE_OBSERVER_PRIO,               \
                         ble_nus_on_ble_evt,                      \
                         &_name)
*/


// Our BLE_NUS_DEF does not have a BLE_OBSERVER! We need to do this on our own!
#define BLE_FEC_DEF(_name, _fec_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_fec_max_clients),                  \
                             sizeof(ble_fec_client_context_t));   \
    static ble_fec_t _name =                                      \
    {                                                             \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
    };                                                            
    


#define BLE_UUID_TACX_SERVICE 0xFEC1 /**< The UUID of the Nordic UART Service. */

#define BLE_UUID_TACX_RX_FEC2_CHARACTERISTIC 0xFEC2               /**< The UUID of the RX Characteristic. */
#define BLE_UUID_TACX_TX_FEC3_CHARACTERISTIC 0xFEC3               /**< The UUID of the TX Characteristic. */

#define BLE_FEC_MAX_RX_CHAR_LEN        BLE_FEC_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_FEC_MAX_TX_CHAR_LEN        BLE_FEC_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */

//#define NUS_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */
#define TACX_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0xC1, 0xFE, 0x40, 0x6E}} /**< Used vendor specific UUID. */


#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_FEC_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_NUS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif


/**@brief   Nordic UART Service event types. */
typedef enum
{
    BLE_FEC_EVT_RX_DATA,      /**< Data received. */
    BLE_FEC_EVT_TX_RDY,       /**< Service is ready to accept new data to be transmitted. */
    BLE_FEC_EVT_COMM_STARTED, /**< Notification has been enabled. */
    BLE_FEC_EVT_COMM_STOPPED, /**< Notification has been disabled. */
} ble_fec_evt_type_t;


/* Forward declaration of the ble_nus_t type. */
typedef struct ble_fec_s ble_fec_t;


/**@brief   Nordic UART Service @ref BLE_NUS_EVT_RX_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_NUS_EVT_RX_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data; /**< A pointer to the buffer with received data. */
    uint16_t        length; /**< Length of received data. */
} ble_fec_evt_rx_data_t;


/**@brief Nordic UART Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_fec_client_context_t;


/**@brief   Nordic UART Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
    ble_fec_evt_type_t         type;        /**< Event type. */
    ble_fec_t                * p_fec;       /**< A pointer to the instance. */
    uint16_t                   conn_handle; /**< Connection handle. */
    ble_fec_client_context_t * p_link_ctx;  /**< A pointer to the link context. */
    union
    {
        ble_fec_evt_rx_data_t rx_data; /**< @ref BLE_NUS_EVT_RX_DATA event data. */
    } params;
} ble_fec_evt_t;


/**@brief Nordic UART Service event handler type. */
typedef void (* ble_fec_data_handler_t) (ble_fec_t *p_fec, ble_fec_evt_t * p_evt);

typedef enum
{
    BLE_FEC_PAGE_10,
    BLE_FEC_PAGE_19,
    BLE_FEC_PAGE_50,
    BLE_FEC_PAGE_55
} ble_fec_page_type_t;

typedef enum
{
    BLE_FEC_PAGE_GENERAL_FE_DATA = 0x10,
    BLE_FEC_PAGE_GENERAL_SETTINGS = 0x11,
    BLE_FEC_PAGE_GENERAL_FE_METABOLIC_DATA = 0x12,
    BLE_FEC_PAGE_SPECIFIC_TRAINER_STATIONARY_BIKEDATA = 0x19,
    BLE_FEC_CONTROL_SET_BASIC_RESISTANCE = 0x30,
    BLE_FEC_CONTROL_SET_TARGET_POWER = 0x31,
    BLE_FEC_CONTROL_SET_WIND_RESISTANCE = 0x32,
    BLE_FEC_CONTROL_SET_TRACK_RESISTANCE = 0x33,
    BLE_FEC_PAGE_SET_USER_CONFIGURATION = 0x37,
    BLE_FEC_REQUEST_DATA_PAGE = 0x46,
    BLE_FEC_PAGE_COMMAND_STATUS = 0x47,
    BLE_FEC_PAGE_COMMON_MANUFACTURER_IDENT = 0x50,
    BLE_FEC_PAGE_COMMON_PRODUCT_INFORMATION = 0x51,    
    BLE_FEC_PAGE_COMMON_FE_CAPABILITIES = 0x36,
    BLE_FEC_PAGE_OPEN_SLOT = 0xff,
    // Tacx Custom pages (from Tacx Utility App)
    // tacx.unified.communication.datamessages.fec package
    // "Magnum" -> out of scope: https://bikerumor.com/2017/12/15/you-can-ride-your-bike-on-the-tacx-magnumb-smart-treadmill-trainer/

    BLE_FEC_TACX_CUSTOM_NEO_ERROR = 0xf0, // tacx.unified.communication.datamessages.fec.specific.neo.NeoError
    BLE_FEC_TACX_CUSTOM_NEO_GEAR1 = 0xf5, // tacx.unified.communication.datamessages.fec.specific.neo.NeoGear1
    BLE_FEC_TACX_CUSTOM_NEO_GEAR2 = 0xf6, // tacx.unified.communication.datamessages.fec.specific.neo.NeoGear2

    // Collision
    // BLE_FEC_TACX_CUSTOM_MAGNUM_ERROR = 0xf0,
    
    // BLE_FEC_TACX_CUSTOM_MAGNUM_STATUS1 = 0xfa,
    // BLE_FEC_TACX_CUSTOM_MAGNUM_VERSION = 0xfb,
    // BLE_FEC_TACX_CUSTOM_NEOBIKE = 0xfb,
    // BLE_FEC_TACX_CUSTOM_NEOBIKE_VERSION = 0xfa,
    // BLE_FEC_TACX_CUSTOM_MAGNUM_STATUS2 = 0xfd,
    BLE_FEX_TACX_CUSTOM_DATAPAGE_TACXSETTING = 0xf9, // tacx.unified.communication.datamessages.fec.specific.TacxSetting
    BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION1 = 0xfa, // Neoversion tacx.unified.communication.datamessages.fec.specific.neo.NeoVersion1
    BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION2 = 0xfb, // tacx.unified.communication.datamessages.fec.specific.neo.NeoVersion2
    BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOTRAINING = 0xfc, // tacx.unified.communication.datamessages.fec.specific.neo.NeoTraining
} ble_fec_page_names_t;

typedef enum
{
    TACX_ROADFEEL_NONE = 0x00,
    TACX_ROADFEEL_CONCRETE_PLATES = 0x01,
    TACX_ROADFEEL_CATTLE_GRID = 0x02,
    TACX_ROADFEEL_HARD_COBBLESTONE = 0x03,
    TACX_ROADFEEL_SOFT_COBBLESTONE = 0x04,
    TACX_ROADFEEL_BRICK_ROAD = 0x05,
    TACX_ROADFEEL_SOFT_ROAD = 0x06,
    TACX_ROADFEEL_GRAVEL = 0x07,
    TACX_ROADFEEL_ICE = 0x08,
    TACX_ROADFEEL_WOODEN_BOARDS = 0x09,
} tacx_roadfeel_surfaces_t;

typedef enum
{
    BLE_FEC_REQUEST_TYPE_BROADCAST = 0x4e,
    BLE_FEC_REQUEST_TYPE_ACKNOWLEDGED = 0x4f,
} ble_fec_request_type_t;

typedef enum
{
    BLE_FEC_COMMAND_STATUS_OK = 0x00,
    BLE_FEC_COMMAND_STATUS_FAIL = 0x01,
    BLE_FEC_COMMAND_STATUS_NOT_SUPPORTED = 0x02,
    BLE_FEC_COMMAND_STATUS_REJECTED = 0x03,
    BLE_FEC_COMMAND_STATUS_PENDING = 0x04,
    BLE_FEC_COMMAND_STATUS_UNINITIALIZED = 0xff,
} ble_fec_command_status_t;
typedef struct
{
        ble_fec_page_type_t         page;
        ble_fec_request_type_t      type;
        uint8_t                     payload[32];
        uint8_t                     payload_length;
}
ble_fec_page_evt_t;

typedef void (* ble_fec_page_handler_t) (ble_fec_t *p_fec, ble_fec_page_evt_t * p_fec_page_evt);
               
/**@brief   Nordic UART Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_nus_init
 *          function.
 */
typedef struct
{
    ble_fec_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_fec_init_t;


/**@brief   Nordic UART Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_fec_s
{
    uint8_t                         uuid_type;          /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t                        service_handle;     /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        tx_fec3_handles;         /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        rx_fec2_handles;         /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
    blcm_link_ctx_storage_t * const p_link_ctx_storage; /**< Pointer to link context storage with handles of all current connections and its context. */
    ble_fec_data_handler_t          data_handler;       /**< Event handler to be called for handling received data. */
    uint16_t                      conn_handle;  
};


/**@brief   Function for initializing the Nordic UART Service.
 *
 * @param[out] p_nus      Nordic UART Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_nus_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_nus or p_nus_init is NULL.
 */
uint32_t ble_fec_init(ble_fec_t * p_fec, ble_fec_init_t const * p_fec_init);


/**@brief   Function for handling the Nordic UART Service's BLE events.
 *
 * @details The Nordic UART Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Nordic UART Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     Nordic UART Service structure.
 */
void ble_fec_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for sending a data to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in]     p_nus       Pointer to the Nordic UART Service structure.
 * @param[in]     p_data      String to be sent.
 * @param[in,out] p_length    Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_fec_data_send(ble_fec_t * p_fec,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle);

void ble_fec_on_cscs_evt(ble_fec_t * p_fec, ble_cscs_c_evt_t * p_cscs_c_evt);
void fec_handle_command(ble_fec_t *p_fec, ble_fec_evt_t *p_evt);
void fec_callback_handler(ble_fec_t *p_fec, ble_fec_page_evt_t * p_fec_page_evt);
uint8_t ok_or_init();
#ifdef __cplusplus
}
#endif

#endif // BLE_FEC_H__

/** @} */
