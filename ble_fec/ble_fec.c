/**
 * Copyright (c) 2012 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_FEC)
#include "ble.h"
#include "ble_fec.h"
#include "ble_srv_common.h"
#include "helper.h"
#include "calculations.h"

#define NRF_LOG_MODULE_NAME ble_fec
#if BLE_FEC_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_FEC_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_FEC_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_FEC_CONFIG_DEBUG_COLOR
#else // BLE_FEC_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_FEC_CONFIG_LOG_ENABLED

#include "nrf_log.h"

#define LOGHERE 0

NRF_LOG_MODULE_REGISTER();

static uint8_t command_counter = 0xff;

// Keep state of FE-C custom Tacx state

tacx_roadfeel_surfaces_t stored_roadfeel_surface = TACX_ROADFEEL_NONE;
bool still_init=true;
uint8_t stored_isokinetic_active = 1;
uint8_t stored_roadfeel_intensity = 100; // 100% intensity
uint8_t stored_training_setpoint = 84; // Values between 84 and 168

/************************************************************************
 * Notes
 * tacx.unified.communication.datamessages.TacxAntConstants in Tacx Utility App v2.20
 * Interesting constants, maybe identifier for ANT subpages?
 * Not disclosed here: tacx.unified.communication.datamessages.AntPlusConstants
 * Individual pages are listed here: tacx.unified.communication.datamessages.fec
 * Pages are shared between models
 * 0xfc - Neo training
 * public class NeoTraining {
    static final int MAX_TRAINING_SETPOINT = 168;
    static final int MIN_TRAINING_SETPOINT = 84;
    @U8BIT(4)
    public int future_use1;
    @U8BIT(7)
    public int future_use2;
    @Page(AntPlusConstants.FEC.PAGE_MANUFACTURER_SPECIFIC_252)
    int page;
    @U8BIT(1)
    public int power_correction;
    @U8BIT(6)
    public int roadsurface_intensity;
    @U8BIT(5)
    public int roadsurface_type;
    @U8BIT(3)
    public int training_setpoint;
    @U8BIT(2)
    public int training_type;

    0xfa- Neo version
    tacx.unified.communication.datamessages.fec.specific.neo.NeoVersion1
    public class NeoVersion1 {
    @U8BIT(7)
    public int bootloaderAccessCode;
    @U8BIT(6)
    public int bootloaderResult;
    @U8BIT(3)
    public int communicationBootloaderBuild;
    @U8BIT(1)
    public int communicationBootloaderMayor;
    @U8BIT(2)
    public int communicationBootloaderMinor;
    @Page(250)
    int page;
    }

    0xfb - Neo Version 2
    ublic class NeoVersion2 {
    @U8BIT(3)
    public int communicationBuild;
    @U8BIT(1)
    public int communicationMayor;
    @U8BIT(2)
    public int communicationMinor;
    @U8BIT(6)
    public int motorControlBuild;
    @U8BIT(4)
    public int motorControlMayor;
    @U8BIT(5)
    public int motorControlMinor;
    @Page(AntPlusConstants.FEC.PAGE_MANUFACTURER_SPECIFIC_251)
    int page;
    }

    0xf9 Tacx Setting
    ublic class TacxSetting {
    private static final int INVALID_VALUE = 4095;
    private static final double WEIGHT_STEP = 10.0d;
    @LSBUXBIT(bitLength = 1, startBit = 0, value = 7)
    private int antDisabled;
    @LSBUXBIT(bitLength = 12, startBit = 4, value = 5)
    private int bikeWeight;
    @Page(AntPlusConstants.FEC.PAGE_MANUFACTURER_SPECIFIC_249)
    int page;
    @LSBUXBIT(bitLength = 12, startBit = 0, value = 4)
    private int userWeight;

    public boolean getAntDisabled() {
        return this.antDisabled == 1;
    }

    public void setAntDisabled(boolean z) {
        this.antDisabled = z ? 1 : 0;
    }

    LSBUXBIT === 32 bit?
    (from houtbecke.rs.antbytes.AntBytesImpl.java)
    if (annotationType == LSBUXBIT.class) {
                            LSBUXBIT lsbuxbit = (LSBUXBIT) annotation2;
                            BitBytes.outputLSB(bArr, lsbuxbit.value(), lsbuxbit.startBit(), getLongFromField(field, t), lsbuxbit.bitLength());
                        
    public static void output(byte[] bArr, int i, int i2, long j, int i3) {
        output(bArr, (i * 8) + i2, j, i3);
    }

    value is the offset into the buf

    Example for 0xf9:

    
0xf9 Tacx Setting
    public class TacxSetting {
    private static final int INVALID_VALUE = 4095;
    private static final double WEIGHT_STEP = 10.0d;
    @LSBUXBIT(bitLength = 1, startBit = 0, value = 7)
    private int antDisabled;
    @LSBUXBIT(bitLength = 12, startBit = 4, value = 5)
    private int bikeWeight;
    @Page(AntPlusConstants.FEC.PAGE_MANUFACTURER_SPECIFIC_249)
    int page;
    @LSBUXBIT(bitLength = 12, startBit = 0, value = 4)
    private int userWeight;

buf:0x00000000 0x00000000 0x00000000 0x00000000 0x00000000 0x00000000 0x00000000 0x00000000

    UserWEight:
                                     ^^^^^^^^^^^^^^^^^
    bikeweight:                                       ^^^^  ^^^^^^^^^                     ^




 ************************************************************************/
// The 6E40-FEC2-B5A3-F393-E0A9-E50E24DCCA9E is the RX part of the service this characteristic will notify is the trainer sends out an packet(Notification must be turned on in the CharacteristicConfigurationBits). 
// Characteristic 6E40-FEC3-B5A3-F393-E0A9-E50E24DCCA9E is the TX part of the service, message that need to be send to the trainer can be written to this characteristic. The Ant+ FE-C wound exceed the 
// 20 byte maximum , so there is no need to split message up in multiple sections. 

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_fec_t * p_fec, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_fec_evt_t              evt;
    ble_gatts_value_t          gatts_val;
    uint8_t                    cccd_value[2];
    ble_fec_client_context_t * p_client = NULL;


    p_fec->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;   
    err_code = blcm_link_ctx_get(p_fec->p_link_ctx_storage,
                                 p_ble_evt->evt.gap_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gap_evt.conn_handle);
    }

    /* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_fec->rx_fec2_handles.cccd_handle,
                                      &gatts_val);

    
    if ((err_code == NRF_SUCCESS)     &&
        (p_fec->data_handler != NULL) &&
        ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        if (p_client != NULL)
        {
            p_client->is_notification_enabled = true;
        }

        memset(&evt, 0, sizeof(ble_fec_evt_t));
        evt.type        = BLE_FEC_EVT_COMM_STARTED;
        evt.p_fec       = p_fec;
        evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_fec->data_handler(p_fec, &evt);
    }
}

uint8_t ok_or_init()
{
    if (still_init)
    {
        still_init = !still_init;
        return BLE_FEC_COMMAND_STATUS_UNINITIALIZED;
    }
    else
    {
        return BLE_FEC_COMMAND_STATUS_OK;
    }
}
void fec_handle_command(ble_fec_t *p_fec, ble_fec_evt_t *p_evt)
{
    bool dispatch = true;
    // NRF_LOG_INFO("FE-C: Received data from app");
    
    ble_fec_page_evt_t ble_fec_page_evt;

    // A4 09 4F 05 46 FF FF FF FF 80 51 01 71  
    // Strip first 4 bytes
    const uint8_t *idx = p_evt->params.rx_data.p_data;
    uint8_t length = 0;
    uint8_t lsb = 0;
    uint8_t msb = 0;

    ble_fec_page_evt.type = BLE_FEC_REQUEST_TYPE_BROADCAST; //*(idx+2);    
    ble_fec_page_evt.page = *(idx+4);

    const uint8_t *payload = idx+5;

    switch (ble_fec_page_evt.page)
    {
        case BLE_FEC_CONTROL_SET_BASIC_RESISTANCE:
            {
            uint8_t basic_resistance = *(payload + 6);
            char resistance_percent_s[8];
            sprintf(resistance_percent_s, "%.01f", (float)basic_resistance/2.0);
            NRF_LOG_INFO("Command: SET_BASIC_RESISTANCE: %s%% (not yet implemented)", resistance_percent_s);
            //NRF_LOG_HEXDUMP_INFO(payload, 8);

            ble_fec_page_evt.page = BLE_FEC_PAGE_COMMAND_STATUS;
            ble_fec_page_evt.type = BLE_FEC_REQUEST_TYPE_ACKNOWLEDGED;
            length = 0;
            ble_fec_page_evt.payload[length++] = BLE_FEC_CONTROL_SET_BASIC_RESISTANCE; // Last command
            ble_fec_page_evt.payload[length++] = command_counter++;   // Sequence number
            ble_fec_page_evt.payload[length++] = ok_or_init(); // Result
            ble_fec_page_evt.payload[length++] = 0xff;
            ble_fec_page_evt.payload[length++] = 0xff;
            ble_fec_page_evt.payload[length++] = 0xff;
            ble_fec_page_evt.payload[length++] = basic_resistance;
            ble_fec_page_evt.payload_length = length;
            fec_callback_handler(p_fec, &ble_fec_page_evt);
            dispatch = false;
            break;
            }
        case BLE_FEC_CONTROL_SET_TARGET_POWER:
            {
            uint8_t lsb = *(payload+5);
            uint8_t msb = *(payload+6);            
            uint16_t target_power = (msb << 8) + lsb;
            target_power = (uint16_t) (0.25 * (double) target_power);
                
            if (getFECMode() != FEC_TARGET_POWER)
            {
                setFECMode(FEC_TARGET_POWER);
                oled_data.mode = ERG;
                NRF_LOG_INFO("Switching to mode FEC_TARGET_MODE");
            }
            set_target_power(target_power, getAverageCadence());
        
            NRF_LOG_INFO("Command: SET_TARGET_POWER: %u", target_power);            

            // Send Command Status (0x47/71) as response
            
            ble_fec_page_evt.page = BLE_FEC_PAGE_COMMAND_STATUS;
            ble_fec_page_evt.type = BLE_FEC_REQUEST_TYPE_ACKNOWLEDGED;
            length = 0;
            ble_fec_page_evt.payload[length++] = BLE_FEC_CONTROL_SET_TARGET_POWER; // Last command
            ble_fec_page_evt.payload[length++] = command_counter++;   // Sequence number
            ble_fec_page_evt.payload[length++] = ok_or_init(); // Result
            
            ble_fec_page_evt.payload[length++] = 0xff;
            ble_fec_page_evt.payload[length++] = 0xff;
            ble_fec_page_evt.payload[length++] = lsb;
            ble_fec_page_evt.payload[length++] = msb;
            ble_fec_page_evt.payload_length = length;
            fec_callback_handler(p_fec, &ble_fec_page_evt);
            dispatch = false; // Added because missing here and present in all other cases
            break;
            }
        case BLE_FEC_CONTROL_SET_WIND_RESISTANCE:
            {
            NRF_LOG_INFO("Command: BLE_FEC_CONTROL_SET_WIND_RESISTANCE");
            // NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

            uint8_t wrc = *(payload+4);

            if (wrc == 0xff)
            {
                 // Controllable trainers shall [MD_FEC_001] assume the default wind resistance coefficient of 0.51 kg/m if the controller 
                 // populates the wind resistance coefficient field with an invalid value (0xFF). 
                 wrc = 51;
            }

            char wrc_s[16];
            sprintf(wrc_s, "%f", ((double) wrc) * 0.01);

            uint8_t windspeed_raw = *(payload+5);
            int8_t windspeed = 0;

            if (windspeed_raw == 0xff)
            {
                // Controllable fitness equipment shall [MD_FEC_001] assume the default wind speed of 0 km/h 
                // if an invalid value (0xFF) is transmitted.
                windspeed = 0;
            } 
            else
            {
                // Simulated Wind Speed (km/h) = Raw Wind Speed Value  127 km/h 
                windspeed = windspeed_raw - 127;
            }

            uint8_t drafting_factor = *(payload+6);

            if (drafting_factor == 0xff)
            {
                 drafting_factor = 100;
            }

            char drafting_factor_s[16];
            sprintf(drafting_factor_s, "%.2f", ((double) drafting_factor) * 0.01);            

            //NRF_LOG_INFO("Cntr: %02x, wrc (Raw) = %u, wrc = %s kg/m, Windspeed (Raw) = %u km/h, Windspeed = %d km/h, Drafting factor (Raw): %u, Drafting factor: %s", command_counter, wrc, wrc_s, windspeed_raw, windspeed, drafting_factor, drafting_factor_s);
            NRF_LOG_INFO("Cntr: %02x, wrc (Raw) = %u, wrc = %s kg/m, Windspeed (Raw) = %u km/h, Windspeed = %d km/h, Drafting factor: %s", command_counter, wrc, wrc_s, windspeed_raw, windspeed, /* drafting_factor,*/ drafting_factor_s);
            
            if (getFECMode() != FEC_SIMULATION)
            {
                setFECMode(FEC_SIMULATION);
                oled_data.mode = SIM;
                NRF_LOG_INFO("Switching to mode FEC_SIMULATION");
            }
            
            setWindResistanceCoefficient(wrc);
            setWindSpeed(windspeed);

            setDraftingFactor(drafting_factor);

            oled_data.wind_change_counter = 4;

            // set_ftms_simulation_parameter()

            // Send Command Status (0x47/71) as response
            ble_fec_page_evt.page = BLE_FEC_PAGE_COMMAND_STATUS;
            ble_fec_page_evt.type = BLE_FEC_REQUEST_TYPE_ACKNOWLEDGED;
            length = 0;
            ble_fec_page_evt.payload[length++] = BLE_FEC_CONTROL_SET_WIND_RESISTANCE; // Last command
            ble_fec_page_evt.payload[length++] = command_counter++;   // Sequence number
            ble_fec_page_evt.payload[length++] = ok_or_init(); // Result
            
            ble_fec_page_evt.payload[length++] = 0xff;
            ble_fec_page_evt.payload[length++] = wrc;
            ble_fec_page_evt.payload[length++] = windspeed_raw;
            ble_fec_page_evt.payload[length++] = drafting_factor;
            ble_fec_page_evt.payload_length = length;
            fec_callback_handler(p_fec, &ble_fec_page_evt);     

            dispatch = false;
            break;
            }

        case BLE_FEX_TACX_CUSTOM_DATAPAGE_TACXSETTING:
        {
          NRF_LOG_INFO("Command: BLE_FEX_TACX_CUSTOM_DATAPAGE_TACXSETTING");
          NRF_LOG_INFO("Not taking into account, using hard coded values");
         /*
                                       ^   ^ 
32 kg, 15k = 0x96
[00:13:17.538,452] <error> ble_fec: Unknown page requested: 0xF9
[00:13:17.544,433] <info> ble_fec:  A4 09 4F 05 F9 00 00 00|.	O.....
[00:13:17.544,433] <info> ble_fec:  40 61 09 01 37         |@a	.7   
                                       ^   ^
100 lg, 11kg = 100kg = 1000 = 0x3e8, 11kg = 110 = 0x4
[00:15:21.826,110] <error> ble_fec: Unknown page requested: 0xF9
[00:15:21.832,092] <info> ble_fec:  A4 09 4F 05 F9 00 00 00|.	O.....
[00:15:21.832,092] <info> ble_fec:  E8 E3 06 01 12         |.....   

*/
          NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
                        
          break;
        }
        case BLE_FEC_CONTROL_SET_TRACK_RESISTANCE:
            {
            NRF_LOG_INFO("Command: BLE_FEC_CONTROL_SET_TRACK_RESISTANCE");
            //NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

            uint8_t lsb = *(payload+4);
            uint8_t msb = *(payload+5);
            uint16_t grade = 0xffff;
            
            if (lsb == 0xff && msb == 0xff)
            {
                // Controllable fitness equipment shall [MD_FEC_001] assume the default grade of 0% (flat track) if the display sets the grade field to 
                // invalid (0xFFFF).             
                lsb = 0x20;
                msb = 0x4e;     // (0x4e << 8) + 0x20 = 20000 == 0%           
            }

            grade = ((msb << 8) + lsb);  // -200% - +200%              
            
            uint8_t crr = 0xff;
            crr = *(payload+6);

            char grade_s[16];
            float grade_f = ((float) grade/100.0) - 200.0;
            sprintf(grade_s, "%.2f", grade_f);
            oled_data.grade = grade_f;

            char crr_s[16];
            
            // Controllable trainers shall [MD_FEC_001] assume a default coefficient of rolling resistance of 0.004 (bicycle tires on 
            // asphalt road) for calculating rolling resistance if the controller sets this field to invalid (0xFF). 
            if (crr == 0xff)
            {
                crr = 80; // 80 * 0.00005 => 0.004
            }

            sprintf(crr_s, "%.4f", ((float) crr) * 0.00005);
            
            NRF_LOG_INFO("Cntr: %02x, Grade (Raw) = %d, Grade (Real World) = %s%%, crr (Raw) = 0x%02x, crr (Real world) = %s", command_counter, grade, grade_s, crr, crr_s);

            if (getFECMode() != FEC_SIMULATION)
            {
                setFECMode(FEC_SIMULATION);
                oled_data.mode = SIM;
                NRF_LOG_INFO("Switching to mode FEC_SIMULATION");
            }

            // set_ftms_simulation_parameter()
            setCoefficientRollingResistance(crr);
            setGrade(grade);

            oled_data.gravitational_change_counter = 4;
            oled_data.rolling_change_counter = 4;
            // Send Command Status (0x47/71) as response
            
            ble_fec_page_evt.page = BLE_FEC_PAGE_COMMAND_STATUS;
            ble_fec_page_evt.type = BLE_FEC_REQUEST_TYPE_ACKNOWLEDGED;
            length = 0;
            ble_fec_page_evt.payload[length++] = BLE_FEC_CONTROL_SET_TRACK_RESISTANCE; // Last command
            ble_fec_page_evt.payload[length++] = command_counter++;   // Sequence number
            ble_fec_page_evt.payload[length++] = ok_or_init(); // Result
            
            ble_fec_page_evt.payload[length++] = 0xff;
            ble_fec_page_evt.payload[length++] = lsb;
            ble_fec_page_evt.payload[length++] = msb;
            ble_fec_page_evt.payload[length++] = crr;
            ble_fec_page_evt.payload_length = length;
            fec_callback_handler(p_fec, &ble_fec_page_evt);             
            
            dispatch = false;
            break;
            }
        case BLE_FEC_PAGE_COMMAND_STATUS:
            NRF_LOG_INFO("Command: BLE_FEC_PAGE_COMMAND_STATUS as per request (not in response to command) - Not yet implemented");
            dispatch = false;
            break;
        case BLE_FEC_PAGE_SET_USER_CONFIGURATION:
            NRF_LOG_INFO("Command: BLE_FEC_PAGE_SET_USER_CONFIGURATION");
            NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
            /*
            [00:00:00.611,938] <info> ble_nus: Command: BLE_FEC_PAGE_SET_USER_CONFIGURATION
            [00:00:00.612,426] <info> ble_nus:  A4 09 4F 05 37 4C 1D FF|.   O.7L..
            [00:00:00.612,426] <info> ble_nus:  08 0A 43 00 3F         |..C.?
            [00:00:00.613,159] <info> ble_nus: User Config: Weight: 42836, Diameter_Offset: 10, Bike weight: 15360, Wheel Diameter: 0, Gear Ratio: 63
            [00:00:00.613,952] <info> calculations: Set new user weight: 84
            [00:00:00.614,318] <info> calculations: Set new bike weight: 0
            4c weight lsb   byte 1
            1d weight msb   byte 2
            ff reserverd    byte 3
            08 offset + bike weight lsb   byte 4
            0a bike weight msb
            43 wheel diameter
            00 gear ratio
            3f crc


            [00:00:00.522,277] <info> ble_nus:  A4 09 4F 05 37 FF FF FF|.   O.7...
[00:00:00.522,277] <info> ble_nus:  F8 FF 43 00 6B         |..C.k



            */
            lsb = *(payload);
            msb = *(payload+1);
            
            uint16_t user_weight = 0xffff;
            // Gravitational resistance is calculated using the grade of the simulated track and the combined mass of the user plus fitness 
            // equipment. Controllable trainers shall [MD_FEC_001] assume an equipment (bicycle) mass of 10kg and a user mass of 
            // 75kg if invalid values were set during configuration. 

            // The Tacx utility app somethimes sends "legit" 0 values - which does not make sense at all, but are legit according to FE-C
            if (!(lsb == 0xff && msb == 0xff))
            {
                user_weight = (uint16_t) (((msb << 8) + lsb) * 0.01);                
            }
            else // Invalid value, assume 75kg
            {
                user_weight = 75;
            }
            

            uint8_t diameter_offset = *(payload+3) & 0xf; // Offset in mm 0-10mm

            lsb = *(payload+3) >> 4;  
            msb = *(payload+4);

            uint16_t bicycle_weight = 0xfff;
            
            if (!(lsb == 0xf && msb == 0xff))
            {
                bicycle_weight = (uint16_t) (((msb << 4) + lsb) * 0.05); //  << 4 because it' 1.5 bytes! The MSB is only shifted by 4 bytes
            }
            else
            {
                bicycle_weight = 10;
            }
            

            uint8_t bicycle_diameter = *(payload+5);
            if (bicycle_diameter == 0xff) // Invalid value
            {
                bicycle_diameter = 70;
            }
            uint8_t gear_ratio = *(payload+6); // According to spec 0 is invalid but no default value is given.

            NRF_LOG_INFO("Cntr: %02x, User Config: Weight: %u kg, Diameter_Offset: %u mm, Bike weight: %u kg, Wheel Diameter: %u cm, Gear Ratio: %u", command_counter, user_weight, diameter_offset, bicycle_weight, bicycle_diameter, gear_ratio);
            
            if (user_weight != 0xffff)
            {
                setUserWeight(user_weight);
            }

            if (bicycle_weight != 0xfff)
            {
                setBikeWeight(bicycle_weight);
            }
            // No confirmation needed?
            // dispatch = false;
            // 20220402: Yes! Needed! 8.10.2 Data Page 55 (0x37) – User Configuration - page 61 in FE-C documentation
            ble_fec_page_evt.page = BLE_FEC_PAGE_SET_USER_CONFIGURATION;
            ble_fec_page_evt.type = BLE_FEC_REQUEST_TYPE_ACKNOWLEDGED;
            length = 0;            
            ble_fec_page_evt.payload[length++] = *(payload);   
            ble_fec_page_evt.payload[length++] = *(payload+1);   
            ble_fec_page_evt.payload[length++] = *(payload+2);   
            ble_fec_page_evt.payload[length++] = *(payload+3);   
            ble_fec_page_evt.payload[length++] = *(payload+4);   
            ble_fec_page_evt.payload[length++] = *(payload+5);   
            ble_fec_page_evt.payload[length++] = *(payload+6);   
            ble_fec_page_evt.payload_length = length;
            fec_callback_handler(p_fec, &ble_fec_page_evt);   
            dispatch=false;

            break;
        case BLE_FEC_REQUEST_DATA_PAGE:
            // NRF_LOG_INFO("Received REQUEST_DATA_PAGE");
            // parse params?
            // 0x51 = Product Information
            // 0x36 = FE Capabilities
            // 0x50 = Manufacturer Identification
            // Just guesswork
            {
                uint16_t slave_serial_number = *(payload)<<8+*(payload+1);
                uint8_t desc_byte1 = *(payload+2);
                uint8_t desc_byte2 = *(payload+3);
                uint8_t requested_response = *(payload+4);
                uint8_t requested_page = *(payload+5); //*(idx+10);
                uint8_t command_type = *(payload+6);

                switch (command_type)
                {
                  case 0x01:
                    // Request for data page
                    break;
                  case 0x02:
                    // Request ANT-FS Session
                    break;
                  case 0x03: 
                    // Request data page from slave
                    break;
                  case 0x04:
                    // Request data page set
                    break;
                  default:
                    break;
                }

                

                switch (requested_page)
                {
                    case BLE_FEC_PAGE_COMMON_FE_CAPABILITIES:
                        NRF_LOG_INFO("Requested page: BLE_FEC_PAGE_COMMON_FE_CAPABILITIES");
                        dispatch = true;
                        break;
                    case BLE_FEC_PAGE_COMMON_MANUFACTURER_IDENT:
                        NRF_LOG_INFO("Requested page: BLE_FEC_PAGE_COMMON_MANUFACTURER_IDENT");                    
                        dispatch = true;
                        break;
                    break;
                    case BLE_FEC_PAGE_COMMON_PRODUCT_INFORMATION:
                        NRF_LOG_INFO("Requested page: BLE_FEC_PAGE_COMMON_PRODUCT_INFORMATION");                    
                        dispatch = true;
                        break;
                    break;
                    case BLE_FEX_TACX_CUSTOM_DATAPAGE_TACXSETTING:
                        dispatch = true;
                        NRF_LOG_INFO("Serving request for BLE_FEX_TACX_CUSTOM_DATAPAGE_TACXSETTING (0xf9)");
                        
                        break;
                    case BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOTRAINING:
                        //dispatch = true;
                        // TODO: If we dispatch = true, we will send fake data
                        // which will likely crash
                        // RE info: See fec_callback_handler
                        dispatch = true;
                        NRF_LOG_INFO("Serving request for BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOTRAINING (0xfc)");
                        
                        break;
                    case BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION1:
                        //dispatch = true;
                        // TODO: If we dispatch = true, we will send fake data
                        // which will likely crash
                        dispatch = true;
                        NRF_LOG_INFO("Serving request for BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION1 (0xfa)");
                        break;
                    case BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION2:
                        //dispatch = true;
                        // TODO: If we dispatch = true, we will send fake data
                        // which will likely crash
                        dispatch = true;
                        NRF_LOG_INFO("Serving request for BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION2 (0xfb)");
                        break;
                    
                    default:
                        NRF_LOG_ERROR("Unhandled data page requested: 0x%02x", requested_page);
                        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
                        dispatch = false;
                        break;
                }
                if (dispatch) 
                {
                    ble_fec_page_evt.page = requested_page;
                    if (*(payload+4) == 0x80)
                    {
                      // Transmit until a successful acknowledge is received .
                      // Send using acknowledged messaging
                      ble_fec_page_evt.type = BLE_FEC_REQUEST_TYPE_ACKNOWLEDGED;
                    }
                    fec_callback_handler(p_fec, &ble_fec_page_evt);
                }
            }
        break;
        case BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOTRAINING:
        {
            /*
            A4 09 4F 05 FC 00 01 58 00 00 00 00 42 == 16 km/h
            A4 09 4F 05 FC 00 01 5E 00 00 00 00 44 == 17 km/h
	             64, 18 km/h
                     69, 19 km/h
                     6f, 20 km/h
                     74, 21 km/h
                     7a, 22 km/h
                     7f, 23 km/h
                     85, 24 km/h
                     8a, 25 km/h
                     90, 26 km/h
                     96, 27 km/h
                     9b, 28 km/h
                     a1, 29 km/h
                     a6, 30 km/h     
            A4 09 4F 05 FC 00 00 00 00 01 46 00 5C == Concrete 70
                                          ^ Intensity
            A4 09 4F 05 FC 00 00 00 00 02 5a 00 5C == Cattle Grid 90 
                                           ^ Intensity
                                        ^ Road feel type
            Road feel type:
                                                   
            0x01 Concrete Plates
            0x02 Cattle Grid
            0x03 Hard Cobblestone
            0x04 Soft Cobblestone
            0x05 Brick Road
            0x06 Soft Road
            0x07 Gravel
            0x08 Ice
            0x09 Wooden Boards

            Intensity: 0x00 - 0x64 (=0-100)



            */
            uint8_t isokinetic_active = *(payload +1);
            uint8_t training_setpoint = *(payload + 2);
            tacx_roadfeel_surfaces_t roadfeel_type = *(payload + 4);
            uint8_t roadfeel_intensity = *(payload + 5);

            // Save for later page request
            stored_isokinetic_active = isokinetic_active;
            stored_training_setpoint = training_setpoint; // if isokinetic_active == 0 -> training_setpoint should be 0!
            stored_roadfeel_surface = roadfeel_type;
            stored_roadfeel_intensity = roadfeel_intensity;
            
            int8_t kmh = 0;
            switch (training_setpoint)
            {  
                // Read value / 20 (or *0.05) in mps -> *3.6 => km/h
                case 0x58:
                    kmh = 16;
                    break;
                case 0x5e:
                    kmh = 17;
                    break;
                case 0x64:
                    kmh = 18;
                    break;
                case 0x69:
                    kmh = 19;
                    break;
                case 0x6f:
                    kmh = 20;
                    break;
                case 0x74:
                    kmh = 21;
                    break;                
                case 0x7a:
                    kmh = 22;
                    break;
                case 0x7f:
                    kmh = 23;
                    break;
                case 0x85:
                    kmh = 24;
                    break;
                case 0x8a:
                    kmh = 25;
                    break;
                case 0x90:
                    kmh = 26;
                    break;
                case 0x96:
                    kmh = 27;
                    break;
                case 0x9b:
                    kmh = 28;
                    break;
                case 0xa1:
                    kmh = 29;
                    break;
                case 0xa6:
                    kmh = 30;
                    break;
                default:
                    kmh = 0;

                    break;
            }
                       
            if (roadfeel_type != TACX_ROADFEEL_NONE)
            {
                switch (roadfeel_type)
                {
                     case TACX_ROADFEEL_CONCRETE_PLATES:
                        NRF_LOG_INFO("Road feel: Concrete plates, intensity: %u", roadfeel_intensity);
                        break;
                     case TACX_ROADFEEL_CATTLE_GRID:
                        NRF_LOG_INFO("Road feel: Cattle grid, intensity: %u", roadfeel_intensity);
                        break;
                     case TACX_ROADFEEL_HARD_COBBLESTONE:
                        NRF_LOG_INFO("Road feel: Hard cobblestone, intensity: %u", roadfeel_intensity);
                        break;
                     case TACX_ROADFEEL_SOFT_COBBLESTONE:
                        NRF_LOG_INFO("Road feel: Soft cobblestone, intensity: %u", roadfeel_intensity);
                        break;
                     case TACX_ROADFEEL_BRICK_ROAD:
                        NRF_LOG_INFO("Road feel: Brick road, intensity: %u", roadfeel_intensity);
                        break;
                     case TACX_ROADFEEL_SOFT_ROAD:
                        NRF_LOG_INFO("Road feel: Soft road, intensity: %u", roadfeel_intensity);
                        break;
                     case TACX_ROADFEEL_GRAVEL:
                        NRF_LOG_INFO("Road feel: Gravel, intensity: %u", roadfeel_intensity);
                        break;
                     case TACX_ROADFEEL_ICE:
                        NRF_LOG_INFO("Road feel: Ice, intensity: %u", roadfeel_intensity);
                        break;
                     case TACX_ROADFEEL_WOODEN_BOARDS:
                        NRF_LOG_INFO("Road feel: Wooden boards, intensity: %u", roadfeel_intensity);
                        break;
                     default:
                        NRF_LOG_INFO("Road feel: ???, intensity: %u", roadfeel_intensity);
                        break;
                }
            }
            else if (kmh != 0)
            {
                NRF_LOG_INFO("Command: BLE_FEC_TACX_CUSTOM_COMMAND: Training setpoint: %d km/h (not yet implemented)", kmh);
            }
            NRF_LOG_HEXDUMP_INFO(payload, 8);

            /*
            We don' know the expected packet format
            ble_fec_page_evt.page = BLE_FEC_PAGE_COMMAND_STATUS;
            ble_fec_page_evt.type = BLE_FEC_REQUEST_TYPE_ACKNOWLEDGED;
            length = 0;
            ble_fec_page_evt.payload[length++] = BLE_FEC_CONTROL_SET_BASIC_RESISTANCE; // Last command
            ble_fec_page_evt.payload[length++] = command_counter++;   // Sequence number
            ble_fec_page_evt.payload[length++] = ok_or_init(); // Result
            ble_fec_page_evt.payload[length++] = 0xff;
            ble_fec_page_evt.payload[length++] = 0xff;
            ble_fec_page_evt.payload[length++] = 0xff;
            ble_fec_page_evt.payload[length++] = basic_resistance;
            ble_fec_page_evt.payload_length = length;
            fec_callback_handler(p_fec, &ble_fec_page_evt);
            */
            dispatch = false;
            break;
            }
        default:
            NRF_LOG_ERROR("Unknown page requested: 0x%02x", ble_fec_page_evt.page);
            NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
            // 0xfa, 0xfc, 0xf9

        break;
    }
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_fec_t * p_fec, ble_evt_t const * p_ble_evt)
{
    ret_code_t                    err_code;
    ble_fec_evt_t                 evt;
    ble_fec_client_context_t    * p_client;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    
    err_code = blcm_link_ctx_get(p_fec->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }

    memset(&evt, 0, sizeof(ble_fec_evt_t));
    evt.p_fec       = p_fec;
    evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    evt.p_link_ctx  = p_client;

    //NRF_LOG_INFO("ble_fec::on_write");
    //NRF_LOG_INFO("Handle: 0x%02x, UUID: 0x%02x", p_evt_write->handle, p_evt_write->uuid.uuid);
    
    //NRF_LOG_HEXDUMP_INFO(p_evt_write->data, p_evt_write->len);
    
    
    // RX2App/FEC2: FE-C -> App
    if ((p_evt_write->handle == p_fec->rx_fec2_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        //NRF_LOG_INFO("NUS Handle = CCCD_handle")
        if (p_client != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                p_client->is_notification_enabled = true;
                evt.type                          = BLE_FEC_EVT_COMM_STARTED;
                
            }
            else
            {
                p_client->is_notification_enabled = false;
                evt.type                          = BLE_FEC_EVT_COMM_STOPPED;                
            }

            if (p_fec->data_handler != NULL)
            {
                // NRF_LOG_INFO("Calling data_handler()")
                p_fec->data_handler(p_fec, &evt);
            }

        }
    }
    // RX/FEC2: App -> FE-C
    else if ((p_evt_write->handle == p_fec->tx_fec3_handles.value_handle) &&
             (p_fec->data_handler != NULL))
    {        
        //NRF_LOG_INFO("Received something from app to FEC3");
        //NRF_LOG_HEXDUMP_INFO(p_evt_write->data, p_evt_write->len);

        evt.type                  = BLE_FEC_EVT_RX_DATA;
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;

        p_fec->data_handler(p_fec, &evt);
    }
    else
    {
        /*uint16_t uuid = p_evt_write->uuid.uuid;
        // Do Nothing. This event is not relevant for this service.
        NRF_LOG_INFO("UUID: %u", uuid);
        NRF_LOG_HEXDUMP_INFO(p_evt_write->data, p_evt_write->len);
        */
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_hvx_tx_complete(ble_fec_t * p_fec, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_fec_evt_t              evt;
    ble_fec_client_context_t * p_client;

    err_code = blcm_link_ctx_get(p_fec->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
        return;
    }

    if ((p_client->is_notification_enabled) && (p_fec->data_handler != NULL))
    {
        memset(&evt, 0, sizeof(ble_fec_evt_t));
        evt.type        = BLE_FEC_EVT_TX_RDY;
        evt.p_fec       = p_fec;
        evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_fec->data_handler(p_fec, &evt);
    }
}


void ble_fec_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_fec_t * p_fec = (ble_fec_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:            
            on_connect(p_fec, p_ble_evt);
            break;
        
        case BLE_GATTS_EVT_WRITE:            
            on_write(p_fec, p_ble_evt);
            break;
        case BLE_GATTS_EVT_HVC:
            //NRF_LOG_INFO("Rvcd: BLE_GATTS_EVT_HCV");
            break;
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:            
            on_hvx_tx_complete(p_fec, p_ble_evt);
            break;
        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            /*
            NRF_LOG_INFO("Received BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST");
            ble_gatts_evt_t const * p_gatts_evt = &p_ble_evt->evt.gatts_evt;
            ble_gatts_evt_rw_authorize_request_t const * p_auth_req = &p_gatts_evt->params.authorize_request;
            NRF_LOG_INFO("Target: Handle: 0x%02x, UUID: 0x%02x", p_auth_req->request.write.handle, p_auth_req->request.write.uuid.uuid);
            */
            break;

        default:
            // NRF_LOG_INFO("Other event ID: 0x%02x", p_ble_evt->header.evt_id);            
            break;
    }
}


uint32_t ble_fec_init(ble_fec_t * p_fec, ble_fec_init_t const * p_fec_init)
{
    ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         fec_base_uuid = TACX_BASE_UUID;
    ble_add_char_params_t add_char_params;

    VERIFY_PARAM_NOT_NULL(p_fec);
    VERIFY_PARAM_NOT_NULL(p_fec_init);

    // Initialize the service structure.
    p_fec->data_handler = p_fec_init->data_handler;

    /**@snippet [Adding proprietary Service to the SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&fec_base_uuid, &p_fec->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_fec->uuid_type;
    ble_uuid.uuid = BLE_UUID_TACX_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_fec->service_handle);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the RX Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = BLE_UUID_TACX_TX_FEC3_CHARACTERISTIC;
    add_char_params.uuid_type                = p_fec->uuid_type;
    add_char_params.max_len                  = BLE_FEC_MAX_TX_CHAR_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_fec->service_handle, &add_char_params, &p_fec->tx_fec3_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add the TX Characteristic.
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_TACX_RX_FEC2_CHARACTERISTIC;
    add_char_params.uuid_type         = p_fec->uuid_type;
    add_char_params.max_len           = BLE_FEC_MAX_RX_CHAR_LEN;
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_fec->service_handle, &add_char_params, &p_fec->rx_fec2_handles); 
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }       
           
    registerFECCallback(p_fec, fec_callback_handler);
    return err_code; // Implicit 0
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
}


void ble_fec_on_cscs_evt(ble_fec_t * p_fec, ble_cscs_c_evt_t * p_cscs_c_evt)
{
;
}

void fec_callback_handler(ble_fec_t *p_fec, ble_fec_page_evt_t * p_fec_page_evt)
{
    uint8_t data[32];
    uint16_t length;
    uint8_t idx = 0;

    char buf[128];
    
    // Fill an open slot with page 
    if (p_fec_page_evt->page == BLE_FEC_PAGE_OPEN_SLOT)
    {
        p_fec_page_evt->page = BLE_FEC_PAGE_SPECIFIC_TRAINER_STATIONARY_BIKEDATA;
    }

    // FE-State
    uint8_t state = 0x03; // Check FTMS handling! 0x2 = IDLE, 0x3 = InUse

    memset(data,0,sizeof(data));
    data[idx++] = 0xa4;
    data[idx++] = 0x09; // TODO: Needs tweaking depending on page?
    data[idx++] = p_fec_page_evt->type;
    data[idx++] = 0x05;
    switch (p_fec_page_evt->page)
    {
        case BLE_FEC_PAGE_GENERAL_FE_DATA: // 0x10 = 16
            //NRF_LOG_INFO("Building page BLE_FEC_PAGE_GENERAL_FE_DATA");
            data[idx++] = BLE_FEC_PAGE_GENERAL_FE_DATA; // Page number
            data[idx++] = 0x19; // 0x19 = Equipment type Indoor bike
            data[idx++] = getFECElapsedTime(); // Elpased time in value * 0.25 sec;.
            data[idx++] = getFECDistanceTraveled(); // Distance traveled in m
            // int(items[idx+1],16) << 8) + int(items[idx],16)) / 1000
            
            uint16_t i_speed_mm_s = (uint16_t) (getInstantaneousSpeed() * 1000.0);
            char buf3[16];
            sprintf(buf, "%.2f km/h", getAverageSpeed());           
            // NRF_LOG_INFO("Inst speed (m/s): %s", buf);
            //NRF_LOG_INFO("Inst speed (km/h): %u", i_speed_mm_s);

            data[idx++] = i_speed_mm_s & 0xff; // i_speed LSB;
            data[idx++] = i_speed_mm_s >> 8; // i_speed MSB;

            data[idx++] = 0xff; // No HRS

            uint8_t capa_0x10 = 0x04; // Fixed: Will transmit distance traveled in byte 3
            data[idx++] = state << 4 | capa_0x10; 
            
            if (LOGHERE)
            {
                //sprintf(buf, "Page: 0x%02x, Eq.-Type: 0x%02x, Time: %u, Distance: %u, Speed: %u, HRS: 0x%02x: Capa: 0x%02x, State: 0x%02x", data[4], data[5], data[6], data[7], i_speed_mm_s, data[10], data[11], capa_0x10, state);
                //NRF_LOG_INFO("%s", buf);
            }
            break;
        case BLE_FEC_PAGE_SPECIFIC_TRAINER_STATIONARY_BIKEDATA: // 0x19 = 25            
            //NRF_LOG_INFO("Building page BLE_FEC_PAGE_SPECIFIC_TRAINER_STATIONARY_BIKEDATA");
            data[idx++] = BLE_FEC_PAGE_SPECIFIC_TRAINER_STATIONARY_BIKEDATA; // Page number
            data[idx++] = getFECEventcount(); // Eventcount
            data[idx++] = (uint8_t) getAverageCadence(); // RPM

            uint16_t acc_power = getFECAccumulatedPower();
            data[idx++] = acc_power & 0xff; // acc_power LSB;
            data[idx++] = acc_power >> 8;   // acc_power MSB;

            uint16_t i_power = (uint16_t) getInstantaneousPower(); // Cast to uint16_t
            uint8_t trainer_status = 0x00;
            // Trainer status => 0 (No calib, no spindown required)
            data[idx++] = i_power & 0xff; // i_power LSB;
            data[idx++] = i_power >> 8 | (trainer_status << 3);   // i_power MSB + Trainer State (shifted)

            uint8_t flags = 0x00; // 0=Trainer operating at the target power, or no target power set, 3=Undetermined, maximum or minimum target power reached                        
            data[idx++] = (state << 4) | flags; // 20, 30 or 33
            if (LOGHERE)
            {
                //sprintf(buf, "Page: 0x%02x, EvtCnt: 0x%u, InstCad: %u, AccWatts: %u, InstWatt: %u, TrainStatus: 0x%02x: Flags: 0x%02x, FE-State: 0x%02x", data[4], data[5], data[6],acc_power, i_power, trainer_status, flags, state);
                //NRF_LOG_INFO("%s", buf);
                sprintf(buf,"\t\t\t\t\t\t\t\t\t\t\t\tSpeed: %.2f km/h, Grade: %.2f%% -> Watt: %u", getAverageSpeed(), ((((double) getGrade()) * 0.01) - 200.0), i_power);
                NRF_LOG_INFO("%s", buf);
            }
            break;                
        case BLE_FEC_PAGE_COMMON_MANUFACTURER_IDENT: // 0x50 = 80
            //NRF_LOG_INFO("Building page BLE_FEC_PAGE_COMMON_MANUFACTURER_IDENT");
            data[idx++] = BLE_FEC_PAGE_COMMON_MANUFACTURER_IDENT; // Page number
            /*
            50=Manufacturer Identification 
                        Common data page 80/0x50 shall [MD_0009] transmit the manufacturers ID, model number, and hardware revision

                        Manufacturer ID Tacx == 89 0x0059 (16 bit), according to FITs profile.xlsx
                        Tacx Flow = 2240 = 0x08c0

                        a4094e0550 ffff01 5900     c008 26  ff leave out?)
                                       hw rev ^Tacx	^Model number
                        */
            data[idx++] = 0xff;
            data[idx++] = 0xff;
            data[idx++] = 0x01; // HW_REVISION
            data[idx++] = 0x59; // Identifier Tax 0x0059
            data[idx++] = 0x00; // Identifier Tax 0x0059
            data[idx++] = 2875 & 0xff;
            data[idx++] = (2875 >> 8) & 0xff;            
            break;
        case BLE_FEC_PAGE_COMMON_PRODUCT_INFORMATION: // 0x51 = 81
            //NRF_LOG_INFO("Building page BLE_FEC_PAGE_COMMON_PRODUCT_INFORMATION");
            data[idx++] = BLE_FEC_PAGE_COMMON_PRODUCT_INFORMATION; // Page number
            // Based on Felix Tacx Flow
            data[idx++] = 0xff;
            data[idx++] = 0x01;
            data[idx++] = 0x16;
            data[idx++] = 0xa0;
            data[idx++] = 0x00;
            data[idx++] = 0x00;
            data[idx++] = 0x00;            
            break;
        case BLE_FEC_PAGE_COMMON_FE_CAPABILITIES: // 0x36 = 54
            //NRF_LOG_INFO("Building page BLE_FEC_PAGE_COMMON_FE_CAPABILITIES");
            data[idx++] = BLE_FEC_PAGE_COMMON_FE_CAPABILITIES;
            data[idx++] = 0xff;
            data[idx++] = 0xff;
            data[idx++] = 0xff;
            data[idx++] = 0xff;

            uint16_t max_resistance = 250;
            data[idx++] = max_resistance & 0xff;
            data[idx++] = max_resistance >> 8;

            uint8_t capabilities = 0x7; // Bit 0 = Support Basic resistance, Bit 1 = Support Target Power, Bit 2 = Support Simulation
            data[idx++] = capabilities;
            break;
        case BLE_FEC_PAGE_GENERAL_FE_METABOLIC_DATA: // 0x12 = 18
            data[idx++] = BLE_FEC_PAGE_GENERAL_FE_METABOLIC_DATA;
            data[idx++] = 0xff;
            data[idx++] = 0xff;
            data[idx++] = 0xff;
            data[idx++] = 0xff;
            data[idx++] = 0xff;

            data[idx++] = 0x00; // Calories
            uint8_t capa_0x12 = 0x00; // 0 = does not transfer calories!            
            data[idx++] = state << 4 | capa_0x12; 
            break;
        case BLE_FEC_PAGE_GENERAL_SETTINGS: // 0x11 = 17
            data[idx++] = BLE_FEC_PAGE_GENERAL_SETTINGS;
            data[idx++] = 0xff;
            data[idx++] = 0xff;
            data[idx++] = (uint8_t) CIRCUMFERENCE_WHEEL_CM;
            
            uint8_t incline = 0;
            data[idx++] = incline & 0xff;
            data[idx++] = (incline >> 8) & 0xff;

            uint8_t percentage_resistance = (uint8_t) (2 * 100.0 * (double) resistance_level / NUM_RESISTANCE_LEVELS);
            data[idx++] = percentage_resistance;

            uint8_t capa_0x11 = 0x00; // Fixed: 0 for this page!            
            data[idx++] = state << 4 | capa_0x11; 
            break;
        case BLE_FEX_TACX_CUSTOM_DATAPAGE_TACXSETTING:
            // NRF_LOG_INFO("Building fake BLE_FEX_TACX_CUSTOM_DATAPAGE_TACXSETTING");
            // TODO: This page will crash the client!
            data[idx++] = BLE_FEX_TACX_CUSTOM_DATAPAGE_TACXSETTING;
            data[idx++] = 0x00; // Undef
            data[idx++] = 0x00; // Undef
            data[idx++] = 0x00; // Undef
            data[idx++] = 0xe8; // 8 bit bike weight (bike weight == 100 kg * 10 = 0x3e8)
            data[idx++] = 0xe3; // 4 bit bike weight, 4 bit user weight (bike = 11kg * 10 = 0x6e)
            data[idx++] = 0x06; // // 8 bit user weight
            data[idx++] = 0x01; // ANT disabled
            /*
            100 lg, 11kg = 100kg = 1000 = 0x3e8, 11kg = 110 = 0x4
[00:15:21.826,110] <error> ble_fec: Unknown page requested: 0xF9
[00:15:21.832,092] <info> ble_fec:  A4 09 4F 05 F9 00 00 00|.	O.....
[00:15:21.832,092] <info> ble_fec:  E8 E3 06 01 12         |.....   

            */
            break;
        case BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION1:
        /*
        Neo Version1:
        public class NeoVersion1 {
    @U8BIT(7)
    public int bootloaderAccessCode;
    @U8BIT(6)
    public int bootloaderResult;
    @U8BIT(3)
    public int communicationBootloaderBuild;
    @U8BIT(1)
    public int communicationBootloaderMayor;
    @U8BIT(2)
    public int communicationBootloaderMinor;
    @Page(250)
    int page;
    }
    */
            // NRF_LOG_INFO("Building fake BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION1");
            // TODO: This page will crash the client!
            data[idx++] = BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION1;
            data[idx++] = 0x00; // communicationBootloaderMajor, DFU_NORDIC_BOOTLOADER
            data[idx++] = 0x02; // communicationBootloaderMinor, DFU_NORDIC_BOOTLOADER
            data[idx++] = 0x01; // communicationBootloaderBuild, DFU_NORDIC_BOOTLOADER
            data[idx++] = 0x00; // ?
            data[idx++] = 0x00; // ?
            data[idx++] = 0x00; // bootloaderResult
            data[idx++] = 0x00; // bootloaderAccessCode
            /*
            1st communicationBootloaderMajor
            2nd communicationBootloaderMinro
            3rd communicationBootloaderBuild
            4th 0x00?
            5th 0x00?
            6th bootloaderResult
            7th bootloaderAccessCode
            */
            break;
        case BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION2:
            data[idx++] = BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOVERSION2;
            data[idx++] = 0x00; // communicationMajor, DFU_NORDIC_APPLICATION
            data[idx++] = 0x07; // communicatonnMinor, DFU_NORDIC_APPLICATION
            data[idx++] = 0x04; // communicatonnBuild, DFU_NORDIC_APPLICATION
            data[idx++] = 0x00; // motorControlMajor, DFU_DSP_APPLICATION
            data[idx++] = 0x08; // motorControlMinor, DFU_DSP_APPLICATION
            data[idx++] = 0x04; // motorControlBuild, DFU_DSP_APPLICATION
            data[idx++] = 0x00; // Undef
            /*
            1st: communicationMajor 
2nd: communicatonnMinor
3rd: communicationBuild
4th: motorControlMayor
5th: motorControlMinor
6th: motorControlBuild*/
        case BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOTRAINING:
            // TODO: This page will crash the client
            /*
            public class NeoTraining {
    static final int MAX_TRAINING_SETPOINT = 168;
    static final int MIN_TRAINING_SETPOINT = 84;
    @U8BIT(4)
    public int future_use1;
    @U8BIT(7)
    public int future_use2;
    @Page(AntPlusConstants.FEC.PAGE_MANUFACTURER_SPECIFIC_252)
    int page;
    @U8BIT(1)
    public int power_correction;
    @U8BIT(6)
    public int roadsurface_intensity;
    @U8BIT(5)
    public int roadsurface_type;
    @U8BIT(3)
    public int training_setpoint;
    @U8BIT(2)
    public int training_type;
    */
            // NRF_LOG_INFO("Building page BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOTRAINING");
            data[idx++] = BLE_FEC_TACX_CUSTOM_DATAPAGE_NEOTRAINING;
            data[idx++] = 0x00; // Power correction
            data[idx++] = stored_isokinetic_active; // Training type setpoint
            data[idx++] = stored_training_setpoint;
            data[idx++] = 0x00; // Unknown
            data[idx++] = stored_roadfeel_surface;
            data[idx++] = stored_roadfeel_intensity;
            data[idx++] = 0x00; // Unknown
            break;
            /*
            A4 09 4F 05 FC 00 01 58 00 00 00 00 42 == 16 km/h
            A4 09 4F 05 FC 00 01 5E 00 00 00 00 44 == 17 km/h
	             64, 18 km/h
                     69, 19 km/h
                     6f, 20 km/h
                     74, 21 km/h
                     7a, 22 km/h
                     7f, 23 km/h
                     85, 24 km/h
                     8a, 25 km/h
                     90, 26 km/h
                     96, 27 km/h
                     9b, 28 km/h
                     a1, 29 km/h
                     a6, 30 km/h     
            A4 09 4F 05 FC 00 00 00 00 01 46 00 5C == Concrete 70
                                          ^ Intensity
            A4 09 4F 05 FC 00 00 00 00 02 5a 00 5C == Cattle Grid 90 
                                           ^ Intensity
                                        ^ Road feel type
            Road feel type:
                                                   
            0x01 Concrete Plates
            0x02 Cattle Grid
            0x03 Hard Cobblestone
            0x04 Soft Cobblestone
            0x05 Brick Road
            0x06 Soft Road
            0x07 Gravel
            0x08 Ice
            0x09 Wooden Boards

            Intensity: 0x00 - 0x64 (=0-100)



            */
        case BLE_FEC_PAGE_SET_USER_CONFIGURATION: // 0x37/55
            data[idx++] = BLE_FEC_PAGE_SET_USER_CONFIGURATION;
            for (uint8_t payload_idx=0; payload_idx<p_fec_page_evt->payload_length; payload_idx++)
            {
                data[idx++] = p_fec_page_evt->payload[payload_idx];
            }
            break;
            
        case BLE_FEC_PAGE_COMMAND_STATUS: // 0x47/71
            data[idx++] = BLE_FEC_PAGE_COMMAND_STATUS;
            for (uint8_t payload_idx=0; payload_idx<p_fec_page_evt->payload_length; payload_idx++)
            {
                data[idx++] = p_fec_page_evt->payload[payload_idx];
            }
            break;
        

        default:
            NRF_LOG_ERROR("Error! Page not implemented: %u", p_fec_page_evt->page);
    }

    // Add checksum;
    // python3 -c "print(hex(0xa4 ^ 0x09 ^ 0x4e ^ 0x05 ^ 0x50  ^ 0xff ^ 0xff ^ 0x01 ^ 0x59 ^ 0x00 ^ 0x3b ^ 0x0b ))" -> returns checksum

    uint8_t checksum = 0x00;
    for (int chk_idx = 0; chk_idx < idx; chk_idx++) checksum ^= data[chk_idx];
    data[idx++] = checksum;
    
    length = idx;
    // NRF_LOG_HEXDUMP_INFO(data, length);
    ble_fec_data_send(p_fec,data, &length, p_fec->conn_handle);
    // NRF_LOG_INFO("Sending out FEC2");
    
    // What do we need to send?
    // Data generated is a complete ANT message, sent out at 4Hz! (This sub assumes we send at 1 Hz!)
    // 1st byte: 0xA4 - Sync
    // 2nd byte: 0x.. - Length
    // 3rd byte: 0x.. - Message ID/Data Type/
    // 4th byte: 0x.. - Channel (ignored when sent to the trainer)
    // ... length-1 databytes
    // nth byte: 0x00 - Checksum: XOR of ALL bytes excluding sync byte (others say INCLUDING!)
    // Example: 0xa4 0x09 0x4e 0x05 0x10 0x19 0x04 0x00 0x00 0x00 0xff 0x24 0x30 (0x30 == chksum)

    // FEC-Datapage: 0x19, EventCnt: 11, InstCad: 86, SumPow: 13546, InstPow: 175, TStatus: 12, Flags: 0, FE-State: 6, ChkSum: 131
    // FEC-Datapage: 0x19, EventCnt: 12, InstCad: 86, SumPow: 13721, InstPow: 175, TStatus: 12, Flags: 0, FE-State: 6, ChkSum: 246
    // FEC-Datapage: 0x19, EventCnt: 13, InstCad: 86, SumPow: 13896, InstPow: 175, TStatus: 12, Flags: 0, FE-State: 6, ChkSum: 37
    // FEC-Datapage: 0x19, EventCnt: 14, InstCad: 86, SumPow: 14067, InstPow: 171, TStatus: 12, Flags: 0, FE-State: 6, ChkSum: 153
    // Eventcount rollover 8 bit
    // Sum pow rollover: 16 bit
    // 

    // FEC-Datapage: 0x10, Equipment: Trainer/Stationary Bike, ElapsedTime: 58.75s, Distance: 162m, InstSpeed: 7.78m/s, 28.00 km/h, HRS: 255, Capabilities: 4, State: 6, ChkSum: 18
    // FEC-Datapage: 0x10, Equipment: Trainer/Stationary Bike, ElapsedTime: 59.25s, Distance: 162m, InstSpeed: 7.78m/s, 28.00 km/h, HRS: 255, Capabilities: 4, State: 6, ChkSum: 20
    // FEC-Datapage: 0x10, Equipment: Trainer/Stationary Bike, ElapsedTime: 60.25s, Distance: 173m, InstSpeed: 7.75m/s, 27.90 km/h, HRS: 255, Capabilities: 4, State: 6, ChkSum: 32
    // FEC-Datapage: 0x10, Equipment: Trainer/Stationary Bike, ElapsedTime: 60.75s, Distance: 173m, InstSpeed: 7.75m/s, 27.90 km/h, HRS: 255, Capabilities: 4, State: 6, ChkSum: 34
    // FEC-Datapage: 0x10, Equipment: Trainer/Stationary Bike, ElapsedTime: 61.25s, Distance: 179m, InstSpeed: 7.64m/s, 27.50 km/h, HRS: 255, Capabilities: 4, State: 6, ChkSum: 169
    // FEC-Datapage: 0x10, Equipment: Trainer/Stationary Bike, ElapsedTime: 61.75s, Distance: 185m, InstSpeed: 7.69m/s, 27.70 km/h, HRS: 255, Capabilities: 4, State: 6, ChkSum: 122
    // FEC-Datapage: 0x10, Equipment: Trainer/Stationary Bike, ElapsedTime: 62.25s, Distance: 185m, InstSpeed: 7.69m/s, 27.70 km/h, HRS: 255, Capabilities: 4, State: 6, ChkSum: 116


    // First try: Just send once per sec one 10 then one 19 then repeat   
}

uint32_t ble_fec_data_send(ble_fec_t * p_fec,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    ble_fec_client_context_t * p_client;

    VERIFY_PARAM_NOT_NULL(p_fec);    

    err_code = blcm_link_ctx_get(p_fec->p_link_ctx_storage, conn_handle, (void *) &p_client);
    VERIFY_SUCCESS(err_code);

    if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!p_client->is_notification_enabled)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (*p_length > BLE_FEC_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_fec->rx_fec2_handles.value_handle;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = p_length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    
    uint8_t buf[64];
    hex2str(buf, sizeof(buf), p_data, *p_length);
    NRF_LOG_INFO("Send: %s", buf);

    
    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}


#endif // NRF_MODULE_ENABLED(BLE_NUS)
