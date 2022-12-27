#include "helper.h"
#include "stdio.h"
#include "assert.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "math.h"
#include "nrfx_twim.h"
#include "oled_controller.h"


#include "u8g2.h"
#include "u8x8.h"

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(U8G2_OLED_CONTROLLER)

#define NRF_LOG_MODULE_NAME oled
#if OLED_CONFIG_LOG_ENABLED 
#define NRF_LOG_LEVEL       OLED_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  OLED_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR OLED_CONFIG_DEBUG_COLOR
#else // OLED_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       3
#endif // OLED_CONFIG_LOG_ENABLED
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

bool detected_device = false;

#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

#define TOP_ROW 30
#define MIDDLE_ROW 45
#define BOTTOM_ROW 60
static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(TWI_INSTANCE_ID);

APP_TIMER_DEF(m_oled_timer_id);
 
static volatile bool m_xfer_done = false;
static uint8_t m_sample;
static bool readsomething = false;
static u8g2_t u8g2;

oled_data_t oled_data;

static void oled_redraw_handler(void * p_context)
{
    if (!detected_device)
    {        
        return;
    }

    u8g2_ClearBuffer(&u8g2);   
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr); 
    
    oled_printKmh(oled_data.kmh);
    oled_printCadence(oled_data.cadence);
    oled_printPower(oled_data.power);
    // oled_printGear(oled_data.gear);
    oled_printGearRatio(oled_data.gear_ratio);
    oled_printRawResistance(oled_data.raw_resistance);
    // oled_printSimResistance(oled_data.sim_resistance);
    oled_printTrainerResistance(oled_data.trainer_resistance);

    if (oled_data.gear_down_counter>0)
    {
        oled_printGearIndicator(GEAR_DOWN_INDICATOR);
        oled_data.gear_down_counter--;
    }
    else if (oled_data.gear_up_counter>0)
    {
        oled_printGearIndicator(GEAR_UP_INDICATOR);
        oled_data.gear_up_counter--;
    }

    oled_data.tick =! oled_data.tick;
    if (oled_data.tick)
    {
        oled_printTick();
    }

    u8g2_SendBuffer(&u8g2);
}

void oled_printTick()
{
    oled_printStringAt(BOTTOM_ROW, 80, ".", false, false);          
}

void oled_printGearIndicator(gear_indicator_t indicator)
{
    switch (indicator)
    {
        case GEAR_UP_INDICATOR:
            oled_printStringAt(BOTTOM_ROW, 70, "U", false, false);         
            break;
        case GEAR_DOWN_INDICATOR:
            oled_printStringAt(BOTTOM_ROW, 70, "D", false, false);         
            break;
        default:
            break;
    }    
}
static void oled_printRawResistance(int16_t raw_resistance)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "RR: %d", raw_resistance);
    oled_printStringAt(BOTTOM_ROW, 10, buf, false, false);
}

static void oled_printSimResistance(int16_t sim_resistance)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "SR: %d", sim_resistance);
    oled_printStringAt(BOTTOM_ROW, 50, buf, false, false);
}

static void oled_printTrainerResistance(uint8_t sim_resistance)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "TR: %u", sim_resistance);
    oled_printStringAt(BOTTOM_ROW, 90, buf, false, false);
}

static void oled_printKmh(double kmh)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "Km/h: %.0f", kmh);
    oled_printStringAt(MIDDLE_ROW, 70, buf, false, false);   
}

static void oled_printCadence(double rpm)
{
    if (!detected_device)
    {        
        return;
    }

    char buf[16];
    sprintf(buf, "RPM: %.0f", rpm);
    oled_printStringAt(TOP_ROW, 70, buf, false, false);
}

static void oled_printPower(uint16_t power)
{
    if (!detected_device)
    {        
        return;
    }

    char buf[16];
    sprintf(buf, "Pow: %u", power);
    oled_printStringAt(MIDDLE_ROW, 10, buf, false, false);
}

static void oled_printGearRatio(double gear_ratio)
{
    if (!detected_device)
    {        
        return;
    }

    char buf[16];
    //sprintf(buf, "Gear: %d", gear_offset);
    sprintf(buf, "Gear: %.2f", gear_ratio);
    oled_printStringAt(TOP_ROW, 10, buf, false, false);
}

static void oled_PrintGear(int8_t gear)
{
    if (!detected_device)
    {        
        return;
    }

    char buf[16];
    //sprintf(buf, "Gear: %d", gear_offset);
    sprintf(buf, "Gear: %d", gear);
    oled_printStringAt(TOP_ROW, 10, buf, false, false);
}


void oled_printStringAt(uint8_t x, uint8_t y, char *str, bool cls, bool printImmediately)
{
    if (!detected_device)
    {        
        return;
    }
    if (cls)
    {
        u8g2_ClearBuffer(&u8g2);
    }
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);    
    u8g2_DrawStr(&u8g2, y, x, str);

    if (printImmediately)
    {
        u8g2_SendBuffer(&u8g2);
    }
}

void oled_initialize()
{
    ret_code_t err_code;
    uint8_t address;
        
//    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
//    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    twi_init();
    
    
    /*
    readsomething=false;
    for (address = 0x0; address <= TWI_ADDRESSES; address++)
    {
        if (detected_device) break;        
        m_xfer_done = false;        
        err_code = nrf_drv_twi_rx(&m_twi, address, &m_sample, sizeof(m_sample));
        while (!m_xfer_done)
        {
            __WFE();
        }
        if (readsomething)
        {
            detected_device = true;
            
            NRF_LOG_INFO("TWI device detected at address 0x%x: Read value 0x%02x", address, m_sample);
        }
        NRF_LOG_FLUSH();
    }

    if (!detected_device)
    {
        NRF_LOG_ERROR("No device was found.");
        NRF_LOG_FLUSH();        
        return;
    }
    
    */
    
   if (detected_device) return;        
   m_xfer_done = false;        
   err_code = nrfx_twim_rx(&m_twi, OLED_ADDR, &m_sample, sizeof(m_sample));
   while (!m_xfer_done)
   {
       __WFE();
   }
   if (readsomething)
   {
       detected_device = true;       
       NRF_LOG_INFO("TWI device detected at address 0x%x: Read value 0x%02x", address, m_sample);
   }
    
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_HW_com_nrf52832, u8g2_nrf_gpio_and_delay_cb);    
    u8g2_SetI2CAddress(&u8g2, OLED_ADDR);
    
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2,0);     

    err_code = app_timer_create(&m_oled_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                oled_redraw_handler);
    APP_ERROR_CHECK(err_code);
    
}

void startOLEDUpdates()
{   
    if (!detected_device)
    {        
        return;
    }
    oled_data.tick = true;
    ret_code_t err_code = app_timer_start(m_oled_timer_id, APP_TIMER_TICKS(REDRAW_OLED), NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Start OLED redraw()");
}

void stopOLEDUpdates()
{   
    if (!detected_device)
    {        
        return;
    }
    ret_code_t err_code = app_timer_stop(m_oled_timer_id);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Stop OLED redraw()");
}

static void twi_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRFX_TWIM_EVT_ADDRESS_NACK:
        {
            NRF_LOG_ERROR("Got back ADDRESS NACK");
            m_xfer_done = true;            
            break;
        }
        case NRFX_TWIM_EVT_DATA_NACK:
        {
            NRF_LOG_ERROR("Got back DATA NACK");
            m_xfer_done = true;            
            break;
        }        
        case NRFX_TWIM_EVT_DONE:
        {
            if (p_event->xfer_desc.type == NRFX_TWIM_XFER_RX)
            {
                readsomething=true;
            }
            m_xfer_done = true;
            break;
        }        
        default:
        {
            m_xfer_done = false;
            break;
        }
    }    
}

static void twi_init (void)
{
    ret_code_t err_code;

    const nrfx_twim_config_t twi_oled_config = {
       .scl                = OLED_I2C_PIN_SCL,
       .sda                = OLED_I2C_PIN_SDA,
       .frequency          = NRF_TWIM_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_MID, //was _HIGH - While debugging TWI(?) issues
       //.clear_bus_init     = false,       
    };

    err_code = nrfx_twim_init(&m_twi, &twi_oled_config, twi_handler, NULL);
    //err_code = nrf_drv_twi_init(&m_twi, &twi_oled_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrfx_twim_enable(&m_twi);
}

static uint8_t u8g2_nrf_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)

{
    switch(msg)
    {   
        case U8X8_MSG_DELAY_MILLI:
            // NRF_LOG_INFO("nrf_delay_ms(%d)", arg_int);
            nrf_delay_ms(arg_int);
            break;

        case U8X8_MSG_DELAY_10MICRO:
            // NRF_LOG_INFO("nrf_delay_us(%d)", 10*arg_int);
            nrf_delay_us(10*arg_int);
            break;
        
        default:
            u8x8_SetGPIOResult(u8x8, 1); // default return value
            break;  
    }
    return 1;
}

static uint8_t u8x8_HW_com_nrf52832(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    uint8_t *data;
    bool res = false;
    ret_code_t err_code;    
    static uint8_t buffer[32];
    static uint8_t buf_idx;
    switch(msg)
    {
      case U8X8_MSG_BYTE_SEND:
      {
            data = (uint8_t *)arg_ptr;      
            while( arg_int > 0 )
            {
              buffer[buf_idx++] = *data;
              data++;
              arg_int--;
            }      
            break;  
      }      
      case U8X8_MSG_BYTE_START_TRANSFER:                    
      {
            buf_idx = 0;            
            m_xfer_done = false;
            break;
      }
      case U8X8_MSG_BYTE_END_TRANSFER:
      {
            // NRF_LOG_INFO("Dumping send buffer of len %d ('buf_idx')", buf_idx);
            // NRF_LOG_HEXDUMP_INFO(buffer, buf_idx);
            uint8_t addr = u8x8_GetI2CAddress(u8x8);
            // NRF_LOG_INFO("Now calling 'nrf_drv_twi_tx(&m_twi, 0x%02x, buffer, buf_idx, false)'", addr);
           
            err_code = nrfx_twim_tx(&m_twi, u8x8_GetI2CAddress(u8x8) , buffer, buf_idx, false);
            APP_ERROR_CHECK(err_code);
            while (!m_xfer_done)
            {
                __WFE();
            }
            break;
      }
      default:
            return 0;
    }
    return 1;
}

#endif // U8G2_OLED_CONTROLLER