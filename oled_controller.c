#include "sdk_config.h"
#include "helper.h"
#include "stdio.h"
#include "assert.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "math.h"
#include "nrfx_spim.h"
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
#define SPI_INSTANCE 1
static const nrfx_spim_t m_spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);


APP_TIMER_DEF(m_oled_timer_id);
 
static volatile bool m_xfer_done = false;
static uint8_t m_sample;
static bool readsomething = false;
static u8g2_t u8g2;

oled_data_t oled_data;

volatile bool doUpdateOLED;

void updateOLEDDisplay()
{
    if (doUpdateOLED)
    {
        oled_redraw_handler(NULL);
    }

    doUpdateOLED = false;
}

static void oled_timer_handler(void * p_context)
{
    if (!doUpdateOLED)
    {
        doUpdateOLED = true;
    }
}

static void oled_redraw_handler(void * p_context)
{
    if (!detected_device)
    {        
        return;
    }

    oled_data.tick =! oled_data.tick;

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

    oled_printSterzoDeactivatedIndicator();
    oled_printFTMSActivatedIndicator();

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

    if (oled_data.trigger_down_counter>0)
    {
        oled_printTriggerDownIndicator();
        oled_data.trigger_down_counter--;
    }
    else if (oled_data.trigger_up_counter>0)
    {
        oled_printTriggerUpIndicator();
        oled_data.trigger_up_counter--;
    }

    if (oled_data.mode == SIM)
    {
        oled_printGrade(oled_data.grade);
        oled_printWindResistance(oled_data.wind_resistance);
        oled_printGravitationalResistance(oled_data.gravitational_resistance);
        oled_printRollingResistance(oled_data.rolling_resistance);
    
        if (oled_data.wind_change_counter > 0)
        {
            if (oled_data.tick)
            {
                oled_printWindChangeIndicator();
            }
            oled_data.wind_change_counter--;
        }

        if (oled_data.rolling_change_counter > 0)
        {
            if (oled_data.tick)
            {
                oled_printRollingChangeIndicator();
            }
            oled_data.rolling_change_counter--;
        }

        if (oled_data.gravitational_change_counter > 0)
        {
            if (oled_data.tick)
            {
                oled_printGravitationalChangeIndicator();
            }
            oled_data.gravitational_change_counter--;
        }

        oled_printmode(SIM);
    }
    else if (oled_data.mode == ERG)
    {
        oled_printmode(ERG);
    }

    if (oled_data.tick)
    {
        oled_printTick();
    }

    u8g2_SendBuffer(&u8g2);
}

static void oled_printmode(sim_mode_t mode)
{
    if (!detected_device)
    {        
        return;
    }
    switch (mode)
    {
        case SIM:
            oled_printStringAt(SIXTH_ROW, 70, "SIM", false, false);         
            break;
        case ERG:
            oled_printStringAt(SIXTH_ROW, 70, "ERG", false, false);         
            break;
        default:
            break;
    }
}
static void oled_printTick()
{
    if (!detected_device)
    {        
        return;
    }
    oled_printStringAt(THIRD_ROW, 60, ".", false, false);          
}

static void oled_printGrade(double grade)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "Gr: %.2f%%", grade);
    oled_printStringAt(FOURTH_ROW, 70, buf, false, false);
}

void oled_printGearIndicator(gear_indicator_t indicator)
{
    if (!detected_device)
    {        
        return;
    }
    switch (indicator)
    {
        case GEAR_UP_INDICATOR:
            oled_printStringAt(SIXTH_ROW, 110, "U", false, false);         
            break;
        case GEAR_DOWN_INDICATOR:
            oled_printStringAt(SIXTH_ROW, 110, "D", false, false);         
            break;
        default:
            break;
    }    
}

static void oled_printTriggerUpIndicator()
{
    if (!detected_device)
    {        
        return;
    }
    
    oled_printStringAt(THIRD_ROW, 105, "U", false, false);
}

static void oled_printTriggerDownIndicator()
{
    if (!detected_device)
    {        
        return;
    }
    
    oled_printStringAt(THIRD_ROW, 115, "D", false, false);
}



static void oled_printWindChangeIndicator()
{
    if (!detected_device)
    {        
        return;
    }
    oled_printStringAt(FOURTH_ROW, 60, ".", false, false);    
}

static void oled_printFTMSActivatedIndicator()
{
    if (!detected_device)
    {
        return;
    }

    if (oled_data.ftms_active)
    {
        oled_printStringAt(FIFTH_ROW, 70, "FTMS", false, false);    
    }
    else
    {
        oled_printStringAt(FIFTH_ROW, 70, "F-EC", false, false);    
    }
}

static void oled_printSterzoDeactivatedIndicator()
{
    if (!detected_device)
    {        
        return;
    }

    if (!oled_data.sterzo_active)
    {
        oled_printStringAt(FIFTH_ROW, 75, "x", false, false);    
    }
}


static void oled_printGravitationalChangeIndicator()
{
    if (!detected_device)
    {        
        return;
    }
    oled_printStringAt(FIFTH_ROW, 60, ".", false, false);    
}

static void oled_printRollingChangeIndicator()
{
    if (!detected_device)
    {        
        return;
    }
    oled_printStringAt(SIXTH_ROW, 60, ".", false, false);    
}

static void oled_printRawResistance(int16_t raw_resistance)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "RR: %d", raw_resistance);
    oled_printStringAt(THIRD_ROW, 10, buf, false, false);
}

static void oled_printWindResistance(int16_t wind_resistance)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "w: %d", wind_resistance);
    oled_printStringAt(FOURTH_ROW, 10, buf, false, false);
}

static void oled_printGravitationalResistance(int16_t gravitational_resistance)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "g: %d", gravitational_resistance);
    oled_printStringAt(FIFTH_ROW, 10, buf, false, false);
}

static void oled_printRollingResistance(int16_t rolling_resistance)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "r: %d", rolling_resistance);
    oled_printStringAt(SIXTH_ROW, 10, buf, false, false);
}

static void oled_printSimResistance(int16_t sim_resistance)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "SR: %d", sim_resistance);
    oled_printStringAt(THIRD_ROW, 50, buf, false, false);
}

static void oled_printTrainerResistance(uint8_t sim_resistance)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "TR: %u", sim_resistance);
    oled_printStringAt(THIRD_ROW, 70, buf, false, false);
}

static void oled_printKmh(double kmh)
{
    if (!detected_device)
    {        
        return;
    }
    char buf[16];
    sprintf(buf, "Km/h: %.0f", kmh);
    oled_printStringAt(SECOND_ROW, 70, buf, false, false);   
}

static void oled_printCadence(double rpm)
{
    if (!detected_device)
    {        
        return;
    }

    char buf[16];
    sprintf(buf, "RPM: %.0f", rpm);
    oled_printStringAt(FIRST_ROW, 70, buf, false, false);
}

static void oled_printPower(uint16_t power)
{
    if (!detected_device)
    {        
        return;
    }

    char buf[16];
    sprintf(buf, "Pow: %u", power);
    oled_printStringAt(SECOND_ROW, 10, buf, false, false);
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
    oled_printStringAt(FIRST_ROW, 10, buf, false, false);
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
    oled_printStringAt(FIRST_ROW, 10, buf, false, false);
}


void oled_printStringAt(display_row_t x, uint8_t y, char *str, bool cls, bool printImmediately)
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
        
    spi_init();        
    u8g2_Setup_ssd1327_ws_128x128_f(&u8g2, U8G2_R0, u8x8_HW_com_spi_nrf52832, u8g2_nrf_gpio_and_delay_spi_cb);

    
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2,0);     

    err_code = app_timer_create(&m_oled_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                oled_timer_handler);//oled_redraw_handler);
    APP_ERROR_CHECK(err_code);    
}


void startOLEDUpdates()
{   
    if (!detected_device)
    {        
        return;
    }

    oled_data.mode = NONE;
    oled_data.tick = true;
    NRF_LOG_INFO("Start OLED redraw()");
    ret_code_t err_code = app_timer_start(m_oled_timer_id, APP_TIMER_TICKS(REDRAW_OLED), NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Started OLED redraw()");
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

static uint8_t u8x8_HW_com_spi_nrf52832(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    uint8_t *data;
    bool res = false;
    ret_code_t err_code;    
    static uint8_t buffer[64];
    static uint8_t buf_idx = 1;
    
    switch(msg)
    {
      case U8X8_MSG_BYTE_SEND:
      {
            buf_idx = 0;  
            data = (uint8_t *)arg_ptr;            
            while( arg_int > 0 )
            {
              buffer[buf_idx++] = *data;
              data++;
              arg_int--;
            }

            // m_xfer_done = false;
            nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TX(&buffer, buf_idx);    
            err_code = nrfx_spim_xfer(&m_spi, &spim_xfer_desc,0);
            APP_ERROR_CHECK(err_code);
            /*while (!m_xfer_done)
            {
                __WFE();
            }
            */
            break;  
      }      
      case U8X8_MSG_BYTE_SET_DC:
      {            
            u8x8_gpio_SetDC(u8x8, arg_int);
            break;
      }
      case U8X8_MSG_BYTE_START_TRANSFER:                    
      {
            buf_idx = 0;                        
            
            break;
      }
      case U8X8_MSG_BYTE_END_TRANSFER:
      {                   
            break;
      }
      default:
            return 0;
    }
    return 1;
}

/* // Only used in asynchronous mode
void spi_handler(nrfx_spim_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRFX_SPIM_EVENT_DONE:
                m_xfer_done = true;
                break;       
    }
    m_xfer_done = true;
}
*/

void spi_init (void)
{
    ret_code_t err_code;

    nrfx_spim_config_t spi_oled_config = NRFX_SPIM_DEFAULT_CONFIG;

    spi_oled_config.sck_pin   = OLED_SPI_PIN_CLK;
    spi_oled_config.mosi_pin  = OLED_SPI_PIN_DI;
    spi_oled_config.ss_pin    = OLED_SPI_PIN_CS;
    spi_oled_config.frequency = NRF_SPIM_FREQ_4M; // The SSD1326 can go up to 10 MHz clock
    spi_oled_config.mode      = NRF_SPIM_MODE_0;
    spi_oled_config.ss_active_high = false;
    
    err_code = nrfx_spim_init(&m_spi, &spi_oled_config, NULL, NULL);   
    //err_code = nrfx_spim_init(&m_spi, &spi_oled_config, spi_handler, NULL); // Asynchronous mode   
    APP_ERROR_CHECK(err_code);

    // Enable the out-of-band GPIOs
    nrf_gpio_cfg_output(OLED_SPI_PIN_DC);
    nrf_gpio_cfg_output(OLED_SPI_PIN_RESET);

    detected_device = true;
}


uint8_t u8g2_nrf_gpio_and_delay_spi_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch(msg)
    {   
        case U8X8_MSG_GPIO_DC:				
            nrf_gpio_pin_write(OLED_SPI_PIN_DC, arg_int);            
            break;

        case U8X8_MSG_GPIO_RESET:            
            nrf_gpio_pin_write(OLED_SPI_PIN_RESET, arg_int);
            break;

        case U8X8_MSG_DELAY_MILLI:            
            nrf_delay_ms(arg_int);
            break;

        case U8X8_MSG_DELAY_10MICRO:            
            nrf_delay_us(10*arg_int);
            break;
        
        default:
            u8x8_SetGPIOResult(u8x8, 1); // default return value
            break;  
    }
    return 1;
}


#endif // U8G2_OLED_CONTROLLER