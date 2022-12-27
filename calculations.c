#include "sdk_common.h"
#include <string.h>

#define NRF_LOG_MODULE_NAME calculations
#if CALCULATIONS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       CALCULATIONS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  CALCULATIONS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR CALCULATIONS_CONFIG_DEBUG_COLOR
#else // CALCULATIONS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // CALCULATIONS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

// Important!

/*

https://devzone.nordicsemi.com/f/nordic-q-a/29345/nrf52832-executing-twi-function-from-app_timer-handler

NRF_LOG_FLUSH() is a blocking function and if you are using UART as backend 
this is using interrupts while printing messages. Hence, you are triggering 
interrupts from within an interrupt context and this might cause priority issues. 
If you are using RTT as backend it will not use interrupts the same way, but 
since NRF_LOG_FLUSH() is blocking and since it takes a significant amount of 
time to format a string you should never flush in an interrupt context. NRF_LOG_FLUSH() 
is intended to be used only in main context.
*/

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <string.h>
#include "nordic_common.h"
#include "app_util.h"

#include "calculations.h"
#include "ble_cscs_c.h"
#include "math.h"
#include "helper.h"
#include "assert.h"
#include "app_timer.h"
#include "oled_controller.h"
#include <ble_gattc.h> 

static calculation_helper_t calculation_helper;
volatile bool doTriggerFECEvent; 
volatile bool doCalcModel;


uint32_t pid_timer = 0;

#define BENCHMARK 0
#define SPEEDS_SIZE 5

#define PID_CONTROLLER_INTERVAL 1000
#define FEC_CONTROLLER_INTERVAL 250

static double speeds[SPEEDS_SIZE] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
static int speed_ptr = 0;
static int speed_size = 0;

static uint8_t schedule[132];
static uint8_t schedule_idx = 0;

// Only to be switched in an "error" condition when not using our setup
static bool external_wheel_sensor_data_present = false;

uint32_t getCumulativeWheelRevs()
{
  return (uint32_t) calculation_helper.cumulative_wheel_revs;
}

uint16_t getLastWheelEventTime()
{
  return calculation_helper.last_wheel_event_time;
}

uint16_t getLastCrankEventTime()
{
  return calculation_helper.last_crank_event_time;
}

void initializeCalculationHelper()
{
  memset(&calculation_helper,0,sizeof(calculation_helper_t));
  setResistanceLevel(resistance_level); // Initialize with the minimum value (from main.c to ensure values are in sync)
  setUserWeight(USER_MASS);
  setBikeWeight(EQUIPMENT_MASS); 
  setCoefficientRollingResistance(80);
  setWindResistanceCoefficient(51);
  setDraftingFactor(100);
  setGrade(20000);

  setFECMode(FEC_TARGET_POWER);
  
}
void incResistanceLevel()
{
  if (calculation_helper.resistance_level != resistance_level)
  {
    NRF_LOG_ERROR("Error resistance_level out of sync");
  }

  if (resistance_level == 32)
  {
    ///NRF_LOG_ERROR("Error resistance_level already max, can't inc");
  }
  calculation_helper.resistance_level++;
  oled_data.trainer_resistance = calculation_helper.resistance_level;
  resistance_level++;
  // NRF_LOG_INFO("Updated++");
}

void decResistanceLevel()
{
//Dangerous to log here! In a timer context!
  if (calculation_helper.resistance_level != resistance_level)
  {
    NRF_LOG_ERROR("Error resistance_level out of sync");
  }
  
  if (resistance_level == 1)
  {
    //NRF_LOG_ERROR("Error resistance_level already min, can't dec");
  }
  calculation_helper.resistance_level--;
  oled_data.trainer_resistance = calculation_helper.resistance_level;
  resistance_level--;
  //NRF_LOG_INFO("Updated--");
}

 
void setResistanceLevel(uint8_t resistance_level)
{
  calculation_helper.resistance_level = resistance_level;
  NRF_LOG_INFO("setResistanceLevel() called");
}

uint8_t getResistanceLevel()
{
  return calculation_helper.resistance_level;
}

static void updateCadenceReadings()
{
  if (calculation_helper.prev_last_crank_event_time == 0)
  {
      // Handle unrealisitic large first measurements
      calculation_helper.prev_last_crank_event_time = calculation_helper.last_crank_event_time;
  }

  if (calculation_helper.prev_cumulative_crank_revs == 0)
  {
      // Handle unrealisitic large first measurements
      calculation_helper.prev_cumulative_crank_revs == calculation_helper.cumulative_crank_revs;
  }

  calculation_helper.delta_crank_event_times  = calculation_helper.last_crank_event_time - calculation_helper.prev_last_crank_event_time;
  calculation_helper.delta_cum_crank_revs     = calculation_helper.cumulative_crank_revs - calculation_helper.prev_cumulative_crank_revs;
  
  if (calculation_helper.delta_crank_event_times == 0)
  {      
      // NRF_LOG_ERROR("                                           delta_crank_event_times = 0");
      return;
  }

  // TODO: 
  // We do not allow for 0 cadence changes because of the above changes, i.e. i_cadence and avg_cadence stay as they are
  //assert(calculation_helper.delta_crank_event_times>0);
  //assert(calculation_helper.delta_crank_event_times>0);

  // Also set it in the struct directly
  calculation_helper.i_cadence = (double) (calculation_helper.delta_cum_crank_revs) * 1024.0 / (double)(calculation_helper.delta_crank_event_times);  
  calculation_helper.avg_cadence = calculation_helper.i_cadence * 60; // 60s * 1024th  
}

double getInstantaneousCadence()
{
  return calculation_helper.i_cadence;
}

double getAverageCadence()
{
  return calculation_helper.avg_cadence;
}

uint16_t getPrevLastWheelEventTime()
{
  return calculation_helper.prev_last_wheel_event_time;
}

static double updateInstantaneousSpeed()
{
  //calculation_helper.i_speed = 3.6 * calculation_helper.delta_cum_wheel_revs * CIRCUMFERENCE_WHEEL / calculation_helper.delta_wheel_event_times;
  // New!
  if (calculation_helper.delta_wheel_event_times > 0)
  {
    //calculation_helper.i_speed = ((double) calculation_helper.delta_achieved_whole_cumulative_wheel_revs) * CIRCUMFERENCE_WHEEL * 1024.0 / ((double) calculation_helper.delta_wheel_event_times);         
    calculation_helper.i_speed =  calculation_helper.i_wheel_revs * CIRCUMFERENCE_WHEEL;
  }
  else
  {
    calculation_helper.i_speed = 0.0;
  }
  return calculation_helper.i_speed;
}

static double updateInstantaneousWheelRevs()
{
  //calculation_helper.i_wheel_revs = ((double) calculation_helper.delta_achieved_whole_cumulative_wheel_revs * 1024.0) / (double) calculation_helper.delta_wheel_event_times; 
  //calculation_helper.i_wheel_revs = (((double) calculation_helper.delta_cum_wheel_revs) * 1024.0) / ((double) calculation_helper.delta_crank_event_times / (double) calculation_helper.delta_cum_crank_revs); 
  //calculation_helper.i_wheel_revs = (((double) calculation_helper.delta_cum_wheel_revs) * 1024.0) / (calculation_helper.delta_crank_event_times-calculation_helper.delta_wheel_event_times);
  calculation_helper.i_wheel_revs = (double) calculation_helper.delta_achieved_whole_cumulative_wheel_revs * 1024.0 /(double) calculation_helper.delta_wheel_event_times;
  return calculation_helper.i_wheel_revs;
}

double getInstantaneousSpeed()
{
  return calculation_helper.i_speed;
}

static double getAveragedSpeed(double in_value)
{
    // Put value
    if (speed_size<SPEEDS_SIZE) 
    {   
        speed_size++;
    }

    speeds[(speed_ptr++) % SPEEDS_SIZE] = in_value;

    // Average it
    double sum = 0.0;
    for (int idx=0; idx<SPEEDS_SIZE-1; idx++)
    {
        sum+=speeds[idx];
    }

    double averaged = (double) sum / (double) speed_size;
    
    /*
    char buf[16];
    sprintf(buf, "%f" , sum);

    char buf2[16];
    sprintf(buf2, "%f" , averaged);
    
    NRF_LOG_INFO("Averaged km/h: Sum: %s, Elements: %d -> %s", buf, speed_size, buf2);
    */
    return averaged;
}

static double updateAverageSpeed()
{    
    // TODO: Hm, is the following correct? What if we don' pedal? immediately 0.0 km/h?
    // We don' update the values at all!
    if (calculation_helper.delta_cum_crank_revs == 0 || calculation_helper.delta_crank_event_times == 0)
    {
        return 0.0;
    }

    double intermediate_speed = 3.6 * calculation_helper.i_speed;

    /*
    calculation_helper.crank_events_per_sec = calculation_helper.avg_cadence / 60.0; // rpm can be calculated precisely because delta_cum_crank_revs and delta_last_crank_time is physical correct and precise
    calculation_helper.current_time_interval_one_crank_event = (double) calculation_helper.delta_crank_event_times / (double) calculation_helper.delta_cum_crank_revs; // Approx the _current_ time interval for ONE full crank rottion
    
    // The next one does not seem right to me? why h->cumulative_crank-revs... not the delta?
    //h->precise_cum_wheel_revs_at_last_crank_event = h->cumulative_crank_revs * getGearRatio();
    calculation_helper.precise_cum_wheel_revs_at_last_crank_event = calculation_helper.delta_cum_crank_revs * getGearRatio() + calculation_helper.cumulative_wheel_revs;

    if (calculation_helper.prev_last_wheel_event_time == 0)
    {
         // We initialize it to prev_last_crank_event, to avoid unrealisitic high readings
         calculation_helper.prev_last_wheel_event_time = calculation_helper.prev_last_crank_event_time;
    }
    else
    {
         calculation_helper.prev_last_wheel_event_time = calculation_helper.last_wheel_event_time;         
    }
    
    calculation_helper.prev_cumulative_wheel_revs = calculation_helper.cumulative_wheel_revs;
    
    // Sometimes this here goes sky-rocketing...

    //kmh = (((double) delta_cum_wheel_revs) * 3.60 * CIRCUMFERENCE_WHEEL ) / ((double)(delta_last_wheel_time));
    calculation_helper.timespan_for_wheel_events = calculation_helper.last_crank_event_time - calculation_helper.prev_last_wheel_event_time;
    calculation_helper.duration_per_wheel_event = calculation_helper.timespan_for_wheel_events / calculation_helper.precise_cum_wheel_revs_at_last_crank_event;
    calculation_helper.timespan_until_last_full_wheel_event = (uint16_t) round(calculation_helper.duration_per_wheel_event * floor(calculation_helper.precise_cum_wheel_revs_at_last_crank_event));

    // That's it?
    calculation_helper.last_wheel_event_time = (uint16_t) (calculation_helper.prev_last_wheel_event_time + calculation_helper.timespan_until_last_full_wheel_event); // Let it overflow if need be
    calculation_helper.cumulative_wheel_revs = floor(calculation_helper.precise_cum_wheel_revs_at_last_crank_event);

    // Again prevention of skyrocketing
    if (calculation_helper.prev_cumulative_wheel_revs == 0)
    {
        calculation_helper.prev_cumulative_wheel_revs = calculation_helper.cumulative_wheel_revs;
    }
    calculation_helper.delta_cum_wheel_revs = calculation_helper.cumulative_wheel_revs - calculation_helper.prev_cumulative_wheel_revs;        

    //NRF_LOG_INFO("h->last_wheel_event_time:      %d", h->last_wheel_event_time);
    //NRF_LOG_INFO("h->prev_last_wheel_event_time: %d", h->prev_last_wheel_event_time);

    calculation_helper.delta_last_wheel_time = calculation_helper.last_wheel_event_time - calculation_helper.prev_last_wheel_event_time; // We need to cope with uint16_t overflows here (happens every 64 seconds!)
    
    //NRF_LOG_INFO("h->delta_cum_wheel_revs:       %d", h->delta_cum_wheel_revs);
    //NRF_LOG_INFO("h->delta_last_wheel_time:      %d", h->delta_last_wheel_time);
    
    // TODO: Why are we calculating again??? -> Should use existing value
    double intermediate_speed = (((double) calculation_helper.delta_cum_wheel_revs) * 3.6 * CIRCUMFERENCE_WHEEL ) / ((double) (calculation_helper.delta_last_wheel_time)); 
    */
    if (intermediate_speed > 120.0)
    {
      char buf[128];
      
      stopOLEDUpdates();          
      oled_printStringAt(30,10, "CRITICAL ERROR", true, true);
    
      sprintf(buf, "Something went wrong: intermediate_speed = %f", intermediate_speed);
      NRF_LOG_INFO("%s",buf);
      NRF_LOG_FLUSH();
      debugCalculationHelper();
      assert(true==false);
      APP_ERROR_CHECK(-1);
    }
         
    calculation_helper.avg_speed = intermediate_speed;// getAveragedSpeed(intermediate_speed);    
    return calculation_helper.avg_speed;
}

uint16_t getCumulativeCrankRevs()
{
  return calculation_helper.cumulative_crank_revs;
}

// Not needed
/*
void setCumulativeCrankRevs(uint16_t cumulative_crank_revs)
{
  calculation_helper.cumulative_crank_revs = cumulative_crank_revs;
}
*/
double calcPowerFromGivenCadence(uint16_t cadence)
{

}

void debugCalculationHelper()
{
    char buf[128];
    calculation_helper_t *h = &calculation_helper; 
    NRF_LOG_INFO("uint8_t  resistance_level = %u", h->resistance_level);
    NRF_LOG_INFO("uint16_t last_crank_event_time = %u", h->last_crank_event_time);
    NRF_LOG_INFO("uint16_t prev_last_crank_event_time = %u", h->prev_last_crank_event_time);
    NRF_LOG_INFO("uint16_t cumulative_crank_revs = %u", h->cumulative_crank_revs);
  
    NRF_LOG_INFO("uint16_t prev_cumulative_crank_revs = %u", h->prev_cumulative_crank_revs);

    NRF_LOG_INFO("uint16_t cumulative_wheel_revs = %u", h->cumulative_wheel_revs);
    NRF_LOG_INFO("uint16_t prev_cumulative_wheel_revs = %u", h->prev_cumulative_wheel_revs);

    NRF_LOG_INFO("uint16_t last_wheel_event_time = %u", h->last_wheel_event_time);
    NRF_LOG_INFO("uint16_t prev_last_wheel_event_time = %u", h->prev_last_wheel_event_time);

    NRF_LOG_INFO("uint16_t timespan_until_last_full_wheel_event = %u", h->timespan_until_last_full_wheel_event);
    NRF_LOG_INFO("uint16_t timespan_for_wheel_events = %u", h->timespan_for_wheel_events);
   
    NRF_LOG_INFO("uint16_t delta_wheel_event_times = %u", h->delta_wheel_event_times);
    NRF_LOG_INFO("uint16_t delta_cum_wheel_revs = %u", h->delta_cum_wheel_revs);
   
    NRF_LOG_INFO("uint16_t delta_last_wheel_time = %u", h->delta_last_wheel_time);
   
    sprintf(buf, "double crank_events_per_sec = %f", h->crank_events_per_sec);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double current_time_interval_one_crank_event = %f", h->current_time_interval_one_crank_event);
    NRF_LOG_INFO("%s",buf);
  
    sprintf(buf, "double precise_cum_wheel_revs_at_last_crank_event = %f", h->precise_cum_wheel_revs_at_last_crank_event);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double upper_limit_i_wheel_revs_based_on_cadence = %f", h->upper_limit_i_wheel_revs_based_on_cadence);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double duration_per_wheel_event = %f", h->duration_per_wheel_event);
    NRF_LOG_INFO("%s",buf);

    NRF_LOG_INFO("uint16_t delta_crank_event_times = %u", h->delta_crank_event_times); 
    
    NRF_LOG_INFO("uint16_t delta_cum_crank_revs = %u", h->delta_cum_crank_revs); 
    
    sprintf(buf, "double i_cadence = %f", h->i_cadence);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double i_wheel_revs = %f", h->i_wheel_revs);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double i_speed = %f", h->i_speed);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double avg_speed = %f", h->avg_speed);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double avg_cadence = %f", h->avg_cadence);
    NRF_LOG_INFO("%s",buf);
       
    sprintf(buf, "double i_power = %u", h->i_power);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double ideal_cumulative_wheel_revs = %f", h->ideal_cumulative_wheel_revs);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double achieved_wheel_revs_during_crank_time = %f", h->achieved_wheel_revs_during_crank_time);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double tweak_wheel_cadence = %f", h->tweak_wheel_cadence);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double timespan_per_wheel_rev = %f", h->timespan_per_wheel_rev);
    NRF_LOG_INFO("%s",buf);

    sprintf(buf, "double fraction = %f", h->fraction);
    NRF_LOG_INFO("%s",buf);
    
    NRF_LOG_INFO("int16_t integrator = %d", h->integrator);
#if HANDLE_FREEROLL
    NRF_LOG_INFO("uint16_t freeroll_duration = %u", h->freeroll_duration);
    
    sprintf(buf, "crank_events_per_sec: %f", h->crank_events_per_sec);
    NRF_LOG_INFO("%s",buf);
#endif 
}

double getAverageSpeed()
{
  return calculation_helper.avg_speed;
}

void stopProcessing()
{
  ret_code_t err_code;
  
  if (m_pid_timer_id!=NULL)
  {
    err_code = app_timer_stop(m_pid_timer_id);
    APP_ERROR_CHECK(err_code);
  }

  if (m_fec_timer_id!=NULL)
  {
    err_code = app_timer_stop(m_fec_timer_id);
    APP_ERROR_CHECK(err_code);
  }
}

void startProcessing()
{
  // Create timers
  ret_code_t err_code = app_timer_create(&m_pid_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                //pid_handler);
                                calculation_handler);
  APP_ERROR_CHECK(err_code);   
  
  // Fire every PID_CONTROLLER_INTERVAL ms
  NRF_LOG_DEBUG("Calculation timer start!");
  
  err_code = app_timer_start(m_pid_timer_id, APP_TIMER_TICKS(PID_CONTROLLER_INTERVAL), NULL);
  APP_ERROR_CHECK(err_code);

  // Check if FE-C over BLE is actually enabled
  if (!ble_fec_active) return;

  err_code = app_timer_create(&m_fec_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                //pid_handler);
                                fec_timer_handler);
  
  // Populate scheduler;

  // Full set;
  bool toggle1 = true;
  bool toggle2 = true;

  for (uint8_t offset=0; offset<132; offset+=66)
  {
      uint8_t idx = 0;
      for (idx = 0; idx < 64; idx+=4)
      {
        schedule[offset+idx] = BLE_FEC_PAGE_GENERAL_FE_DATA;
        schedule[offset+idx+1] = BLE_FEC_PAGE_GENERAL_FE_DATA;
    
        if (toggle1)
        {
            schedule[offset+idx+2] = BLE_FEC_PAGE_OPEN_SLOT;
            schedule[offset+idx+3] = BLE_FEC_PAGE_GENERAL_SETTINGS;   
        }
        else
        {
             schedule[offset+idx+2] = BLE_FEC_PAGE_GENERAL_FE_METABOLIC_DATA;   
             schedule[offset+idx+3] = BLE_FEC_PAGE_OPEN_SLOT;
        }

        toggle1 = !toggle1;
      }

      if (toggle2)
      {
        schedule[offset+idx] = BLE_FEC_PAGE_COMMON_MANUFACTURER_IDENT;
        schedule[offset+idx+1] = BLE_FEC_PAGE_COMMON_MANUFACTURER_IDENT;
      }
      else
      {
        schedule[offset+idx] = BLE_FEC_PAGE_COMMON_PRODUCT_INFORMATION;
        schedule[offset+idx+1] = BLE_FEC_PAGE_COMMON_PRODUCT_INFORMATION;
      }
      toggle2 = !toggle2;
  }
  schedule_idx = 0;

  // Fire every PID_CONTROLLER_INTERVAL ms
  NRF_LOG_DEBUG("FEC timer start!");

  err_code = app_timer_start(m_fec_timer_id, APP_TIMER_TICKS(FEC_CONTROLLER_INTERVAL), NULL);
  APP_ERROR_CHECK(err_code);                                
}
 

void updateModelFromCSCSensorData(const ble_gattc_evt_hvx_t *p_notif)
{
    uint32_t         index = 0;
    bool isCrankRevDataPresent = false;
    bool isWheelRevDataPresent = false;
   
    isCrankRevDataPresent  = p_notif->data[index] >> BLE_CSCS_CRANK_REV_DATA_PRESENT      & 0x01;
    isWheelRevDataPresent  = p_notif->data[index] >> BLE_CSCS_WHEEL_REV_DATA_PRESENT      & 0x01;
    
    index++;
    
    if (isWheelRevDataPresent)
    {
        external_wheel_sensor_data_present = true;
        NRF_LOG_WARNING("We have external wheel sensor data! This is not expected!");

        uint32_t _cumulative_wheel_revs = uint32_decode(&p_notif->data[index]);       
        index += sizeof(uint32_t);

        uint16_t _last_wheel_event_time = uint16_decode(&p_notif->data[index]);
        index += sizeof(uint16_t);

#if HONOR_ZERO_DELTA
        if (_cumulative_crank_revs != calculation_helper.cumulative_crank_revs)
        {
#endif
            if (calculation_helper.prev_cumulative_wheel_revs == 0)
            {
                calculation_helper.prev_cumulative_wheel_revs = _cumulative_wheel_revs;
            }
            else
            {
                calculation_helper.prev_cumulative_wheel_revs = calculation_helper.cumulative_wheel_revs;
            }
        
            calculation_helper.cumulative_wheel_revs = _cumulative_wheel_revs;
#if HONOR_ZERO_DELTA
        }

        if (_last_crank_event_time != calculation_helper.last_crank_event_time)
        {
#endif     
            if (calculation_helper.prev_last_wheel_event_time == 0)
            {
                calculation_helper.prev_last_wheel_event_time = _last_wheel_event_time;
            }
            else
            {
                calculation_helper.prev_last_wheel_event_time = calculation_helper.last_wheel_event_time;
            }

            calculation_helper.last_wheel_event_time = _last_wheel_event_time;                        
#if HONOR_ZERO_DELTA
        }    
#endif    
    }

    if (isCrankRevDataPresent)
    {
      uint16_t _cumulative_crank_revs = uint16_decode(&p_notif->data[index]);                
      index += sizeof(uint16_t);

      // Last crank event time
      uint16_t _last_crank_event_time = uint16_decode(&p_notif->data[index]);
      index += sizeof(uint16_t);

#if HONOR_ZERO_DELTA
      if (_cumulative_crank_revs != calculation_helper.cumulative_crank_revs)
      {
#endif
        if (calculation_helper.prev_cumulative_crank_revs == 0) 
        {
            // Ensure the very first measurement is not unrealistic high
            calculation_helper.prev_cumulative_crank_revs = _cumulative_crank_revs;
            NRF_LOG_INFO("prev_cumulative_crank_revs=0 - Should never happen after start!");
        }
        else
        {
            calculation_helper.prev_cumulative_crank_revs = calculation_helper.cumulative_crank_revs;                
        }           
        calculation_helper.cumulative_crank_revs = _cumulative_crank_revs;
#if HONOR_ZERO_DELTA
      }

      if (_last_crank_event_time != calculation_helper.last_crank_event_time)          
      {
#endif      
        if (calculation_helper.prev_last_crank_event_time == 0)            
        {
            // Ensure the very first measurement is not unrealistic high                
            calculation_helper.prev_last_crank_event_time = _last_crank_event_time;
            NRF_LOG_INFO("prev_last_crank_event_time=0 - Very unlikely to happen!");
        }
        else
        {
            calculation_helper.prev_last_crank_event_time = calculation_helper.last_crank_event_time;                
        }
        
        calculation_helper.last_crank_event_time = _last_crank_event_time;
        
        // Init on first measurement value
        if (calculation_helper.last_wheel_event_time == 0)
        {
            calculation_helper.last_wheel_event_time = _last_crank_event_time;
        }
#if HONOR_ZERO_DELTA
      }
#endif           
    }
    // calculation_handler(NULL);
}
static void debugPrintIWheelRevRelations()
{
      char s_upper_limit_i_wheel_revs_based_on_cadence[16];
      char s_i_wheel_revs[16];
      char s_percentage_reached[16];
      char s_percentage_int_reached[16];

      sprintf(s_upper_limit_i_wheel_revs_based_on_cadence,"%f", calculation_helper.upper_limit_i_wheel_revs_based_on_cadence);        
      sprintf(s_i_wheel_revs,"%f", calculation_helper.i_wheel_revs);    
  
      if (calculation_helper.upper_limit_i_wheel_revs_based_on_cadence > 0)
      {
        double percentage_reached = calculation_helper.i_wheel_revs * 100.0 / calculation_helper.upper_limit_i_wheel_revs_based_on_cadence;
        sprintf(s_percentage_reached, "%f", percentage_reached);        
      }
      else
      {
        sprintf(s_percentage_reached, "0.0");
        
      }
  
      NRF_LOG_INFO("Delta crank rev = %u, delta_crank_event_time = %u, Target instantaneous wheel revs = %s, Current wheel revs = %s, Percentage reached = %s", 
                        calculation_helper.delta_cum_crank_revs, 
                        calculation_helper.delta_crank_event_times, 
                        s_upper_limit_i_wheel_revs_based_on_cadence, 
                        s_i_wheel_revs, 
                        s_percentage_reached);
      NRF_LOG_FLUSH();
}

static void updatePIDIntegrator(double i)
{
    if (calculation_helper.fraction > 1 && calculation_helper.fraction < 1023) calculation_helper.integrator+=i;        
    if (calculation_helper.integrator < -1023)calculation_helper.integrator = -1023;
    if (calculation_helper.integrator > 1023) calculation_helper.integrator = 1023;
}

static void debugPrintCadenceReadings()
{
    char s_i_cadence[16], s_avg_cadence[16];
    sprintf(s_i_cadence,"%f", calculation_helper.i_cadence);        
    sprintf(s_avg_cadence,"%f", calculation_helper.avg_cadence);        
    NRF_LOG_INFO("Instantaneous cadence: %s, Average cadence: %s",s_i_cadence, s_avg_cadence);        
    NRF_LOG_FLUSH();
}

static void debugPrintAchievedWheelRevsDuringCrankTime()
{
    char s_achieved_wheel_revs_during_crank_time[32];
    sprintf(s_achieved_wheel_revs_during_crank_time,"%f", calculation_helper.achieved_wheel_revs_during_crank_time); 
    NRF_LOG_INFO("achieved_wheel_revs_during_crank_time = %s", s_achieved_wheel_revs_during_crank_time);
    NRF_LOG_FLUSH();
}

static void debugPrintTimeSpanPerWheelRev()
{
    char s_timespan_per_wheel_rev[32];
    char s_achieved_wheel_revs_during_crank_time[32];
    char s_delta_achieved_whole_cumulative_wheel_revs[32];
    sprintf(s_timespan_per_wheel_rev, "%f", calculation_helper.timespan_per_wheel_rev);
    sprintf(s_achieved_wheel_revs_during_crank_time, "%f", calculation_helper.achieved_wheel_revs_during_crank_time);    
    NRF_LOG_INFO("timespan_per_wheel_rev = %s, achieved_wheel_revs_during_crank_time = %s, delta_achieved_whole_cumulative_wheel_revs = %u", s_timespan_per_wheel_rev, s_achieved_wheel_revs_during_crank_time,calculation_helper.delta_achieved_whole_cumulative_wheel_revs);    
    NRF_LOG_FLUSH();
}

static void debugPrintSpeed()
{
    char buf[32];
    sprintf(buf, "m/s: %.2f, km/h: %.2f", calculation_helper.i_speed, calculation_helper.avg_speed);
    NRF_LOG_INFO("%s",buf);
    NRF_LOG_FLUSH();
}

static void debugPrintPIDValues(double d1, double d2, double d3, double i, double p, double ki, int16_t integrator, double output1, double output)
{
    char buf[256];
    sprintf(buf, "d1: %f, d2: %f, d3(*): %f, i: %f, p: %f, ki: %f, integrator: %d, output1: %f: output(*): %f", d1, d2, d3, i, p, ki, integrator, output1, output);
    NRF_LOG_INFO("%s", buf);
    NRF_LOG_FLUSH();
}

static void debugPrintPower()
{
    NRF_LOG_INFO("Inst. power: %u, Avg. power: %d", calculation_helper.i_power, calculation_helper.avg_power);
    NRF_LOG_FLUSH();
}
/**@brief Timeout handler for the repeated timer.
 */

static void debugPrintDeltaWheelDetail()
{
    char s_delta_cum_wheel_revs[16];
    sprintf(s_delta_cum_wheel_revs, "%f", calculation_helper.delta_cum_wheel_revs);
    NRF_LOG_INFO("delta_cum_wheel_revs: %s, delta_wheel_event_times: %u", s_delta_cum_wheel_revs, calculation_helper.delta_wheel_event_times);
    NRF_LOG_FLUSH();
}

static void debugPrintCumWheel()
{
    char s_cumulative_wheel_revs[16];
    char s_prev_cumulative_wheel_revs[16];
    sprintf(s_cumulative_wheel_revs, "%f", calculation_helper.cumulative_wheel_revs);
    sprintf(s_prev_cumulative_wheel_revs, "%f", calculation_helper.prev_cumulative_wheel_revs);    
    NRF_LOG_INFO("delta_achieved_whole_cumulative_wheel_revs: %d, cumulative_wheel_revs: %s, prev_cumulative_wheel_revs: %s", calculation_helper.delta_achieved_whole_cumulative_wheel_revs, s_cumulative_wheel_revs, s_prev_cumulative_wheel_revs);
    NRF_LOG_FLUSH();
}
static void debugPrintTimeCompare()
{    
    NRF_LOG_INFO("last_crank_event_time: %u, last_wheel_event_time: %u", calculation_helper.last_crank_event_time, calculation_helper.last_wheel_event_time);
    NRF_LOG_FLUSH();
}

ble_fec_page_handler_t getFECPageHandler()
{
    return calculation_helper.fec_data.fec_page_handler;
}

ble_fec_t *getFECHandle()
{
    return calculation_helper.fec_data.fec_handle;
}

uint8_t getFECPage()
{
    return calculation_helper.fec_data.page;
}

static void fec_timer_handler(void * p_context)
{
static bool update_toggle = true;
//ble_fec_evt_t ble_fec_evt;

    calculation_helper.fec_data.page = schedule[(++schedule_idx % 128)];
    if (update_toggle)
    {   
        calculation_helper.fec_data.event_count++;
        calculation_helper.fec_data.accumulated_power += calculation_helper.i_power;        
        calculation_helper.fec_data.elapsed_time++;
        calculation_helper.fec_data.distance_traveled += (uint8_t) (0.5 * calculation_helper.i_speed);               
    }        

    update_toggle = !update_toggle;
    if (!doTriggerFECEvent)
    {
        doTriggerFECEvent = true;
    }

    /*
    ble_fec_evt.page = schedule[(++schedule_idx % 128)];

    // Do this every 500 ms
    if (update_toggle)
    {   
        calculation_helper.fec_data.event_count++;
        calculation_helper.fec_data.accumulated_power += calculation_helper.i_power;        
        calculation_helper.fec_data.elapsed_time++;
        calculation_helper.fec_data.distance_traveled += (uint8_t) (0.5 * calculation_helper.i_speed);               
    }        

    ble_fec_evt.type = BLE_FEC_REQUEST_TYPE_BROADCAST;

    if (calculation_helper.fec_data.fec_page_handler != NULL)
    {            
            (calculation_helper.fec_data.fec_page_handler)(calculation_helper.fec_data.fec_handle, &ble_fec_evt);            
            update_toggle = !update_toggle;
    }
    */
}

void setGrade(uint16_t grade)
{
    calculation_helper.fec_data.grade = grade;
}

uint16_t getGrade()
{
    return calculation_helper.fec_data.grade;
}

void setCoefficientRollingResistance(uint8_t crr)
{
    calculation_helper.fec_data.crr = crr;
}

uint8_t getCoefficientRollingResistance()
{
    return calculation_helper.fec_data.crr;
}

void setWindResistanceCoefficient(uint8_t wcr)
{
    calculation_helper.fec_data.wcr = wcr;
}

uint8_t getWindResistanceCoefficient()
{
    return calculation_helper.fec_data.wcr;
}

void setWindSpeed(int8_t windspeed)
{
    calculation_helper.fec_data.windspeed = windspeed;
}

int8_t getWindSpeed()
{
    return calculation_helper.fec_data.windspeed;
}

void setDraftingFactor(uint8_t drafting_factor)
{
    calculation_helper.fec_data.drafting_factor = drafting_factor;
}

uint8_t getDraftingFactor()
{
    return calculation_helper.fec_data.drafting_factor;
}

static void setOledGrade()
{
    char grade_s[16];
    float grade_f = ((double) getGrade() * 0.01) - 200.0;
    sprintf(grade_s, "%.2f", grade_f);
    oled_data.grade = grade_f;
}
static void performResistanceCalculation()
{
#if BENCHMARK
    uint32_t execution_start, execution_end;
    execution_start = app_timer_cnt_get();
#endif               
    char resistance_s[128];    
    
    // Wind Resistance [N] = (0.5 Wind Resistance Coefficient * (Relative Speed / 3.6)^2) x Drafting Factor (ANT-FEC, p57)
    double wind_resistance_coefficient = ((double) getWindResistanceCoefficient()) * 0.01; // cw (default 0.51 kg/m (INDOOR_BIKE_SIMULATION_PARAMETERS.cw is 51.0)        
    double rolling_resistance_coefficient = ((double) getCoefficientRollingResistance()) * 0.00005; // crr (default 0.41 ((INDOOR_BIKE_SIMULATION_PARAMETERS.cw is 41.0)

    // wind_speed in FTMS is delivered in m/s  - not km/h like in FE-C!
    double relative_speed_ms = getInstantaneousSpeed() + kmh2ms((double) getWindSpeed());
    
    double wind_resistance = 0.5 * (wind_resistance_coefficient) * pow(relative_speed_ms,2) * ((double) getDraftingFactor()) * 0.01;
    
    // Gravitational Resistance [N] = (Equipment Mass + User Mass) * Grade/100 * 9.81
    
    // uint8_t total_weight = EQUIPMENT_MASS+USER_MASS;
    uint8_t total_weight = getBikeWeight() + getUserWeight(); 
    double gravitational_resistance = total_weight * ((((double) getGrade()) * 0.01) - 200.0) * 0.01 * 9.81; // TODO Check this: Always 0 on plain track? 
    
    // Set oled_grade
    setOledGrade();
    
    // Rolling Resistance [N] = (Bicycle Mass + Cyclist Mass) x Coefficient of Rolling Resistance x 9.8 
    double rolling_resistance = total_weight * rolling_resistance_coefficient * 9.81;

    // Total resistance [N] = Gravitational Resistance + Rolling Resistance + Wind Resistance
    int16_t imposed_resistance = (int16_t) round(gravitational_resistance + rolling_resistance + wind_resistance);
    //debugPrintSimulationResistance(imposed_resistance, gravitational_resistance, rolling_resistance, wind_resistance);

    // Now we have the resistance that is imposed on us. Now try to find the resistance level of the trainer that matches best the resistance
    sprintf(resistance_s, "imposed_resistance: %d (Grav: %.3f + Roll: %.3f + Wind: %.3f)", imposed_resistance, gravitational_resistance, rolling_resistance, wind_resistance);     
    //NRF_LOG_INFO("%s",resistance_s);

    oled_data.raw_resistance = imposed_resistance;
    oled_data.wind_resistance = (int16_t) round(wind_resistance);
    oled_data.rolling_resistance = (int16_t) round(rolling_resistance);
    oled_data.gravitational_resistance = (int16_t) round(gravitational_resistance);

    double least_error = 100000.0;
    uint8_t resistance_level_candidate = 0xff; 
    
    // Calc all power values for current cadence
    // This really looks superfluous now
    double avg_cadence = getAverageCadence(); //kmh2rpm(getAverageSpeed());
    
    //sprintf(buf, "Sanity check: CALC_AVG_CAD: %.2f, TRUE_AVG_CAD: %.2f" , calced_avg_cadence, getAverageCadence());
    //NRF_LOG_INFO("%s", buf);


    double resulting_power = 0.0;
    for (uint8_t pot_resistance_level=1; pot_resistance_level<=NUM_RESISTANCE_LEVELS; pot_resistance_level++)
    {        
        resulting_power = calculatePower(avg_cadence, pot_resistance_level);
        // NRF_LOG_INFO("Resulting power at level %d: %d", pot_resistance_level, (uint16_t) resulting_power);
        double resulting_resistance = resulting_power / getInstantaneousSpeed();
       // NRF_LOG_INFO("Resulting resistance at level %d: %d", pot_resistance_level, (uint16_t) resulting_resistance);
        double error = fabs(resulting_resistance - imposed_resistance);
        if (error < least_error)
        {
            resistance_level_candidate = pot_resistance_level;
            least_error = error;
        }

        // Early break
        // There is only one minimum - once our error increases again we can stop
        
        /*if (error > least_error)
        {
            char e_break[128];
            sprintf(e_break, "Early exit: error (%.2f) > least error (%.2f)", error, least_error);
            //NRF_LOG_INFO("%s", e_break); 
            break;
        }
        */
    }
    
    if (resistance_level_candidate != 0xff)
    {
        if (resistance_level_candidate != resistance_level)
        {
            NRF_LOG_INFO("%s",resistance_s);
            NRF_LOG_INFO("Best resistance_level = %d. Current resistance_level = %d, Current gear delta: %d", resistance_level_candidate, target_resistance_level, gear_offset);
            // Change the resistance setting
            if (resistance_level_candidate == 0)
            {
                NRF_LOG_ERROR("Impossible resistance_level_candidate");
            }

            // TODO/BUG: If we have resistance_level_candidate = 0
            // We will assign it in the next line and the handler will assert in main()!
            target_resistance_level = resistance_level_candidate;
        }
    }
#if BENCHMARK
    execution_end = app_timer_cnt_get();
    uint32_t timer_ticks = app_timer_cnt_diff_compute(execution_end,execution_start);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.3f", (double) timer_ticks / 32.0);
    NRF_LOG_INFO("Execution time resistance (ms): %s", buf);
#endif
}
      
void setUserWeight(uint8_t kg)
{
    NRF_LOG_INFO("Set new user weight: %u", kg);
    calculation_helper.userweight = kg;
}

void setBikeWeight(uint8_t kg)
{
    NRF_LOG_INFO("Set new bike weight: %u", kg);
    calculation_helper.bikeweight = kg;
}

uint8_t getUserWeight()
{
    return calculation_helper.userweight;
}

uint8_t getBikeWeight()
{
    return calculation_helper.bikeweight;
}


uint8_t getFECElapsedTime()
{
    return calculation_helper.fec_data.elapsed_time;
}

uint8_t getFECDistanceTraveled()
{
    return calculation_helper.fec_data.distance_traveled;
}

uint8_t getFECEventcount()
{
    return calculation_helper.fec_data.event_count;
}

uint16_t getFECAccumulatedPower()
{
    return calculation_helper.fec_data.accumulated_power;
}


static void calculation_handler(void * p_context)
{
    if (!doCalcModel)
    {
        doCalcModel = true;
    }
}

void performModelCalculations()
{

    char s_i_kmh[6];
    char s_d[16], s_p[16], s_output[32], s_ideal_cumulative_wheel_revs[32];
    //NRF_LOG_INFO("************************************************************");
    // First we calculate the instantaneous and avg cadence all at once

#if BENCHMARK
    uint32_t execution_start, execution_end;
    execution_start = app_timer_cnt_get(); 
#endif // BENCHMARK

    double gear_ratio = getGearRatio();
    updateCadenceReadings();
                                
        // debugPrintCadenceReadings();
                        
        // Calculate and debug log the upper limit of how many wheel revolutions we can achieve (based on the current instantaneous cadence)
        // vs. how many we actually have 
        calculation_helper.upper_limit_i_wheel_revs_based_on_cadence = calculation_helper.i_cadence * gear_ratio; // during the same time slice 'delta_crank_event_time';                   
        
        //debugPrintIWheelRevRelations();
                
        // calculation_helper.fraction = 1023.0; 
        // Explanation:
        // fraction ==> controlvalue;
        // What is fraction ? 0 - 100% of ideal cumulative_wheel_revs!                              
    
        // We assume we can ideally achieve delta_cum_crank_revs*gear + what we had before
        // TODO: Basically this is the same as the i* (upper_limit_i_wheel_revs_based_on_cadence) variants just on the actual cum number? 
        // calculation_helper.ideal_cumulative_wheel_revs = calculation_helper.delta_cum_crank_revs * getGearRatio() + calculation_helper.prev_cumulative_wheel_revs;
#
                
    // TODO: Is it the goal to get fraction to 100% (1023) to achieve the maximum?
    // Attention! It's upper_limit_i_wheel_revs_based_on_cadence! Not ideal_cumulative_wheel_revs (which is the overall sum!)
    calculation_helper.achieved_wheel_revs_during_crank_time = gear_ratio * calculation_helper.delta_cum_crank_revs; //calculation_helper.upper_limit_i_wheel_revs_based_on_cadence;
    
    

    // First get the integer part of the total achieved revolutions during that time
    uint16_t achieved_whole_wheel_revs_during_crank_time = floor(calculation_helper.achieved_wheel_revs_during_crank_time);

    // debugPrintAchievedWheelRevsDuringCrankTime();
     
    // No we have everything and can update the wheel readings        
    calculation_helper.prev_cumulative_wheel_revs = calculation_helper.cumulative_wheel_revs;
    calculation_helper.prev_last_wheel_event_time = calculation_helper.last_wheel_event_time;

    // Set the new wheel readings

    // We add here the floats not the ints!
    calculation_helper.cumulative_wheel_revs = calculation_helper.prev_cumulative_wheel_revs + calculation_helper.achieved_wheel_revs_during_crank_time;        
    
    // First get the integer part of the total achieved revolutions
    // 2.9 (int=2) + 2.6 = 5.5 (int=5) -> 5.2 = 3 whole cumulative wheel revs!
    calculation_helper.delta_achieved_whole_cumulative_wheel_revs = floor(calculation_helper.cumulative_wheel_revs) - floor(calculation_helper.prev_cumulative_wheel_revs);
    //debugPrintCumWheel();

    // Not sure but give it a try: not prev_last_wheel event_time but prev_last_crank_event_time!
    // Why? In previous times wer had only int cum_events. Which was neglecting the fraction part: 2.6->5.2->7.8 instead only 2,4,6,8, ...
    // Lets further assume crank events come every second. 
    // For the first event we calculated correctly the time of "2" events with respect to the crank event time.
    // So after 1s for the crank event, we had actually 2.6 wheel events, but the 2 wheel events were back at 0.7sec or something.
    // If we - in the next iteration - start at that 0.7 sec and add again the same 0.7, 0.7, 0.7 - we will eventually fall back more and more with
    // respect to the crank time stamps!
    // Crank:   1,   2,   3,   4, 5
    // Wheel: 0.7, 1.4, 2.1, 2.8, ...
    // Instead: We now that we had 2.6 at 1 sec, and we now we would have 5.2 after 2 sec.
    
    // Calculate the time for these (int) wheel revolutions, based on the time slice of the crank event
    if (calculation_helper.achieved_wheel_revs_during_crank_time > 0)
    {
        calculation_helper.timespan_per_wheel_rev = (double) calculation_helper.delta_crank_event_times / calculation_helper.achieved_wheel_revs_during_crank_time;  // + prev_last_crank_time-prev_last_wheel_event_time?    
    }
    else
    {
        calculation_helper.timespan_per_wheel_rev = 0;
    }
    //debugPrintTimeSpanPerWheelRev();
    
    calculation_helper.last_wheel_event_time = calculation_helper.prev_last_wheel_event_time + (uint16_t) (calculation_helper.delta_achieved_whole_cumulative_wheel_revs * calculation_helper.timespan_per_wheel_rev);
    //debugPrintTimeCompare();
       
    calculation_helper.delta_cum_wheel_revs    = (calculation_helper.cumulative_wheel_revs - calculation_helper.prev_cumulative_wheel_revs);
    calculation_helper.delta_wheel_event_times = calculation_helper.last_wheel_event_time - calculation_helper.prev_last_wheel_event_time;
    
    //debugPrintDeltaWheelDetail();

    ////
    //// wenn i_wheel_rev == delta_cum_wheel_rev ==> everything is correct

    //// Idea to get i_wheel_rev was 
    //// calculation_helper.i_wheel_revs = ((double) calculation_helper.delta_achieved_whole_cumulative_wheel_revs * 1024.0) / (double) calculation_helper.delta_wheel_event_times --> Wrong
    //// calculation_helper.i_cadence = (double) (calculation_helper.delta_cum_crank_revs) * 1024.0 / (double)(calculation_helper.delta_crank_event_times);  
    //// delta_cum_wheel_rev = 3.431489, 
    //// delta_achieved_whole_cumulative_wheel_revs = 3
    //// delta_wheel_event_times = 657
    
    // Instant km/h?
    if (calculation_helper.delta_wheel_event_times > 0)
    {          
      updateInstantaneousWheelRevs(); 
      updateInstantaneousSpeed();             
      updateAverageSpeed(); // TODO: This here crashes
      // debugPrintSpeed();  
    }
    else
    {
      //NRF_LOG_DEBUG("m/s: 0.00");
    } 
    
    // Power
    updateInstantaneousPower();

    if (calculation_helper.fec_data.mode == FEC_SIMULATION)
    {
        performResistanceCalculation();
    }
    oled_data.cadence   = calculation_helper.avg_cadence;
    oled_data.kmh   = calculation_helper.avg_speed;
    oled_data.power = calculation_helper.i_power;  
    // debugPrintPower();
#if BENCHMARK
    execution_end = app_timer_cnt_get();
    uint32_t timer_ticks = app_timer_cnt_diff_compute(execution_end,execution_start);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.3f", (double) timer_ticks / 32.0);
    NRF_LOG_INFO("Execution time calculations (ms): %s", buf);
#endif

}

/*
static void calculation_handler(void * p_context)
{    
    char s_i_kmh[6];
    char s_d[16], s_p[16], s_output[32], s_ideal_cumulative_wheel_revs[32];

    // First we calculate the instantaneous and avg cadence all at once
    updateCadenceReadings();
#if HANDLE_FREEROLL    
    if (calculation_helper.i_cadence > 0) // If we have some kind of pedaling    
    {
        calculation_helper.freeroll_duration = 0;
        calculation_helper.freeroll_start = 0.0;
    
#endif                                        
        debugPrintCadenceReadings();
                        
        // Calculate and debug log the upper limit of how many wheel revolutions we can achieve (based on the current instantaneous cadence)
        // vs. how many we actually have 
        calculation_helper.upper_limit_i_wheel_revs_based_on_cadence = calculation_helper.i_cadence * getGearRatio(); // during the same time slice 'delta_crank_event_time';                   
        
        debugPrintIWheelRevRelations();
        
        // PID Algorithm, https://www.mikrocontroller.net/attachment/552/PID.doc 
           
        double kp = 90;  // Choose good values??
        double kd = 50;
        double ki = 50;

        // We want to minimize this delta, delta can be positive or negative
        // i_wheel_revs is only updated at the end
        // TODO: Check if we need to initialize i_wheel_revs     
        double delta = calculation_helper.upper_limit_i_wheel_revs_based_on_cadence - calculation_helper.i_wheel_revs;
    
        double p = delta * kp;
        if (p > MAX_PID_OUTPUT_RANGE) p = MAX_PID_OUTPUT_RANGE;      

        // Debug
        double d1, d2, d3 = 0;
        double d = calculation_helper.tweak_wheel_cadence - calculation_helper.i_wheel_revs;
        d1 = d;
        d = d / PID_CONTROLLER_INTERVAL;
        d = d * kd;
        d2 = d;
        if (d >  MAX_PID_OUTPUT_RANGE) d = MAX_PID_OUTPUT_RANGE;
        if (d < -MAX_PID_OUTPUT_RANGE) d =-MAX_PID_OUTPUT_RANGE;
        d3 = d;

        double i = delta * 1.0 / PID_CONTROLLER_INTERVAL;
      
        i = i * ki;
      
        updatePIDIntegrator(i);
              
        if (ki < 0.01) calculation_helper.integrator = 0;
        double output = d + p; // + calculation_helper.integrator;

        double output1 = output;
        
        // Limit the output
        if (output >  MAX_PID_OUTPUT_RANGE) output = MAX_PID_OUTPUT_RANGE;
        if (output <= 0) output = MAX_PID_OUTPUT_RANGE - output/MAX_PID_OUTPUT_RANGE;
        debugPrintPIDValues(d1, d2, d3, i, p, ki, calculation_helper.integrator, output1, output);
        calculation_helper.fraction = output;
        
        calculation_helper.fraction = 1023.0; 
        // Explanation:
        // fraction ==> controlvalue;
        // What is fraction ? 0 - 100% of ideal cumulative_wheel_revs!                      
        calculation_helper.tweak_wheel_cadence = calculation_helper.i_wheel_revs;
    
        // We assume we can ideally achieve delta_cum_crank_revs*gear + what we had before
        // TODO: Basically this is the same as the i* (upper_limit_i_wheel_revs_based_on_cadence) variants just on the actual cum number? 
        calculation_helper.ideal_cumulative_wheel_revs = calculation_helper.delta_cum_crank_revs * getGearRatio() + calculation_helper.prev_cumulative_wheel_revs;
#if HANDLE_FREEROLL
    }
    else
    {
        NRF_LOG_INFO("No crank activity");
        // Function is v(t) = c * exp(-k * t) with v(t0) at t0=0 = km/h
        // 45*exp(-0.04*x) for v(0) = 45 and x = 0
        // Can we assume the same formula is aquivalent for the avg_crank?
        // TODO: Base this on avg_speed not crank

        if (calculation_helper.freeroll_duration == 0)
        {
            calculation_helper.freeroll_start = calculation_helper.ideal_cumulative_wheel_revs;
        }

        calculation_helper.freeroll_duration++;

        // Update value
        // See above, our ideal_cumulative_wheel_revs is based on an exponential declining function not on cadence
        calculation_helper.ideal_cumulative_wheel_revs = calculation_helper.freeroll_start * exp(-0.04 * calculation_helper.freeroll_duration);
    }
#endif
                
    // TODO: Is it the goal to get fraction to 100% (1023) to achieve the maximum?
    // Attention! It's upper_limit_i_wheel_revs_based_on_cadence! Not ideal_cumulative_wheel_revs (which is the overall sum!)
    calculation_helper.achieved_cumulative_wheel_revs = calculation_helper.fraction * calculation_helper.upper_limit_i_wheel_revs_based_on_cadence / MAX_PID_OUTPUT_RANGE;        
    debugPrintAchievedCumulativeWheelRevs();
    
    // Calculate the time for these (int) wheel revolutions, based on the time slice of the crank event
    if (calculation_helper.achieved_cumulative_wheel_revs > 0)
    {
        calculation_helper.timespan_per_wheel_rev = calculation_helper.delta_crank_event_times / calculation_helper.achieved_cumulative_wheel_revs;        
    }
    else
    {
        calculation_helper.timespan_per_wheel_rev = 0;
    }
    debugPrintTimeSpanPerWheelRev();
    
    // No we have everything and can update the wheel readings        
    calculation_helper.prev_cumulative_wheel_revs = calculation_helper.cumulative_wheel_revs;
    calculation_helper.prev_last_wheel_event_time = calculation_helper.last_wheel_event_time;

    // Set the new wheel readings
    uint16_t achieved_whole_cumulative_wheel_revs = floor(calculation_helper.achieved_cumulative_wheel_revs);
    calculation_helper.cumulative_wheel_revs = calculation_helper.prev_cumulative_wheel_revs + achieved_whole_cumulative_wheel_revs;        
    calculation_helper.last_wheel_event_time = calculation_helper.prev_last_wheel_event_time + (uint16_t) (achieved_whole_cumulative_wheel_revs * calculation_helper.timespan_per_wheel_rev);
       
    calculation_helper.delta_cum_wheel_revs    = calculation_helper.cumulative_wheel_revs - calculation_helper.prev_cumulative_wheel_revs;    
    calculation_helper.delta_wheel_event_times = calculation_helper.last_wheel_event_time - calculation_helper.prev_last_wheel_event_time;
    //assert(calculation_helper.delta_cum_wheel_revs == achieved_whole_cumulative_wheel_revs);

    // Instant km/h?
    if (calculation_helper.delta_wheel_event_times > 0)
    {          
      updateInstantaneousSpeed();       
      updateInstantaneousWheelRevs(); 
      updateAverageSpeed(); // TODO: This here crashes
      debugPrintSpeed();             
    }
    else
    {
      NRF_LOG_DEBUG("m/s: 0.00");
    }        

    // Power
    updateInstantaneousPower();

    oled_data.cadence   = calculation_helper.avg_cadence;
    oled_data.kmh   = calculation_helper.avg_speed;
    oled_data.power = calculation_helper.i_power;   
}
*/

void setFECMode(fec_mode_t fec_mode)
{
    calculation_helper.fec_data.mode = fec_mode;
}

fec_mode_t getFECMode()
{
    return calculation_helper.fec_data.mode;
}

uint16_t getInstantaneousPower()
{
  return calculation_helper.i_power;
}

uint16_t getAveragePower()
{
  return calculation_helper.avg_power;
}

static double updateInstantaneousPower()
{
  calculation_helper.i_power = (uint16_t) calculatePower(calculation_helper.avg_cadence, calculation_helper.resistance_level);
  calculation_helper.avg_power = calculation_helper.i_power;
}

void setTrainerResistanceLevel(uint8_t trainer_resistance_level)
{
  NRF_LOG_INFO("trainer_resistance_level set");
  calculation_helper.trainer_resistance_level = trainer_resistance_level;
}
/*
void updateResistance()
{
    uint8_t final_resistance_level = 
          (int8_t) ((int8_t) calculation_helper.trainer_resistance_level + 0*gear_offset) < 1 ? 1 : (int8_t) calculation_helper.trainer_resistance_level + 0*gear_offset;
    
    if (final_resistance_level > 30)
    {
      final_resistance_level = 30;
    }

    if (resistance_level>=1 && resistance_level<=30 && resistance_level!=final_resistance_level)
    {   
        // NRF_LOG_INFO("Resistance level change needed: Current = %u, Should %u (Target: %u + Gear offset: %d)", resistance_level, final_resistance_level, target_resistance_level, gear_offset);
        if (resistance_level < final_resistance_level) 
        {
            triggerResistancePlus();
            resistance_level++;
        }
        else
        {
            triggerResistanceMinus();
            resistance_level--;
        }                
    }
    else
    {
        //NRF_LOG_INFO("Initial incline %d reached. Stopping timer", incline_level);
        // NRF_LOG_INFO("No change of incline needed (current incline = %d", incline_level);
        //app_timer_stop(m_setup_hometrainer_timer_id);
    }

    oled_data.trainer_resistance = resistance_level;
    oled_data.sim_resistance = target_resistance_level;
}
*/
double calculatePower(double rpm, uint8_t resistance_level)
{
  double watts = 0.0;
  
  switch(resistance_level)
  {
           case 1:
              watts = 62.0 + (rpm - 80.0) * 2.0;
              break;
            case 2:
              watts = 75.0 + (rpm - 80.0) * 2.0;
              break;
            case 3:
              watts = 86.0 + (rpm - 80.0) * 2.0;
              break;
            case 4:
              watts = 102.0 + (rpm - 80.0) * 2.0;
              break;
            case 5:
              watts = 116.0 + (rpm - 80.0) * 2.0;
              break;
            case 6:
              watts = 128.0 + (rpm - 80.0) * 2.0;
              break;
            case 7:
              watts = 139.0 + (rpm - 80.0) * 2.0;
              break;
            case 8:
              watts = 151.0 + (rpm - 80.0) * 3.0;
              break;
            case 9:
              watts = 168.0 + (rpm - 80.0) * 3.0;
              break;
            case 10:
              watts = 178.0 + (rpm - 80.0) * 3.0;
              break;
            case 11:
              watts = 190.0 + (rpm - 80.0) * 3.0;
              break;
            case 12:
              watts = 200.0 + (rpm - 80.0) * 3.0;
              break;
            case 13:
              watts = 210.0 + (rpm - 80.0) * 3.0;
              break;
            case 14:
              watts = 220.0 + (rpm - 80.0) * 3.0;
              break;
            case 15:
              watts = 230.0 + (rpm - 80.0) * 3.0;
              break;
            case 16:
              watts = 236.0 + (rpm - 78.0) * 4.0;
              break;
            case 17:
              watts = 240.0 + (rpm - 76.0) * 4.0;
              break;
            case 18:
              watts = 248.0 + (rpm - 75.0) * 4.0;
              break;
            case 19:
              watts = 258.0 + (rpm - 75.0) * 4.0;
              break;
            case 20:
              watts = 268.0 + (rpm - 74.0) * 4.0; // Eval + 10
              break;
            case 21:
              watts = 278.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 22:
              watts = 288.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 23:
              watts = 298.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 24:
              watts = 308.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 25:
              watts = 318.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 26:
              watts = 328.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 27:
              watts = 338.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 28:
              watts = 348.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 29:
              watts = 358.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 30:
              watts = 368.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 31:
              watts = 378.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            case 32:
              watts = 388.0 + (rpm - 75.0) * 4.0; // Guess
              break;
            default:
              NRF_LOG_INFO("Invalid resistance_level %d (1-%d)!", resistance_level, NUM_RESISTANCE_LEVELS);
              
              watts = 0.0;
              break;
        }

        if (watts < 0.0)
        {
#if LOGHERE
            NRF_LOG_INFO("Watts underflow. Correcting to 0W. Watts: %u, RPM: %d, Resistance level: %d", (uint16_t)watts, rpm, resistance_level);
#endif
            watts = 0.0;
        }

        if (watts > MAX_WATT)
        {
#if LOGHERE
          NRF_LOG_INFO("Watts overflow. Correcting to %uW. Watts: %u, RPM: %d, Resistance level: %d", MAX_WATT, (uint16_t)watts, rpm, resistance_level);
#endif
          // TODO: Add some jitter
          watts = MAX_WATT;
        }
            
      return watts;

}

void registerFECCallback(ble_fec_t *p_fec, ble_fec_page_handler_t fec_page_handler)
{
    calculation_helper.fec_data.fec_handle = p_fec;
    calculation_helper.fec_data.fec_page_handler = fec_page_handler;
    NRF_LOG_INFO("FEC callback registered");
}