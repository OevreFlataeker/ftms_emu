#include "sdk_common.h"

#define NRF_LOG_MODULE_NAME helper
#if HELPER_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       HELPER_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  HELPER_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR HELPER_CONFIG_DEBUG_COLOR
#else // HELPER_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // HELPER_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#include "nrf_log_ctrl.h"
#include "helper.h"
#include "stdio.h"
#include "assert.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "math.h"
#include "ble_ftms_ctrlpt.h"
#include "ble_ftms.h"
#include "shimano.h"
#include "oled_controller.h" 
#include "calculations.h" 

#define LOGHERE 0
#define BENCHMARK 0

#define DRAFTING_FACTOR 1.0  

APP_TIMER_DEF(m_gpio_timer);
APP_TIMER_DEF(m_gpio_clear_timer);

volatile uint8_t gpio;
volatile bool gpio_busy;
volatile bool gpio_finished;



extern oled_data_t oled_data;

char *hex2str(char *buf, uint8_t buflen, uint8_t *data, uint8_t datalen)
{
int pos = 0;
    while ((pos*3)<buflen && pos<datalen)
    {
        sprintf(buf+(pos*3), "%02x ", *(data+pos));
        pos++;
    }
    *(buf+pos*3) = '\0';
    return buf;    
}

static void clear_handler(void *context)
{
    gpio_busy = false;
    gpio = 0;
    gpio_finished = true;
}

static void gpio_handler(void *context)
{
  if (gpio == TRIGGER_RESISTANCE_UP_GPIO)
  {
    nrf_gpio_pin_clear(TRIGGER_RESISTANCE_UP_GPIO);
    incResistanceLevel();
  }
  else
  {
    nrf_gpio_pin_clear(TRIGGER_RESISTANCE_DOWN_GPIO);
    decResistanceLevel();
  }
  
  // Fire up another "wait" timer to keep the signal low for a short amount of time
  ret_code_t err_code = app_timer_create(&m_gpio_clear_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                clear_handler);
  APP_ERROR_CHECK(err_code);  
  err_code = app_timer_start(m_gpio_clear_timer, APP_TIMER_TICKS(TRIGGER_DURATION), NULL);
  APP_ERROR_CHECK(err_code);
}

void triggerResistanceChange(trigger_direction_t direction)
{
    if (gpio_busy) 
    {
      return;
    }

    gpio_busy = true;
    gpio_finished = false;

    if (direction == TRIGGER_UP)
    {
      gpio = TRIGGER_RESISTANCE_UP_GPIO;
      //NRF_LOG_INFO("Trigger resistance UP");
      oled_data.trigger_up_counter = 2;
    }
    else if (direction == TRIGGER_DOWN)
    {
      gpio = TRIGGER_RESISTANCE_DOWN_GPIO;
      //NRF_LOG_INFO("Trigger resistance DOWN");
      oled_data.trigger_down_counter = 2;
    }
     
    nrf_gpio_pin_set(gpio);
    
    ret_code_t err_code = app_timer_create(&m_gpio_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                gpio_handler);
    APP_ERROR_CHECK(err_code);  
    err_code = app_timer_start(m_gpio_timer, APP_TIMER_TICKS(TRIGGER_DURATION), NULL);
    APP_ERROR_CHECK(err_code);
}

double kmh2ms (double kmh)
{
    return (kmh/3.6);
}

double ms2kmh (double ms)
{
    return (ms*3.6);
}

double kmh2rpm (double kmh)
{
    return ((60.0 * kmh / CIRCUMFERENCE_WHEEL) / 3.6) / getGearRatio();
}

static void debugPrintSimulationResistance(int16_t imposed_resistance, 
                                           double gravitational_resistance,
                                           double rolling_resistance,
                                           double wind_resistance)
{
    char buf[128];
    sprintf(buf, "imposed_resistance: %d (Grav: %.3f + Roll: %.3f + Wind: %.3f)", imposed_resistance, gravitational_resistance, rolling_resistance, wind_resistance); 
    NRF_LOG_INFO("%s", buf);
    NRF_LOG_FLUSH();
}

void set_ftms_simulation_parameter(ble_ftms_indoor_bike_simulation_parameters_t indoor_bike_simulation_parameters)
{
#if BENCHMARK
    uint32_t execution_start, execution_end;
    execution_start = app_timer_cnt_get();
#endif               
    // 20220401 rewrite. Try to inject into calculations the appropriate values
    setWindResistanceCoefficient(indoor_bike_simulation_parameters.cw);
    setCoefficientRollingResistance(indoor_bike_simulation_parameters.crr * 2 ); // Warning FTMS = / 10000, FE-C = * 0.00005 -> FTMS: 10000/10000=1 * 0.00005 = 20000 (FEC) -> FTMS: crr *2 == FE-C value
    // Grade
    // FE-C uses an offset of 200, FTMS: value of 100 = 1.00 = 1 % -> FE-C value of 20100: 20100 * 0.01 -200%  = 1
    int16_t tmp_grad = indoor_bike_simulation_parameters.grade + 20000;
    setGrade(tmp_grad); 
    setWindSpeed(indoor_bike_simulation_parameters.wind_speed);

    if (getFECMode() != FEC_SIMULATION)
    {
        setFECMode(FEC_SIMULATION);
        oled_data.mode = SIM;
        NRF_LOG_INFO("Switching to mode FEC_SIMULATION");
    }
    // end of rewrite
    return;

    // Old style FTMS
    char resistance_s[128];
    // Wind Resistance [N] = (0.5 Wind Resistance Coefficient * (Relative Speed / 3.6)^2) x Drafting Factor (ANT-FEC, p57)
    double wind_resistance_coefficient = ((double) indoor_bike_simulation_parameters.cw) / 100.0; // cw (default 0.51 kg/m (INDOOR_BIKE_SIMULATION_PARAMETERS.cw is 51.0)        
    double rolling_resistance_coefficient = ((double) indoor_bike_simulation_parameters.crr) / 10000.0; // crr (default 0.41 ((INDOOR_BIKE_SIMULATION_PARAMETERS.cw is 41.0)

    // wind_speed in FTMS is delivered in m/s  - not km/h like in FE-C!
    double relative_speed_ms = getInstantaneousSpeed() + indoor_bike_simulation_parameters.wind_speed;
    
    double wind_resistance = 0.5 * (wind_resistance_coefficient) * pow(relative_speed_ms,2) * DRAFTING_FACTOR;
    
    // Gravitational Resistance [N] = (Equipment Mass + User Mass) * Grade/100 * 9.81
    
    // uint8_t total_weight = EQUIPMENT_MASS+USER_MASS;
    uint8_t total_weight = getBikeWeight() + getUserWeight(); 
    double gravitational_resistance = total_weight * (((double) indoor_bike_simulation_parameters.grade / 100.0) / 100.0) * 9.81;

    // Rolling Resistance [N] = (Bicycle Mass + Cyclist Mass) x Coefficient of Rolling Resistance x 9.8 
    double rolling_resistance = total_weight * rolling_resistance_coefficient * 9.81;

    // Total resistance [N] = Gravitational Resistance + Rolling Resistance + Wind Resistance
    int16_t imposed_resistance = (int16_t) round(gravitational_resistance + rolling_resistance + wind_resistance);
    //debugPrintSimulationResistance(imposed_resistance, gravitational_resistance, rolling_resistance, wind_resistance);

    sprintf(resistance_s, "imposed_resistance: %d (Grav: %.3f + Roll: %.3f + Wind: %.3f)", imposed_resistance, gravitational_resistance, rolling_resistance, wind_resistance); 
    oled_data.raw_resistance = imposed_resistance;

    // Now we have the resistance that is imposed on us. Now try to find the resistance level of the trainer that matches best the resistance
    
    double least_error = 100000.0;
    uint8_t resistance_level_candidate = 0xff; 
    
    // Calc all power values for current cadence
    // This really looks superfluois now
    double avg_cadence = getAverageCadence(); //kmh2rpm(getAverageSpeed());
    
    //sprintf(buf, "Sanity check: CALC_AVG_CAD: %.2f, TRUE_AVG_CAD: %.2f" , calced_avg_cadence, getAverageCadence());
    //NRF_LOG_INFO("%s", buf);

    for (uint8_t pot_resistance_level=1; pot_resistance_level<=NUM_RESISTANCE_LEVELS; pot_resistance_level++)
    {        
        double resulting_power = calculatePower(avg_cadence, pot_resistance_level);
        // NRF_LOG_INFO("Resulting power at level %d: %d", pot_resistance_level, (uint16_t) resulting_power);
        double resulting_resistance = resulting_power / getInstantaneousSpeed();
        // NRF_LOG_INFO("Resulting resistance at level %d: %d", pot_resistance_level, (uint16_t) resulting_resistance);
        double error = fabs(resulting_resistance - imposed_resistance);
        if (error < least_error)
        {
            resistance_level_candidate = pot_resistance_level;
            least_error = error;
        }
    }

    if (resistance_level_candidate != 0xff)
    {
        if (resistance_level_candidate != resistance_level)
        {
            // NRF_LOG_INFO("%s",resistance_s);
            NRF_LOG_INFO("Best resistance_level = %d. Current resistance_level = %d, Current gear delta: %d", resistance_level_candidate, target_resistance_level, gear_offset);
            // Change the resistance setting
            target_resistance_level = resistance_level_candidate;
        }
    }
#if BENCHMARK
    execution_end = app_timer_cnt_get();
    uint32_t timer_ticks = app_timer_cnt_diff_compute(execution_end,execution_start);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.3f", (double) timer_ticks / 32.0);
    NRF_LOG_INFO("Execution time (ms): %s", buf);
#endif
}
        

void set_target_resistance(uint8_t target_resistance)
{
    // We map resistance 1:1 to incline
    // TODO: Use the same as for sim to match
    uint8_t target_r = target_resistance / 10;
    
    if (target_r > NUM_RESISTANCE_LEVELS) 
    {
        target_r = NUM_RESISTANCE_LEVELS;
    } 
    else if (target_r <= 0)
    {
        target_r = 1;
    }
#if LOGHERE
    //NRF_LOG_INFO("ftms::set_ftms_target_resistance(), target_resistance: %d -> mapped to target_incline: %d" , target_resistance, target_incline);
#endif
    target_resistance_level = target_r;
}



void set_target_power(int16_t target_power, uint16_t target_cadence)
{    
    // Try to meet the specs as closely as possible
    // TODO: Fix: incline_level is the global var from main()
    double least_error = 100000.0;
    uint8_t resistance_level_candidate = -1;
    char buf[128];
    for (uint8_t pot_resistance_level=1; pot_resistance_level<=NUM_RESISTANCE_LEVELS; pot_resistance_level++)
    {
            //double power_outcome = calculate_power_from_avg_cadence((double)target_cadence, pot_resistance_level); // TODO Use our mode' func        
            double power_outcome = calculatePower((double) target_cadence, pot_resistance_level);
            double error = fabs(power_outcome - (double) target_power);
#if LOGHERE
            sprintf(buf, "Cadence: %d, Test: %d -> Output: %f - Target: %d -> Error: %f, Least_Error: %f", target_cadence, pot_resistance_level, power_outcome, target_power, error, least_error);

            //NRF_LOG_INFO("%s", buf);
#endif
            if (error < least_error)
            {
                resistance_level_candidate = pot_resistance_level;
                least_error = error;
            }
    }

    if (resistance_level_candidate != -1)
    {
#if LOGHERE
        NRF_LOG_INFO("Best resistance = %d. Current resistance = %d", resistance_level_candidate, resistance_level);
#endif
        if (resistance_level_candidate != resistance_level)
        {
            // Change the resistance setting
            target_resistance_level = resistance_level_candidate;
        }
    }
}

