#ifndef CALCULATIONS_H__
#define CALCULATIONS_H__

#include <stdint.h>
#include <app_timer.h>
#include <ble_gattc.h> 

#include "ble_fec.h"

#define EQUIPMENT_MASS 9
#define USER_MASS 95

extern volatile bool ble_fec_active;

typedef enum
{
    FEC_SIMULATION,
    FEC_TARGET_POWER,
} fec_mode_t;

typedef struct
{
    uint8_t event_count;       // Page 0x19
    uint16_t accumulated_power; // Page 0x19
    uint8_t elapsed_time;  // Page 0x10 accumulated time since start of workout in 0.25s
    uint8_t distance_traveled; // Page 0x10 distance traveled in meters

    ble_fec_t * fec_handle;
    ble_fec_page_handler_t fec_page_handler;

    uint8_t drafting_factor;
    int8_t windspeed;
    uint8_t wcr;
    uint8_t crr;
    uint16_t grade;
    fec_mode_t mode;

    uint8_t page;
} fec_data_t;


typedef struct
{
    uint8_t  resistance_level;
    uint8_t trainer_resistance_level; // TODO: Not yet implemented
    uint16_t last_crank_event_time;  // used in updateModel, updateCadenceReadings, calcAverageSpeed
    uint16_t last_wheel_event_time;
    uint16_t prev_last_crank_event_time;  // used in updateModel, updateCadenceReadings, calcAverageSpeed
    uint16_t prev_last_wheel_event_time;

    uint16_t cumulative_crank_revs; 
    double cumulative_wheel_revs;
    uint16_t prev_cumulative_crank_revs;        
    double prev_cumulative_wheel_revs;

    
    uint16_t timespan_until_last_full_wheel_event;
    uint16_t timespan_for_wheel_events;
    
    uint16_t delta_achieved_whole_cumulative_wheel_revs; // int delta between the floored current and previous
    uint16_t delta_last_wheel_time;
    uint16_t delta_crank_event_times;  // only write in updateCadenceReadings()
    uint16_t delta_wheel_event_times;
    uint16_t delta_cum_crank_revs;     // only write in updateCadenceReadings()
    double delta_cum_wheel_revs;
  
    uint16_t   i_power;
    uint16_t   avg_power;

    double crank_events_per_sec;
    double current_time_interval_one_crank_event;
    double precise_cum_wheel_revs_at_last_crank_event;
    double upper_limit_i_wheel_revs_based_on_cadence;
    double duration_per_wheel_event;
   
    double i_cadence;                   // only write in updateCadenceReadings()
    double i_wheel_revs;
    double avg_cadence;                 // only write in updateCadenceReadings()   
    
    double i_speed;        // Instantaneous speed in m/s
    double avg_speed;          // Average speed in km/h
    
    
    
    double ideal_cumulative_wheel_revs;
    //double achieved_cumulative_wheel_revs;
    double achieved_wheel_revs_during_crank_time;
    double tweak_wheel_cadence;
    double timespan_per_wheel_rev;
    
    double fraction;
    int16_t integrator;
#if HANDLE_FREEROLL
    uint16_t freeroll_duration;
    double freeroll_start;
#endif
    fec_data_t  fec_data;
    uint8_t userweight;
    uint8_t bikeweight;

} calculation_helper_t;

// Define the circumference in cm of a DIN bicycle wheel
// https://www.bikecalc.com/wheel_size_math
// Rim: 27inch, Tire: 25 mm = Rim: 630mm + 2x 25.00 = 680.00mm (d) * Pi = 2136.28mm
#ifndef CIRCUMFERENCE_WHEEL
#define CIRCUMFERENCE_WHEEL 2.136
#endif

#ifndef CIRCUMFERENCE_WHEEL_CM
#define CIRCUMFERENCE_WHEEL_CM 100.0*CIRCUMFERENCE_WHEEL
#endif

#ifndef FREE_ROLLING_FACTOR
#define FREE_ROLLING_FACTOR 1.15
#endif

#ifndef MAX_PID_OUTPUT_RANGE
#define MAX_PID_OUTPUT_RANGE 1023.0
#endif

#ifndef NUM_RESISTANCE_LEVELS
#define NUM_RESISTANCE_LEVELS 32
#endif

APP_TIMER_DEF(m_pid_timer_id); 
APP_TIMER_DEF(m_fec_timer_id);

static void debugCalculationHelper();

uint8_t getResistanceLevel();
void setResistanceLevel(uint8_t resistance_level);
void updateResistance(); // TODO: not yet implemented

static void fec_timer_handler(void * p_context);
// No setInstantaneousCadence! Always calculate!
#if HANDLE_FREEROLL
void setFreeRollDuration(uint16_t duration);
void setFreeRollStart(double start);
#endif 

uint16_t getLastCrankEventTime(); // just return, no calc: last_crank_event_time, used in cps, cscs_c and main (cscs)
uint16_t getLastWheelEventTime(); // just return, no calc: last_wheel_event_time, used in cps, cscs_c and main (cscs)

uint16_t getCumulativeCrankRevs(); // just return, no calc: cumulative_crank_revs, used in cps, cscs_c and main (cscs)
uint32_t getCumulativeWheelRevs(); // just return, no calc: cumulative_wheel_revs, used in cps, cscs_c and main (cscs)

uint8_t  getTrainerResistanceLevel();
void setTrainerResistanceLevel(uint8_t trainer_resistance_level);

double getInstantaneousCadence(); // just return no calc: i_cadence, used in ftms
double getAverageCadence(); // just return no calc: avg_cadence, used in ftms and cps

double getInstantaneousSpeed(); // just return no calc: i_speed (m/s), used in ftms
double getAverageSpeed(); // just return no calc: a_kmh, used in ftms and cps

uint16_t getInstantaneousPower(); // just return no calc
uint16_t getAveragePower(); // just return no calc

static void updateCadenceReadings(); // Internally needed (to check if it can be replaced)
static void setOledGrade();

static double updateInstantaneousSpeed(); // calc and return i_speed (m/s), only used internally
static double updateInstantaneousWheelRevs(); // calc and return i_wheel_revs, only used internally
static double updateAverageSpeed(); // calc and return a_kmh, only used internally
static double updateInstantaneousPower(); // calc and return i_power
static void updatePIDIntegrator(double i); // calc and update the PID integrator variable
static double updatePower();
double calculatePower(double rpm, uint8_t resistance_level);

static double getAveragedSpeed(double in_value); // Internally used for moving average

double calcPowerFromGivenCadence(uint16_t cadence);

static void debugPrintAchievedCumulativeWheelRevs();
static void debugPrintCadenceReadings();
static void debugPrintIWheelRevRelations();
static void debugPrintTimeSpanPerWheelRev();
static void debugPrintPIDValues(double d1, double d2, double d3, double i, double p, double ki, int16_t integrator, double output1, double output);

static void calculation_handler(void * p_context);

void initializeCalculationHelper();
void startProcessing();
void stopProcessing();
void updateModelFromCSCSensorData(const ble_gattc_evt_hvx_t *p_notif); // Most important thing to kick everyting of

void registerFECCallback(ble_fec_t *p_fec, ble_fec_page_handler_t fec_page_handler);

uint8_t getFECElapsedTime();
uint8_t getFECDistanceTraveled();
uint8_t getFECEventcount();
uint16_t getFECAccumulatedPower();
uint8_t getFECPage();
ble_fec_page_handler_t getFECPageHandler();
ble_fec_t *getFECHandle();

fec_mode_t getFECMode();

static void performResistanceCalculation();
void incResistanceLevel();
void decResistanceLevel();
void performModelCalculations();
void setFECMode(fec_mode_t mode);
void setUserWeight(uint8_t kg);
void setBikeWeight(uint8_t kg);
uint8_t getUserWeight();
uint8_t getBikeWeight();
void initModel();
void setGrade(uint16_t grade);
uint16_t getGrade();
void setCoefficientRollingResistance(uint8_t crr);
uint8_t getCoefficientRollingResistance();
static void performResistanceCalcultion();
void setWindResistanceCoefficient(uint8_t wcr);
uint8_t getWindResistanceCoefficient();
void setWindSpeed(int8_t windspeed);
int8_t getWindSpeed();
void setDraftingFactor(uint8_t drafting_factor);
uint8_t getDraftingFactor();
void fastInitializeModel();


#endif
