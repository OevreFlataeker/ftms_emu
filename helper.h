#ifndef HELPER_H
#define HELPER_H

#include "ble_ftms_ctrlpt.h"
#include "ble_ftms.h"
#include "ble_cscs_c.h"
#include "oled_controller.h"

#define TRIGGER_RESISTANCE_UP_GPIO 24
#define TRIGGER_RESISTANCE_DOWN_GPIO 25
#define MAX_WATT 666
#define TRIGGER_DURATION 100
#define NUM_RESISTANCE_LEVELS 32

extern volatile uint8_t resistance_level;
extern volatile uint8_t target_resistance_level;
extern volatile int8_t gear_offset;

typedef enum {
  TRIGGER_UP,
  TRIGGER_DOWN
} trigger_direction_t;


/**@brief Function for decoding a sint16 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE int16_t sint16_decode(const int8_t * p_encoded_data)
{
        return ( (((int16_t)((uint8_t *)p_encoded_data)[0])) |
                 (((int16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}

/**@brief Function for encoding a sint16 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */

static __INLINE int8_t sint16_encode(int16_t value, int8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (int8_t) ((value & 0xFF00) >> 8);
    return sizeof(int16_t);
}

double kmh2ms (double kmh);
double ms2kmh (double ms);
double kmh2rpm (double kmh);

double getGearRatio();
void set_target_power(int16_t target_power, uint16_t target_cadence);
void set_target_resistance(uint8_t target_resistance);
void triggerResistancePlus();
void triggerResistanceMinus();
void triggerResistanceChange(trigger_direction_t direction);
static void gpio_handler(void *context);
static void debugPrintSimulationResistance(int16_t imposed_resistance, 
                                           double gravitational_resistance,
                                           double rolling_resistance,
                                           double wind_resistance);
void set_ftms_simulation_parameter(ble_ftms_indoor_bike_simulation_parameters_t indoor_bike_simulation_parameters);
char *hex2str(char *buf, uint8_t buflen, uint8_t * data, uint8_t datalen);
double calculate_power_from_avg_cadence(double avg_cadence, uint8_t resistance_level);
#endif
