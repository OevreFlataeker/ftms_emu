#include <stdint.h>

#define DRAFTING_FACTOR 1.0
#define EQUIPMENT_MASS 0
#define USER_MASS 75

typedef struct{
    int16_t wind_speed;
    int16_t grade;
    uint8_t crr;
    uint8_t cw;
} ble_ftms_indoor_bike_simulation_parameters_t;

double ms2kmh(double ms);

void set_ftms_simulation_parameter(ble_ftms_indoor_bike_simulation_parameters_t indoor_bike_simulation_parameters, double rider_relative_speed_ms);