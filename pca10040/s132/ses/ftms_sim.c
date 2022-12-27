#include "ftms_sim.h"
#include "stdio.h"
#include "math.h"

double ms2kmh(double ms)
{
    return ms*3.6;
}

void set_ftms_simulation_parameter(ble_ftms_indoor_bike_simulation_parameters_t indoor_bike_simulation_parameters, double rider_relative_speed_ms)
{                       
    // Wind Resistance [N] = (0.5 Wind Resistance Coefficient * (Relative Speed / 3.6)^2) x Drafting Factor (ANT-FEC, p57)
    double wind_resistance_coefficient = ((double) indoor_bike_simulation_parameters.cw) / 100.0; // cw (default 0.51 kg/m (INDOOR_BIKE_SIMULATION_PARAMETERS.cw is 51.0)        
    printf("wind_resistance_coefficient = %f\n", wind_resistance_coefficient);

    double rolling_resistance_coefficient = (double) indoor_bike_simulation_parameters.crr / 10000.0; // crr (default 0.41 ((INDOOR_BIKE_SIMULATION_PARAMETERS.cw is 41.0)
    printf("rolling_resistance_coefficient = %f\n", rolling_resistance_coefficient);

    // wind_speed in FTMS is delivered in m/s  - not km/h like in FE-C!
    double relative_speed_ms = rider_relative_speed_ms + indoor_bike_simulation_parameters.wind_speed;
    printf("relative_speed_ms = %f, relative_speed_kmh = %f\n", relative_speed_ms, ms2kmh(relative_speed_ms));

    double wind_resistance = 0.5 * (wind_resistance_coefficient) * pow(relative_speed_ms,2) * DRAFTING_FACTOR;
    printf("wind_resistance = %f\n", wind_resistance);
    printf("P(wind_resistance) = %f\n", wind_resistance * relative_speed_ms);
    // Gravitational Resistance [N] = (Equipment Mass + User Mass) * Grade/100 * 9.81
    uint8_t total_weight = EQUIPMENT_MASS+USER_MASS;
    double gravitational_resistance = total_weight * (((double) indoor_bike_simulation_parameters.grade / 100.0) / 100.0) * 9.81;
    printf("gravitational_resistance = %f\n", gravitational_resistance);
    printf("P(gravitational_resistance) = %f\n", gravitational_resistance * relative_speed_ms);
    // Rolling Resistance [N] = (Bicycle Mass + Cyclist Mass) x Coefficient of Rolling Resistance x 9.8 
    double rolling_resistance = total_weight * rolling_resistance_coefficient * 9.81;
    printf("rolling_resistance = %f\n", rolling_resistance);
    printf("P(rolling_resistance) = %f\n", rolling_resistance * relative_speed_ms);
    // Total resistance [N] = Gravitational Resistance + Rolling Resistance + Wind Resistance
    double imposed_resistance = gravitational_resistance + rolling_resistance + wind_resistance;
    printf("imposed_resistance: %.3f (%.3f + %.3f + %.3f)\n", imposed_resistance, gravitational_resistance, rolling_resistance, wind_resistance); 
    printf("Watts needed to counter resistance: %f\n", imposed_resistance * relative_speed_ms);
    // Now we have the resistance that is imposed on us. Now try to find the resistance level of the trainer that matches best the resistance
    /*
    double least_error = 100000.0;
    uint8_t resistance_level_candidate = -1; 
    
    // Calc all power values for current cadence
    // This really looks superfluois now
    double calced_avg_cadence = kmh2rpm(getAverageSpeed());
    
    sprintf(buf, "Sanity check: CALC_AVG_CAD: %.2f, TRUE_AVG_CAD: %.2f" , calced_avg_cadence, getAverageCadence());
    NRF_LOG_INFO("%s", buf);

    for (uint8_t pot_resistance_level=1; pot_resistance_level<=NUM_RESISTANCE_LEVELS; pot_resistance_level++)
    {        
        double resulting_power = calculatePower(calced_avg_cadence, pot_resistance_level);
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

    if (resistance_level_candidate != -1)
    {
//#if LOGHERE
        NRF_LOG_INFO("Best resistance_level = %d. Current resistance_level = %d, Current gear delta: %d", resistance_level_candidate, target_resistance_level, gear_offset*2);
// #endif
        if (resistance_level_candidate != resistance_level)
        {
            // Change the resistance setting
            target_resistance_level = resistance_level_candidate;
        }
    }
    */
}