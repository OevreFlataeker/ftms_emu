/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

#include <stdio.h>
#include <stdlib.h>
// #include "PowerProfile.h"
#include "ftms_sim.h"

/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/

int gear_offset = 0;

int main(void) {
  // iteratePowerProfile();
  ble_ftms_indoor_bike_simulation_parameters_t indoor_bike_simulation_parameters;
  indoor_bike_simulation_parameters.crr = 50;
  indoor_bike_simulation_parameters.cw = 43;
  indoor_bike_simulation_parameters.grade = 500;
  indoor_bike_simulation_parameters.wind_speed = 0;
  double speed_ms = 40/3.6;
  set_ftms_simulation_parameter(indoor_bike_simulation_parameters, speed_ms);
}

/*************************** End of file ****************************/
