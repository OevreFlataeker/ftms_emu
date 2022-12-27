#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h> 
#include "PowerProfile.h"
#include "play_shimano.h"

extern int gear_offset;

void iteratePowerProfile()
{        
        char buf[1024];
        
        
        for (double rpm = 60.0; rpm<101.0; rpm+=5.0)
        {
            printf("RPM: %.1f\n", rpm);
            for (int gear_idx = -4; gear_idx<=6; gear_idx++)
            {
                double average_speed = 0.0;
                double instantaneous_cadence = rpm / 60.0;
                gear_offset = gear_idx;
                double wheel_revs = instantaneous_cadence * getGearRatio();
                average_speed = wheel_revs * 2136.0 * 3.6 / 1000.00;

                memset(buf, 0, sizeof(buf));
                int next_pos = 0;    
                for (uint8_t tested_resistance_level=1; tested_resistance_level<=30; tested_resistance_level++)
                {
                    double resulting_power = calculate_power_from_avg_cadence(rpm, tested_resistance_level);            
                    double resulting_resistance = resulting_power / (average_speed * 1000.0 / 3600.0);
                    next_pos += sprintf(buf+next_pos, "%06.2f W/%05.2fN @ %5.2f km/h, ", resulting_power, resulting_resistance, average_speed);            
                    if (next_pos > sizeof(buf))
                    {
                        printf("Increase buf size postmortem to at least %d!\n", next_pos);
                        exit(1);
                    }
                }
                printf("%d: %s\n", gear_idx, buf);
               
            }
            printf("\n");            
        }  
}

double calculate_power_from_avg_cadence(double rpm, uint8_t resistance_level)
{
  double zPower = 0.0;
  switch(resistance_level)
        {
           case 1:
              zPower = 62.0 + (rpm - 80) * 2.0;
              break;
            case 2:
              zPower = 75.0 + (rpm - 80) * 2.0;
              break;
            case 3:
              zPower = 86.0 + (rpm - 80) * 2.0;
              break;
            case 4:
              zPower = 102.0 + (rpm - 80) * 2.0;
              break;
            case 5:
              zPower = 116.0 + (rpm - 80) * 2.0;
              break;
            case 6:
              zPower = 128.0 + (rpm -80) * 2.0;
              break;
            case 7:
              zPower = 139.0 + (rpm - 80) * 2.0;
              break;
            case 8:
              zPower = 151.0 + (rpm - 80) * 3.0;
              break;
            case 9:
              zPower = 168.0 + (rpm - 80) * 3.0;
              break;
            case 10:
              zPower = 178.0 + (rpm - 80) * 3.0;
              break;
            case 11:
              zPower = 190.0 + (rpm - 80) * 3.0;
              break;
            case 12:
              zPower = 200.0 + (rpm - 80) * 3.0;
              break;
            case 13:
              zPower = 210.0 + (rpm - 80) * 3.0;
              break;
            case 14:
              zPower = 220.0 + (rpm - 80) * 3.0;
              break;
            case 15:
              zPower = 230.0 + (rpm - 80) * 3.0;
              break;
            case 16:
              zPower = 236.0 + (rpm - 78) * 4.0;
              break;
            case 17:
              zPower = 240.0 + (rpm - 76) * 4.0;
              break;
            case 18:
              zPower = 248.0 + (rpm - 75) * 4.0;
              break;
            case 19:
              zPower = 258.0 + (rpm - 75) * 4.0;
              break;
            case 20:
              zPower = 268.0 + (rpm - 74) * 4.0; // Eval + 10
              break;
            case 21:
              zPower = 278.0 + (rpm - 75) * 4.0; // Guess
              break;
            case 22:
              zPower = 288.0 + (rpm - 75) * 4.0; // Guess
              break;
            case 23:
              zPower = 298.0 + (rpm - 75) * 4.0; // Guess
              break;
            case 24:
              zPower = 308.0 + (rpm - 75) * 4.0; // Guess
              break;
            case 25:
              zPower = 318.0 + (rpm - 75) * 4.0; // Guess
              break;
            case 26:
              zPower = 328.0 + (rpm - 75) * 4.0; // Guess
              break;
            case 27:
              zPower = 338.0 + (rpm - 75) * 4.0; // Guess
              break;
            case 28:
              zPower = 348.0 + (rpm - 75) * 4.0; // Guess
              break;
            case 29:
              zPower = 358.0 + (rpm - 75) * 4.0; // Guess
              break;
            case 30:
              zPower = 368.0 + (rpm - 75) * 4.0; // Guess
              break;
              default:
              
              zPower = 0.0;
            break;
        }       
        return zPower;
}