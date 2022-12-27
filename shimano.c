#include "shimano.h"

#define NRF_LOG_MODULE_NAME shimano
#if SHIMANO_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       SHIMANO_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  SHIMANO_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR SHIMANO_CONFIG_DEBUG_COLOR
#else // SHIMANO_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       3
#endif // SHIMANO_CONFIG_LOG_ENABLED
#include "nrf_log.h"

NRF_LOG_MODULE_REGISTER();




extern volatile int8_t gear_offset;

void checkGearBoundaries()
{
    if (gear_offset < -INITIAL_GEAR)
    {
      NRF_LOG_INFO("gear_offset < -%u -> assuming -%u", INITIAL_GEAR, INITIAL_GEAR);      
      gear_offset = -INITIAL_GEAR;    
    }

    if (gear_offset >= NUM_OF_GEARS-INITIAL_GEAR)
    {    
      NRF_LOG_INFO("gear_offset > %u -> assuming %u", NUM_OF_GEARS-INITIAL_GEAR-1, NUM_OF_GEARS-INITIAL_GEAR-1);      
      gear_offset = NUM_OF_GEARS-INITIAL_GEAR-1;
    }
}

double getGearRatio()
{   //
    // Gear ratios: Chainring: 40-52, Cog: 11-21, Chainring: fixed at 44
    // https://www.bikecalc.com/gear_ratios

    // double ratios[11] =  {2.1, 2.2, 2.32, 2.44, 2.58, 2.75, 2.93, 3.14, 3.38, 3.67, 4.0};

    // With Shimano 10-12-14-16-18-21-24-28-32-36-40-45 12-speed Cassette, 46 chain ring
    

    int gears = NUM_OF_GEARS;
    // With Shimano 10,12,14,16,18,21,24,28,32,36,40,45 12-speed Cassette, 46 chain ring
    //double ratios[NUM_OF_GEARS] =  {2.1, 2.3, 2.6, 2.9, 3.3, 3.9, 4.4, 5.2, 5.8, 6.6, 7.7, 9.3};
    //#define INITIAL_GEAR 2 //                 ^^^ 3rd element in array
    

    // With Shimano 45,40,36,32,28,24,21,18,16,14,12,10 12-speed Cassette, 53 chain ring    
    //                                ^^
    double ratios[NUM_OF_GEARS] =  { 1.18, 1.33, 1.47, 1.66, 1.89, 2.21, 2.52, 2.94, 3.31, 3.79, 4.42, 5.3};
    //                     //                                             ^^^ 3rd element in array

    
    int8_t index = 0;

    
    checkGearBoundaries();
    index = gear_offset; 
     
    //char buf[16];
    //sprintf(buf, "%.2f",  ratios[index+INITIAL_GEAR]);
    //NRF_LOG_INFO("Gear ratio: %s", buf);
    return ratios[index + INITIAL_GEAR];
}

#ifdef PLAYGOUND
void printstuff()
{    
    double ratio = getGearRatio();    
    printf("Current gear: %d, Ratio: %.2f\n", gear_offset, ratio); 
}

void shimano() 
{
    for (int idx=-3; idx<11; idx++)
    {
        gear_offset = idx;
        printstuff();
    }
}

#endif // PLAYGROUND