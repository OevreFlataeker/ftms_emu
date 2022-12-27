#include "play_shimano.h"

extern int gear_offset;

double getGearRatio()
{   //
    // Gear ratios: Chainring: 40-52, Cog: 11-21, Chainring: fixed at 44
    // https://www.bikecalc.com/gear_ratios

    // double ratios[11] =  {2.1, 2.2, 2.32, 2.44, 2.58, 2.75, 2.93, 3.14, 3.38, 3.67, 4.0};

    
    // 
    int gears = NUM_OF_GEARS;

    // With Shimano 10,12,14,16,18,21,24,28,32,36,40,45 12-speed Cassette, 46 chain ring
    //double ratios[NUM_OF_GEARS] =  {2.1, 2.3, 2.6, 2.9, 3.3, 3.9, 4.4, 5.2, 5.8, 6.6, 7.7, 9.3};
    //#define INITIAL_GEAR 2 //                 ^^^ 3rd element in array
    
    // With Shimano 45,40,36,32,28,24,21,18,16,14,12,10 12-speed Cassette, 53 chain ring    
    double ratios[NUM_OF_GEARS] =  { 1.18, 1.33, 1.47, 1.66, 1.89, 2.21, 2.52, 2.94, 3.31, 3.79, 4.42, 5.3};
    #define INITIAL_GEAR 6 //                                             ^^^ 3rd element in array

    int8_t index = 0;
    index = gear_offset; 

    if (index < -INITIAL_GEAR)
    {
      printf("gear_offset < -%u -> assuming -%u", INITIAL_GEAR, INITIAL_GEAR);      
      index = -INITIAL_GEAR;    
    }

    if (index >= NUM_OF_GEARS-INITIAL_GEAR)
    {    
      printf("gear_offset > %u -> assuming %u", NUM_OF_GEARS-INITIAL_GEAR-1, NUM_OF_GEARS-INITIAL_GEAR-1);      
      index = NUM_OF_GEARS-INITIAL_GEAR-1;
    }
    
    char buf[16];
    sprintf(buf, "%.3f",  ratios[index+INITIAL_GEAR]);
    printf("Gear ratio: %s", buf);
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

#endif PLAYGROUND