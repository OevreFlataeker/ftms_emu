
#include "stdio.h"
#include "stdint.h"

#define NUM_OF_GEARS 12
#define INITIAL_GEAR 2

#ifdef PLAYGOUND
void shimano();
void printstuff();
#endif

double getGearRatio();