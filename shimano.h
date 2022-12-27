#ifndef SHIMANO_H__
#define SHIMANO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"

#ifdef PLAYGOUND
void shimano();
void printstuff();
#endif

#define NUM_OF_GEARS 12
#define INITIAL_GEAR 6

void checkGearBoundaries();
double getGearRatio();


#ifdef __cplusplus
}
#endif

#endif // SHIMANO_H__

