#ifndef CRYPTOUTIL_H__
#define CRYPTOUTIL_H__

#include "nrf_crypto.h"

void cryptoutil_init();
uint32_t get_random_uint32();
int32_t get_random_uint32_bounded(int32_t lowerbound, int32_t upperbound);

#endif