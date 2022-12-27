#include "sdk_common.h"
#include "cryptoutil.h"
#include "app_error.h"

#if NRF_MODULE_ENABLED(CRYPTOUTIL)
#define NRF_LOG_MODULE_NAME cryptoutil
#if CRYPTOUTIL_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       CRYPTOUTIL_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  CRYPTOUTIL_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR CRYPTOUTIL_CONFIG_DEBUG_COLOR
#else // CRYPTOUTIL_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // CRYPTOUTIL_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

// #define VECTOR_LENGTH   4
// #define ITERATIONS      5

// static uint8_t m_random_vector[VECTOR_LENGTH];
// static uint8_t m_min[VECTOR_LENGTH] = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Big-endian byte array
// static uint8_t m_max[VECTOR_LENGTH] = {0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Big-endian byte array

void cryptoutil_init()
{
ret_code_t ret_val;

    NRF_LOG_INFO("Initializing cryptoutil");
    ret_val = nrf_crypto_init();
    APP_ERROR_CHECK(ret_val);
}

uint32_t get_random_uint32()
{
ret_code_t ret_val;
uint8_t m_32[4];

    ret_val = nrf_crypto_rng_vector_generate(m_32, 4);
    APP_ERROR_CHECK(ret_val);
    uint32_t result = uint32_decode(m_32);
    return result;
}

int32_t get_random_uint32_bounded(int32_t lowerbound, int32_t upperbound)
{
ret_code_t ret_val;
uint32_t rnd;

    rnd = get_random_uint32();

    if (upperbound < lowerbound)
    {
        NRF_LOG_ERROR("get_random_uint32_bounded(): upperbound < lowerbound! Switching params!");
        uint32_t tmp = upperbound;
        upperbound = lowerbound;
        lowerbound = tmp;
    }

    uint32_t  diff  = upperbound - lowerbound + 1;      
    int32_t result = lowerbound + rnd % diff; 
      
    return result;                                                               
}

#endif // NRF_MODULE_ENABLED(CRYPTOUTIL)