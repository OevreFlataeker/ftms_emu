

#include "counter.h"
#include "nrfx_rtc.h"


// RTC driver instance using RTC2.
// RTC0 is used by the SoftDevice, and RTC1 by the app_timer library.
static const nrfx_rtc_t m_rtc = NRFX_RTC_INSTANCE(2);
static volatile uint32_t overflow_counter = 0;

static void rtc_handler(nrfx_rtc_int_type_t int_type)
{
    if (int_type == NRFX_RTC_INT_OVERFLOW) overflow_counter++;    // Used for timer extension
}

void counter_init(void)
{
    ret_code_t err_code;

    // Initialize the RTC instance.
    nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;

    // 1 ms interval.
    config.prescaler = 1;
    //config.prescaler = 256; // prescaler = 1 gives proper timestamps

    err_code = nrfx_rtc_init(&m_rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_rtc_tick_disable(&m_rtc);
}


void counter_start(void)
{
    nrfx_rtc_counter_clear(&m_rtc);
    nrfx_rtc_overflow_enable(&m_rtc, true);  // Count overflow events to extend counter, see below
    // Power on!
    nrfx_rtc_enable(&m_rtc);
}


void counter_stop(void)
{
    nrfx_rtc_disable(&m_rtc);
}


uint32_t counter_get(void)
{
    uint32_t cnt = nrfx_rtc_counter_get(&m_rtc);
    cnt |= overflow_counter << 24;    // We artifically "extend" the counter from 24 bit to 32 bit doing so.
    return cnt;
}

/** @}
 *  @endcond
 */
