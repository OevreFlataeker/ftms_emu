

#ifndef COUNTER_H__
#define COUNTER_H__

#include <stdint.h>

/**@brief   Function for initializing the RTC driver instance. */
void counter_init(void);


/**@brief   Function for starting the counter. */
void counter_start(void);


/**@brief   Function for stopping the counter. */
void counter_stop(void);


/**@brief   Function for retrieving the counter value.
 * @details The counter is configured to tick once every 1 millisecond.
 */
uint32_t counter_get(void);

#endif // COUNTER_H__

/** @}
 *  @endcond
 */
