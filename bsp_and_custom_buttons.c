
#include "bsp_and_custom_buttons.h"
#include <stddef.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "bsp_config.h"
#include "boards.h"

#ifndef BSP_SIMPLE
#include "app_timer.h"
#include "app_button.h"
#endif // BSP_SIMPLE

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "assert.h"
#include "nrf_delay.h"

#include "helper.h"
#include "oled_controller.h" 
#include "shimano.h"

// Make the compiler happy
#ifdef BUTTONS_NUMBER
#undef BUTTONS_NUMBER
#endif

#ifndef LIGHTSOUT
#define LIGHTSOUT 0
#endif

#define STEERING 1

#if STEERING
#define BUTTONS_NUMBER 10 // Ensure sdk_config.h, GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS is set to a higher value when out-of-memory errors occur during init! Default = 6
#else
#define BUTTONS_NUMBER 6
#endif

#define BUTTON_5       22  // Rear shift up
#define BUTTON_6       23  // Rear shift down

#if STEERING
#define BUTTON_7       11
#define BUTTON_8       12
#endif

#define BUTTON_9       26 // Front shift up 
#define BUTTON_10      27 // Front shift down

#define TRAINER_BUTTON_GEAR_UP BUTTON_5
#define TRAINER_BUTTON_GEAR_DOWN BUTTON_6

#define TRAINER_BUTTON_FRONT_GEAR_UP BUTTON_9
#define TRAINER_BUTTON_FRONT_GEAR_DOWN BUTTON_10

#if STEERING
#define STEER_BUTTON_LEFT BUTTON_7
#define STEER_BUTTON_RIGHT BUTTON_8
#endif

#define BSP_BUTTON_5  BUTTON_5
#define BSP_BUTTON_6  BUTTON_6

#if STEERING
    #define BSP_BUTTON_7  BUTTON_7
    #define BSP_BUTTON_8  BUTTON_8
#endif

#define BSP_BUTTON_9  BUTTON_9
#define BSP_BUTTON_10  BUTTON_10

// Make the compiler happy
#ifdef BUTTON_STOP
#undef BUTTON_STOP
#endif

#define BUTTON_STOP 27

// Make the compiler happy
#ifdef BUTTONS_LIST
#undef BUTTONS_LIST
#endif

#if STEERING
#define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4, BUTTON_5, BUTTON_6, BUTTON_7, BUTTON_8, BUTTON_9, BUTTON_10 }
#else 
#define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4, BUTTON_5, BUTTON_6, BUTTON_9, BUTTON_10 }
#endif

// Shall we enable the onboard LEDs? LIGHTSOUT==1 ? no : yes

#ifdef LIGHTSOUT
    #if LIGHTSOUT
    #ifdef LEDS_NUMBER
        #undef LEDS_NUMBER
        #define LEDS_NUMBER 0
    #endif
    #endif
#endif

#if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
static bsp_indication_t m_stable_state        = BSP_INDICATE_IDLE;
static bool             m_leds_clear          = false;
static uint32_t         m_indication_type     = 0;
static bool             m_alert_on            = false;
APP_TIMER_DEF(m_bsp_leds_tmr);
APP_TIMER_DEF(m_bsp_alert_tmr);
#endif // LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)

#if BUTTONS_NUMBER > 0
#ifndef BSP_SIMPLE
static bsp_event_callback_t   m_registered_callback         = NULL;
static bsp_button_event_cfg_t m_events_list[BUTTONS_NUMBER] = {{BSP_EVENT_NOTHING, BSP_EVENT_NOTHING}};
APP_TIMER_DEF(m_bsp_button_tmr);
static void gear_button_event_handler(uint8_t pin_no, uint8_t button_action);
static void bsp_button_event_handler(uint8_t pin_no, uint8_t button_action);
#if STEERING
static void steer_button_event_handler(uint8_t pin_no, uint8_t button_action);
#endif
#endif // BSP_SIMPLE

extern uint8_t incline_level;
extern oled_data_t oled_data;
#if STEERING
extern int8_t sterzo_steer_value;
#endif

gear_button_state_t gear_button_state;

#ifndef BSP_SIMPLE
static const app_button_cfg_t app_buttons[BUTTONS_NUMBER] =
{
    #ifdef BSP_BUTTON_0
    {BSP_BUTTON_0, false, BUTTON_PULL, bsp_button_event_handler},
    #endif // BUTTON_0

    #ifdef BSP_BUTTON_1
    {BSP_BUTTON_1, false, BUTTON_PULL, bsp_button_event_handler},
    #endif // BUTTON_1

    #ifdef BSP_BUTTON_2
    {BSP_BUTTON_2, false, BUTTON_PULL, bsp_button_event_handler},
    #endif // BUTTON_2

    #ifdef BSP_BUTTON_3
    {BSP_BUTTON_3, false, BUTTON_PULL, bsp_button_event_handler},
    #endif // BUTTON_3
    
    #ifdef BSP_BUTTON_4
    {BSP_BUTTON_4, false, BUTTON_PULL, bsp_button_event_handler},
    #endif // BUTTON_4

    #ifdef BSP_BUTTON_5
    {BSP_BUTTON_5, false, BUTTON_PULL, gear_button_event_handler},
    #endif // BUTTON_5

    #ifdef BSP_BUTTON_6
    {BSP_BUTTON_6, false, BUTTON_PULL, gear_button_event_handler},
    #endif // BUTTON_6

    //#ifdef BSP_BUTTON_7
    //{BSP_BUTTON_7, false, BUTTON_PULL, bsp_button_event_handler},
    //#endif // BUTTON_7
    #if STEERING
    #ifdef BSP_BUTTON_7
    {BSP_BUTTON_7, false, BUTTON_PULL, steer_button_event_handler},
    #endif // BUTTON_7
    #ifdef BSP_BUTTON_8
    {BSP_BUTTON_8, false, BUTTON_PULL, steer_button_event_handler},
    #endif // BUTTON_8

    #ifdef BSP_BUTTON_9
    {BSP_BUTTON_9, false, BUTTON_PULL, gear_button_event_handler},
    #endif // BUTTON_9

    #ifdef BSP_BUTTON_10
    {BSP_BUTTON_10, false, BUTTON_PULL, gear_button_event_handler},
    #endif // BUTTON_10
    #endif 

};
#endif // BSP_SIMPLE
#endif // BUTTONS_NUMBER > 0

#if STEERING
static void steer_button_event_handler(uint8_t pin_no, uint8_t button_action)
{	
        
        if(app_button_is_pushed(6))
	{                
                NRF_LOG_INFO("Button STEER_LEFT got pressed!");
                sterzo_steer_value = -35;
	}
        else if(app_button_is_pushed(7))
        {               
               NRF_LOG_INFO("Button STEER_RIGHT got pressed!");                
               sterzo_steer_value = 35;
        }	
        
        if (!(app_button_is_pushed(6) || app_button_is_pushed(7)))
        {
            sterzo_steer_value = 0;
        }
}
#endif

static void gear_button_event_handler(uint8_t pin_no, uint8_t button_action)
{
	if (app_button_is_pushed(4) && app_button_is_pushed(5))
        {
            //NRF_LOG_INFO("Reset!");
            gear_button_state = GEAR_BUTTON_PRESSED_RESET;
        }
        
        else if(app_button_is_pushed(4))
	{
                gear_button_state = GEAR_BUTTON_PRESSED_GEAR_UP;
                
	}
        else if(app_button_is_pushed(5))
        {
               gear_button_state = GEAR_BUTTON_PRESSED_GEAR_DOWN;

               
        }	

        
        // Front gear handling
        if (app_button_is_pushed(8))
        {
            gear_button_state = GEAR_BUTTON_PRESSED_FRONT_GEAR_UP;
            //NRF_LOG_INFO("Button TRAINER_BUTTON_FRONT_GEAR_UP got pressed! No action yet");               
        }
        else if(app_button_is_pushed(9))
        {   
            gear_button_state = GEAR_BUTTON_PRESSED_FRONT_GEAR_DOWN;
            // NRF_LOG_INFO("Button TRAINER_BUTTON_FRONT_GEAR_DOWN got pressed! No action yet");               
        }
        
}

#if (BUTTONS_NUMBER > 0)
bool bsp_button_is_pressed(uint32_t button)
{
    if (button < BUTTONS_NUMBER)
    {
        return bsp_board_button_state_get(button);
    }
    else
    {
        //If button is not present always return false
        return false;
    }
}
#endif

#if (BUTTONS_NUMBER > 0) && !(defined BSP_SIMPLE)
/**@brief Function for handling button events.
 *
 * @param[in]   pin_no          The pin number of the button pressed.
 * @param[in]   button_action   Action button.
 */
static void bsp_button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    bsp_event_t        event  = BSP_EVENT_NOTHING;
    uint32_t           button = 0;
    uint32_t           err_code;
    static uint8_t     current_long_push_pin_no;              /**< Pin number of a currently pushed button, that could become a long push if held long enough. */
    static bsp_event_t release_event_at_push[BUTTONS_NUMBER]; /**< Array of what the release event of each button was last time it was pushed, so that no release event is sent if the event was bound after the push of the button. */

    button = bsp_board_pin_to_button_idx(pin_no);

    if (button < BUTTONS_NUMBER)
    {
        switch (button_action)
        {
            case APP_BUTTON_PUSH:
                event = m_events_list[button].push_event;
                if (m_events_list[button].long_push_event != BSP_EVENT_NOTHING)
                {
                    err_code = app_timer_start(m_bsp_button_tmr, APP_TIMER_TICKS(BSP_LONG_PUSH_TIMEOUT_MS), (void*)&current_long_push_pin_no);
                    if (err_code == NRF_SUCCESS)
                    {
                        current_long_push_pin_no = pin_no;
                    }
                }
                release_event_at_push[button] = m_events_list[button].release_event;
                break;
            case APP_BUTTON_RELEASE:
                (void)app_timer_stop(m_bsp_button_tmr);
                if (release_event_at_push[button] == m_events_list[button].release_event)
                {
                    event = m_events_list[button].release_event;
                }
                break;
            case BSP_BUTTON_ACTION_LONG_PUSH:
                event = m_events_list[button].long_push_event;
        }
    }

    if ((event != BSP_EVENT_NOTHING) && (m_registered_callback != NULL))
    {
        m_registered_callback(event);
    }
}

/**@brief Handle events from button timer.
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */
static void button_timer_handler(void * p_context)
{
    bsp_button_event_handler(*(uint8_t *)p_context, BSP_BUTTON_ACTION_LONG_PUSH);
}


#endif // (BUTTONS_NUMBER > 0) && !(defined BSP_SIMPLE)


#if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
static void leds_off(void)
{
    if (m_alert_on)
    {
        uint32_t i;
        for (i = 0; i < LEDS_NUMBER; i++)
        {
            if (i != BSP_LED_ALERT)
            {
                bsp_board_led_off(i);
            }
        }
    }
    else
    {
        bsp_board_leds_off();
    }
}


/**@brief       Configure leds to indicate required state.
 * @param[in]   indicate   State to be indicated.
 */
static uint32_t bsp_led_indication(bsp_indication_t indicate)
{
    uint32_t err_code   = NRF_SUCCESS;
    uint32_t next_delay = 0;

    if (m_leds_clear)
    {
        m_leds_clear = false;
        leds_off();
    }

    switch (indicate)
    {
        case BSP_INDICATE_IDLE:
            leds_off();
            err_code       = app_timer_stop(m_bsp_leds_tmr);
            m_stable_state = indicate;
            break;

        case BSP_INDICATE_SCANNING:
        case BSP_INDICATE_ADVERTISING:
            // in advertising blink LED_0
            if (bsp_board_led_state_get(BSP_LED_INDICATE_INDICATE_ADVERTISING))
            {
                bsp_board_led_off(BSP_LED_INDICATE_INDICATE_ADVERTISING);
                next_delay = indicate ==
                             BSP_INDICATE_ADVERTISING ? ADVERTISING_LED_OFF_INTERVAL :
                             ADVERTISING_SLOW_LED_OFF_INTERVAL;
            }
            else
            {
                bsp_board_led_on(BSP_LED_INDICATE_INDICATE_ADVERTISING);
                next_delay = indicate ==
                             BSP_INDICATE_ADVERTISING ? ADVERTISING_LED_ON_INTERVAL :
                             ADVERTISING_SLOW_LED_ON_INTERVAL;
            }

            m_stable_state = indicate;
            err_code       = app_timer_start(m_bsp_leds_tmr, APP_TIMER_TICKS(next_delay), NULL);
            break;

        case BSP_INDICATE_ADVERTISING_WHITELIST:
            // in advertising quickly blink LED_0
            if (bsp_board_led_state_get(BSP_LED_INDICATE_ADVERTISING_WHITELIST))
            {
                bsp_board_led_off(BSP_LED_INDICATE_ADVERTISING_WHITELIST);
                next_delay = indicate ==
                             BSP_INDICATE_ADVERTISING_WHITELIST ?
                             ADVERTISING_WHITELIST_LED_OFF_INTERVAL :
                             ADVERTISING_SLOW_LED_OFF_INTERVAL;
            }
            else
            {
                bsp_board_led_on(BSP_LED_INDICATE_ADVERTISING_WHITELIST);
                next_delay = indicate ==
                             BSP_INDICATE_ADVERTISING_WHITELIST ?
                             ADVERTISING_WHITELIST_LED_ON_INTERVAL :
                             ADVERTISING_SLOW_LED_ON_INTERVAL;
            }
            m_stable_state = indicate;
            err_code       = app_timer_start(m_bsp_leds_tmr, APP_TIMER_TICKS(next_delay), NULL);
            break;

        case BSP_INDICATE_ADVERTISING_SLOW:
            // in advertising slowly blink LED_0
            if (bsp_board_led_state_get(BSP_LED_INDICATE_ADVERTISING_SLOW))
            {
                bsp_board_led_off(BSP_LED_INDICATE_ADVERTISING_SLOW);
                next_delay = indicate ==
                             BSP_INDICATE_ADVERTISING_SLOW ? ADVERTISING_SLOW_LED_OFF_INTERVAL :
                             ADVERTISING_SLOW_LED_OFF_INTERVAL;
            }
            else
            {
                bsp_board_led_on(BSP_LED_INDICATE_ADVERTISING_SLOW);
                next_delay = indicate ==
                             BSP_INDICATE_ADVERTISING_SLOW ? ADVERTISING_SLOW_LED_ON_INTERVAL :
                             ADVERTISING_SLOW_LED_ON_INTERVAL;
            }
            m_stable_state = indicate;
            err_code       = app_timer_start(m_bsp_leds_tmr, APP_TIMER_TICKS(next_delay), NULL);
            break;

        case BSP_INDICATE_ADVERTISING_DIRECTED:
            // in advertising very quickly blink LED_0
            if (bsp_board_led_state_get(BSP_LED_INDICATE_ADVERTISING_DIRECTED))
            {
                bsp_board_led_off(BSP_LED_INDICATE_ADVERTISING_DIRECTED);
                next_delay = indicate ==
                             BSP_INDICATE_ADVERTISING_DIRECTED ?
                             ADVERTISING_DIRECTED_LED_OFF_INTERVAL :
                             ADVERTISING_SLOW_LED_OFF_INTERVAL;
            }
            else
            {
                bsp_board_led_on(BSP_LED_INDICATE_ADVERTISING_DIRECTED);
                next_delay = indicate ==
                             BSP_INDICATE_ADVERTISING_DIRECTED ?
                             ADVERTISING_DIRECTED_LED_ON_INTERVAL :
                             ADVERTISING_SLOW_LED_ON_INTERVAL;
            }
            m_stable_state = indicate;
            err_code       = app_timer_start(m_bsp_leds_tmr, APP_TIMER_TICKS(next_delay), NULL);
            break;

        case BSP_INDICATE_BONDING:
            // in bonding fast blink LED_0
            bsp_board_led_invert(BSP_LED_INDICATE_BONDING);

            m_stable_state = indicate;
            err_code       =
                app_timer_start(m_bsp_leds_tmr, APP_TIMER_TICKS(BONDING_INTERVAL), NULL);
            break;

        case BSP_INDICATE_CONNECTED:
            bsp_board_led_on(BSP_LED_INDICATE_CONNECTED);
            m_stable_state = indicate;
            break;

        case BSP_INDICATE_SENT_OK:
            // when sending shortly invert LED_1
            m_leds_clear = true;
            bsp_board_led_invert(BSP_LED_INDICATE_SENT_OK);
            err_code = app_timer_start(m_bsp_leds_tmr, APP_TIMER_TICKS(SENT_OK_INTERVAL), NULL);
            break;

        case BSP_INDICATE_SEND_ERROR:
            // on receving error invert LED_1 for long time
            m_leds_clear = true;
            bsp_board_led_invert(BSP_LED_INDICATE_SEND_ERROR);
            err_code = app_timer_start(m_bsp_leds_tmr, APP_TIMER_TICKS(SEND_ERROR_INTERVAL), NULL);
            break;

        case BSP_INDICATE_RCV_OK:
            // when receving shortly invert LED_1
            m_leds_clear = true;
            bsp_board_led_invert(BSP_LED_INDICATE_RCV_OK);
            err_code = app_timer_start(m_bsp_leds_tmr, APP_TIMER_TICKS(RCV_OK_INTERVAL), NULL);
            break;

        case BSP_INDICATE_RCV_ERROR:
            // on receving error invert LED_1 for long time
            m_leds_clear = true;
            bsp_board_led_invert(BSP_LED_INDICATE_RCV_ERROR);
            err_code = app_timer_start(m_bsp_leds_tmr, APP_TIMER_TICKS(RCV_ERROR_INTERVAL), NULL);
            break;

        case BSP_INDICATE_FATAL_ERROR:
            // on fatal error turn on all leds
            bsp_board_leds_on();
            m_stable_state = indicate;
            break;

        case BSP_INDICATE_ALERT_0:
        case BSP_INDICATE_ALERT_1:
        case BSP_INDICATE_ALERT_2:
        case BSP_INDICATE_ALERT_3:
        case BSP_INDICATE_ALERT_OFF:
            err_code   = app_timer_stop(m_bsp_alert_tmr);
            next_delay = (uint32_t)BSP_INDICATE_ALERT_OFF - (uint32_t)indicate;

            // a little trick to find out that if it did not fall through ALERT_OFF
            if (next_delay && (err_code == NRF_SUCCESS))
            {
                if (next_delay > 1)
                {
                    err_code = app_timer_start(m_bsp_alert_tmr,
                                               APP_TIMER_TICKS(((uint16_t)next_delay * ALERT_INTERVAL)),
                                               NULL);
                }
                bsp_board_led_on(BSP_LED_ALERT);
                m_alert_on = true;
            }
            else
            {
                bsp_board_led_off(BSP_LED_ALERT);
                m_alert_on = false;

            }
            break;

        case BSP_INDICATE_USER_STATE_OFF:
            leds_off();
            m_stable_state = indicate;
            break;

        case BSP_INDICATE_USER_STATE_0:
            leds_off();
            bsp_board_led_on(BSP_LED_INDICATE_USER_LED1);
            m_stable_state = indicate;
            break;

        case BSP_INDICATE_USER_STATE_1:
            leds_off();
            bsp_board_led_on(BSP_LED_INDICATE_USER_LED2);
            m_stable_state = indicate;
            break;

        case BSP_INDICATE_USER_STATE_2:
            leds_off();
            bsp_board_led_on(BSP_LED_INDICATE_USER_LED1);
            bsp_board_led_on(BSP_LED_INDICATE_USER_LED2);
            m_stable_state = indicate;
            break;

        case BSP_INDICATE_USER_STATE_3:

        case BSP_INDICATE_USER_STATE_ON:
            bsp_board_leds_on();
            m_stable_state = indicate;
            break;

        default:
            break;
    }

    return err_code;
}


/**@brief Handle events from leds timer.
 *
 * @note Timer handler does not support returning an error code.
 * Errors from bsp_led_indication() are not propagated.
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */
static void leds_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_indication_type & BSP_INIT_LEDS)
    {
        UNUSED_VARIABLE(bsp_led_indication(m_stable_state));
    }
}


/**@brief Handle events from alert timer.
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */
static void alert_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    bsp_board_led_invert(BSP_LED_ALERT);
}
#endif // #if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)


/**@brief Configure indicators to required state.
 */
uint32_t bsp_indication_set(bsp_indication_t indicate)
{
    uint32_t err_code = NRF_SUCCESS;

#if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)

    if (m_indication_type & BSP_INIT_LEDS)
    {
        err_code = bsp_led_indication(indicate);
    }

#endif // LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
    return err_code;
}


uint32_t bsp_init(uint32_t type, bsp_event_callback_t callback)
{
    uint32_t err_code = NRF_SUCCESS;

#if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
    m_indication_type     = type;
#endif // LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)

#if (BUTTONS_NUMBER > 0) && !(defined BSP_SIMPLE)
    m_registered_callback = callback;

    // BSP will support buttons and generate events
    if (type & BSP_INIT_BUTTONS)
    {
        uint32_t num;

        for (num = 0; ((num < BUTTONS_NUMBER) && (err_code == NRF_SUCCESS)); num++)
        {
            err_code = bsp_event_to_button_action_assign(num, BSP_BUTTON_ACTION_PUSH, BSP_EVENT_DEFAULT);
        }

        if (err_code == NRF_SUCCESS)
        {
            err_code = app_button_init((app_button_cfg_t *)app_buttons,
                                       BUTTONS_NUMBER,
                                       APP_TIMER_TICKS(50));
        }

        if (err_code == NRF_SUCCESS)
        {
            err_code = app_button_enable();
        }

        if (err_code == NRF_SUCCESS)
        {
            err_code = app_timer_create(&m_bsp_button_tmr,
                                        APP_TIMER_MODE_SINGLE_SHOT,
                                        button_timer_handler);
        }
    }
#elif (BUTTONS_NUMBER > 0) && (defined BSP_SIMPLE)
    bsp_board_init(type);
#endif // (BUTTONS_NUMBER > 0) && !(defined BSP_SIMPLE)
#if LIGHTSOUT
    NRF_LOG_INFO("Lights out mode! No LED indications!");
#endif
#if LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
    if (type & BSP_INIT_LEDS)
    {
      //handle LEDs only. Buttons are already handled.
      bsp_board_init(BSP_INIT_LEDS);

      // timers module must be already initialized!
      if (err_code == NRF_SUCCESS)
      {
          err_code =
              app_timer_create(&m_bsp_leds_tmr, APP_TIMER_MODE_SINGLE_SHOT, leds_timer_handler);
      }

      if (err_code == NRF_SUCCESS)
      {
          err_code =
              app_timer_create(&m_bsp_alert_tmr, APP_TIMER_MODE_REPEATED, alert_timer_handler);
      }
    }
#endif // LEDS_NUMBER > 0 && !(defined BSP_SIMPLE)
    
    
    return err_code;
}


#ifndef BSP_SIMPLE
/**@brief Assign specific event to button.
 */
uint32_t bsp_event_to_button_action_assign(uint32_t button, bsp_button_action_t action, bsp_event_t event)
{
    uint32_t err_code = NRF_SUCCESS;

#if BUTTONS_NUMBER > 0
    if (button < BUTTONS_NUMBER)
    {
        if (event == BSP_EVENT_DEFAULT)
        {
            // Setting default action: BSP_EVENT_KEY_x for PUSH actions, BSP_EVENT_NOTHING for RELEASE and LONG_PUSH actions.
            event = (action == BSP_BUTTON_ACTION_PUSH) ? (bsp_event_t)(BSP_EVENT_KEY_0 + button) : BSP_EVENT_NOTHING;
        }
        switch (action)
        {
            case BSP_BUTTON_ACTION_PUSH:
                m_events_list[button].push_event = event;
                break;
            case BSP_BUTTON_ACTION_LONG_PUSH:
                m_events_list[button].long_push_event = event;
                break;
            case BSP_BUTTON_ACTION_RELEASE:
                m_events_list[button].release_event = event;
                break;
            default:
                err_code = NRF_ERROR_INVALID_PARAM;
                break;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_PARAM;
    }
#else
    err_code = NRF_ERROR_INVALID_PARAM;
#endif // BUTTONS_NUMBER > 0

    return err_code;
}

#endif // BSP_SIMPLE


uint32_t bsp_buttons_enable()
{
#if (BUTTONS_NUMBER > 0) && !defined(BSP_SIMPLE)
    return app_button_enable();
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}

uint32_t bsp_buttons_disable()
{
#if (BUTTONS_NUMBER > 0) && !defined(BSP_SIMPLE)
    return app_button_disable();
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}
static uint32_t wakeup_button_cfg(uint32_t button_idx, bool enable)
{
#if !defined(BSP_SIMPLE)
    if (button_idx <  BUTTONS_NUMBER)
    {
        nrf_gpio_pin_sense_t sense = enable ?
                         (BUTTONS_ACTIVE_STATE ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW) :
                         NRF_GPIO_PIN_NOSENSE;
        nrf_gpio_cfg_sense_set(bsp_board_button_idx_to_pin(button_idx), sense);
        return NRF_SUCCESS;
    }
#else
    UNUSED_PARAMETER(button_idx);
    UNUSED_PARAMETER(enable);
#endif
    return NRF_ERROR_NOT_SUPPORTED;

}
uint32_t bsp_wakeup_button_enable(uint32_t button_idx)
{
    return wakeup_button_cfg(button_idx, true);
}

uint32_t bsp_wakeup_button_disable(uint32_t button_idx)
{
    return wakeup_button_cfg(button_idx, false);
}
