#include "timer.h"

extern SystemVariables System;

const nrf_drv_timer_t TIMER1 = NRF_DRV_TIMER_INSTANCE(1);
const nrf_drv_timer_t TIMER4 = NRF_DRV_TIMER_INSTANCE(4);

volatile uint16_t Timer4_Cnt = 0;
//volatile uint8_t  Timer4_10Hz_Flag = 0;

/**
 * @brief Handler for timer events.
 */
void timer1_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
//            nrf_gpio_pin_toggle(LED2);        
            break;

        default:
            //Do nothing.
            break;
    }
}

void timer1_init(void)
{
	uint32_t time_ms = 40; /* Time(in miliseconds) between consecutive compare events. */
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
	
	/* Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other. */
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.interrupt_priority = 5;
    err_code = nrf_drv_timer_init(&TIMER1, &timer_cfg, timer1_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER1, time_ms);

    nrf_drv_timer_extended_compare(
         &TIMER1, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER1);
}

/**
 * @brief Handler for timer events.
 */
void timer4_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            MPU9250_OneStepProcess();
        
            Timer4_Cnt++;
            if(Timer4_Cnt >= 10) 
            {
                nrf_gpio_pin_toggle(LED2); 
                BLE_DataParsing();
                Timer4_Cnt = 0;
            }
            
			Check_Key();           
            break;

        default:
            //Do nothing.
            break;
    }
}

void timer4_init(void)
{
	uint32_t time_ms = 10; /* Time(in miliseconds) between consecutive compare events. */
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
	
	/* Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other. */
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.interrupt_priority = 6;
    err_code = nrf_drv_timer_init(&TIMER4, &timer_cfg, timer4_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER4, time_ms);

    nrf_drv_timer_extended_compare(
         &TIMER4, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER4);
}
