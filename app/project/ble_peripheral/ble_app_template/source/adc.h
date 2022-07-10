#ifndef __ADC_H
#define __ADC_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct _BatteryVariables
{ 
    uint8_t DetFlag;
    uint8_t LowFlag;
    uint8_t ChargingFlag;
    uint8_t Percent;
    uint8_t PercentFlag;
    uint16_t Voltage;
    uint16_t Median_Voltage;
    uint16_t FirstTime_Voltage;
    uint16_t LastTime_Voltage;
	
    uint16_t AvgCnt;
	uint32_t SumADValue;
	
} BatteryVariables;

void timer_handler(nrf_timer_event_t event_type, void * p_context);
void saadc_sampling_event_init(void);
void saadc_sampling_event_enable(void);
void saadc_callback(nrf_drv_saadc_evt_t const * p_event);
void saadc_init(void);
#endif 
