#ifndef __TIMER_H
#define __TIMER_H

#include "main.h"

// Interval : 10ms
#define T4_10MS_TICKS_PER_SEC	    100			

#define T4_10MS_COUNT_1SEC	        T4_10MS_TICKS_PER_SEC
#define T4_10MS_COUNT_01SEC	        (T4_10MS_TICKS_PER_SEC / 10)
#define T4_10MS_COUNT_02SEC	        (T4_10MS_TICKS_PER_SEC / 5)
#define T4_10MS_COUNT_04SEC	        (T4_10MS_TICKS_PER_SEC * 2 / 5)
#define T4_10MS_COUNT_2SEC	        (T4_10MS_TICKS_PER_SEC * 2)
#define T4_10MS_COUNT_3SEC	        (T4_10MS_TICKS_PER_SEC * 3)
#define T4_10MS_COUNT_4SEC	        (T4_10MS_TICKS_PER_SEC * 4)
#define T4_10MS_COUNT_5SEC	        (T4_10MS_TICKS_PER_SEC * 5)
#define T4_10MS_COUNT_10SEC	        (T4_10MS_TICKS_PER_SEC * 10)
#define T4_10MS_COUNT_20SEC	        (T4_10MS_TICKS_PER_SEC * 20)
#define T4_10MS_COUNT_30SEC	        (T4_10MS_TICKS_PER_SEC * 30)

#define T4_10MS_COUNT_1MIN	        (T4_10MS_TICKS_PER_SEC * 60)
#define T4_10MS_COUNT_2MIN	        (T4_10MS_COUNT_1MIN * 2)
#define T4_10MS_COUNT_3MIN	        (T4_10MS_COUNT_1MIN * 3)
#define T4_10MS_COUNT_5MIN	        (T4_10MS_COUNT_1MIN * 5)
#define T4_10MS_COUNT_10MIN	        (T4_10MS_COUNT_1MIN * 10)
#define T4_10MS_COUNT_20MIN	        (T4_10MS_COUNT_1MIN * 20)
#define T4_10MS_COUNT_30MIN	        (T4_10MS_COUNT_1MIN * 30)
#define T4_10MS_COUNT_60MIN	        (T4_10MS_COUNT_1MIN * 60)

//#define T4_COUNT_10Hz		T4_COUNT_01SEC	

// Interval : 100ms
#define T4_100MS_TICKS_PER_SEC	    10			

#define T4_100MS_COUNT_1SEC	        T4_100MS_TICKS_PER_SEC
#define T4_100MS_COUNT_01SEC	   	(T4_100MS_TICKS_PER_SEC / 10)
#define T4_100MS_COUNT_02SEC	    (T4_100MS_TICKS_PER_SEC / 5)
#define T4_100MS_COUNT_04SEC	    (T4_100MS_TICKS_PER_SEC * 2 / 5)
#define T4_100MS_COUNT_2SEC	        (T4_100MS_TICKS_PER_SEC * 2)
#define T4_100MS_COUNT_3SEC	        (T4_100MS_TICKS_PER_SEC * 3)
#define T4_100MS_COUNT_5SEC	        (T4_100MS_TICKS_PER_SEC * 5)
#define T4_100MS_COUNT_10SEC	    (T4_100MS_TICKS_PER_SEC * 10)
#define T4_100MS_COUNT_20SEC	    (T4_100MS_TICKS_PER_SEC * 20)

#define T4_100MS_COUNT_1MIN	        (T4_100MS_TICKS_PER_SEC * 60)
#define T4_100MS_COUNT_2MIN	        (T4_100MS_COUNT_1MIN * 2)
#define T4_100MS_COUNT_3MIN	        (T4_100MS_COUNT_1MIN * 3)
#define T4_100MS_COUNT_5MIN	        (T4_100MS_COUNT_1MIN * 5)
#define T4_100MS_COUNT_10MIN	    (T4_100MS_COUNT_1MIN * 10)
#define T4_100MS_COUNT_20MIN	    (T4_100MS_COUNT_1MIN * 20)
#define T4_100MS_COUNT_30MIN	    (T4_100MS_COUNT_1MIN * 30)
#define T4_100MS_COUNT_60MIN	    (T4_100MS_COUNT_1MIN * 60)
	
/* Private function prototypes -----------------------------------------------*/

void timer1_event_handler(nrf_timer_event_t event_type, void* p_context);
void timer1_init(void);
void timer4_event_handler(nrf_timer_event_t event_type, void* p_context);
void timer4_init(void);

#endif 
