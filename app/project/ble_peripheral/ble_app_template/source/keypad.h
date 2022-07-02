#ifndef __KEYPAD_H
#define __KEYPAD_H

#include "main.h"

#define SW1_PIN					13	/* INPUT */
#define SW2_PIN					14	/* INPUT */

#define NONE_KEY                0x00
#define ONE_KEY                 0x01
#define TWO_KEY                 0x02

#define D_KEY		        	0x01    /* 0b00000001 */
#define U_KEY		        	0x04    /* 0b00000100 */

#define LONG_KEY                0x80    /* 0b10000000 */

typedef struct _KeyStatus
{ 
    uint8_t KeyFlag;
	uint8_t KeyValue;
	uint8_t KeyState;

	uint8_t U_ShortKey;
	uint8_t D_ShortKey;

	uint8_t U_LongKey;
	uint8_t D_LongKey;

	uint16_t U_KeyCount;
	uint16_t D_KeyCount;
} KeyStatus;;

/* Private function prototypes -----------------------------------------------*/
void KeyPad_Init(void);
void KeyPad_Enable(void);
void KeyPad_Disable(void);
void Check_Key(void);
uint8_t AnyKeyIn(void);
uint8_t GetCode(void);

#endif 
