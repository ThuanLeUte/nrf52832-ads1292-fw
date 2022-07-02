#include "keypad.h"

enum state {InitialState, DebounceHLState, KeypressedState, ReleaseState, DebounceLHState};
KeyStatus Keypad;

void sw1_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

void sw2_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

void KeyPad_Init(void)
{	
  	nrf_gpio_cfg_input(SW1_PIN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(SW2_PIN, NRF_GPIO_PIN_NOPULL);
	
//	nrf_drv_gpiote_in_config_t sw1_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
//	sw1_config.pull = NRF_GPIO_PIN_NOPULL;
//	err_code = nrf_drv_gpiote_in_init(SW1_PIN, &sw1_config, sw1_in_pin_handler);	
//	APP_ERROR_CHECK(err_code);
//	nrf_drv_gpiote_in_event_enable(SW1_PIN, true);
//	
//	nrf_drv_gpiote_in_config_t sw2_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
//	sw2_config.pull = NRF_GPIO_PIN_NOPULL;
//	err_code = nrf_drv_gpiote_in_init(SW2_PIN, &sw2_config, sw2_in_pin_handler);	
//	APP_ERROR_CHECK(err_code);
//	nrf_drv_gpiote_in_event_enable(SW2_PIN, true);
  
	ClearMem((unsigned char*)&Keypad, sizeof(KeyStatus));
	Keypad.KeyState = InitialState;
}

void KeyPad_Enable(void)
{
	nrf_gpio_cfg_input(SW1_PIN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(SW2_PIN, NRF_GPIO_PIN_NOPULL);

	ClearMem((unsigned char*)&Keypad, sizeof(KeyStatus));
	Keypad.KeyState = InitialState;
}

void KeyPad_Disable(void)
{
	nrf_gpio_input_disconnect(SW1_PIN);
	nrf_gpio_input_disconnect(SW2_PIN);
	
	ClearMem((unsigned char*)&Keypad, sizeof(KeyStatus));
	Keypad.KeyState = InitialState;
}

void Check_Key(void)
{   
    
	switch(Keypad.KeyState)
	{
		case InitialState : 
			if( NONE_KEY != AnyKeyIn() )
			{
				Keypad.U_KeyCount = 0;  Keypad.D_KeyCount = 0;
				Keypad.U_ShortKey  = 0; Keypad.D_ShortKey  = 0;  
				Keypad.U_LongKey  = 0;  Keypad.D_LongKey  = 0;
				Keypad.KeyValue = 0;
				Keypad.KeyState = DebounceHLState; 
			}
			break;
		  
		case DebounceHLState: 
			Keypad.KeyState = (NONE_KEY != AnyKeyIn())? KeypressedState : InitialState; 
			break;
		  
		case KeypressedState: 
			Keypad.KeyValue = GetCode();
			Keypad.KeyState = ReleaseState;
			break;
		  
		case ReleaseState: 
			if( NONE_KEY == AnyKeyIn() ) 
			{
				if(Keypad.KeyValue == U_KEY) 
				{
					Keypad.U_ShortKey = 1;
				}
				else if(Keypad.KeyValue == D_KEY) 
				{
					Keypad.D_ShortKey = 1;
				}          

				Keypad.KeyState = DebounceLHState;
				break; 
			}   
			else if( ONE_KEY == AnyKeyIn() )
			{
				Keypad.KeyValue = GetCode();

				if( (Keypad.KeyValue == U_KEY) && (Keypad.U_LongKey == 0) ) 
				{
					Keypad.U_KeyCount++;    
					if(Keypad.U_KeyCount >= T4_10MS_COUNT_4SEC)
					{
						Keypad.KeyValue += LONG_KEY;
						Keypad.U_LongKey = 1;
					}
				}
				else if( (Keypad.KeyValue == D_KEY) && (Keypad.D_LongKey == 0) ) 
				{
					Keypad.D_KeyCount++; 
					if(Keypad.D_KeyCount >= T4_10MS_COUNT_2SEC)
					{       
						Keypad.KeyValue += LONG_KEY;
						Keypad.D_LongKey = 1;   
					}
				}

				if( (Keypad.U_LongKey | Keypad.D_LongKey) == 1 ) 
				{
					Keypad.KeyState = DebounceLHState;
					break; 
				}
			}   
		  
		case DebounceLHState: 
			if( NONE_KEY == AnyKeyIn() ) {
				Keypad.KeyState = InitialState;
				Keypad.U_KeyCount = 0; Keypad.D_KeyCount = 0;  
				Keypad.KeyValue = 0;
			}
			break;
		  
		default: 
		  break;
	}      
}  

uint8_t AnyKeyIn(void)
{
	/* Rising Edge */
	if( nrf_gpio_pin_read(SW1_PIN) == 1 ) return ONE_KEY;
	else return NONE_KEY;
}

uint8_t GetCode(void)
{
	if( nrf_gpio_pin_read(SW1_PIN) == 1 ) return U_KEY;
//	else if( nrf_gpio_pin_read(SW2_PIN) == 0 ) return D_KEY;
	else return 0;
}
