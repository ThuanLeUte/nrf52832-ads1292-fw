#include "max30205.h"

/* TWI instance. */
static const nrf_drv_twi_t	m_twi_max_30205 = NRF_DRV_TWI_INSTANCE(1);

MAX30205Variables MAX30205; 

void max30205_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//	MAX30205_Process();
}

void MAX30205_Process(void)
{
	uint16_t temp = 0;

	temp = MAX30205_ReadWord(MAX30205_TEMP);

	MAX30205.Temperature = (int16_t)(temp*0.0390625);
	
//	MAX30205.TxBuff[0] = 0xFF;
//	MAX30205.TxBuff[1] = 0xFF;
//	MAX30205.TxBuff[2] = (uint8_t)(MAX30205.Temperature>>8);
//	MAX30205.TxBuff[3] = (uint8_t)MAX30205.Temperature;
	
//	for(uint8_t n=0; n<4; n++){
//		while(app_uart_put(MAX30205.TxBuff[n]) != NRF_SUCCESS);
//	}	
}

void MAX30205_Init(void)
{
  uint8_t Reg = 0;
  
#if defined(MAX30205_POLLING_MODE)

  Reg = MAX30205_MASK_CONFIG_SHUTDOWN;
  MAX30205_WriteByte(MAX30205_CONFIG, Reg);

#elif defined(MAX30205_INTERRUPT_MODE)
  
  Reg = MAX30205_MASK_CONFIG_INT | MAX30205_MASK_CONFIG_SHUTDOWN;
  MAX30205_WriteByte(MAX30205_CONFIG, Reg);

//  MAX30205_WriteWord(MAX30205_THYST, 0x0A00);
//  MAX30205_WriteWord(MAX30205_TOS, 0x2800);

  MAX30205_INT_Init();
  Enable_MAX30205_INTO();
  
#endif  
}

void MAX30205_OneShot(void)
{
	uint8_t n;

	n = MAX30205_ReadByte(MAX30205_CONFIG);
	MAX30205_WriteByte(MAX30205_CONFIG, (n | MAX30205_MASK_CONFIG_ONESHOT) );
}

void MAX30205_Shutdown(void)
{
	uint8_t n;

	n = MAX30205_ReadByte(MAX30205_CONFIG);
	MAX30205_WriteByte(MAX30205_CONFIG, (n | MAX30205_MASK_CONFIG_SHUTDOWN) );
}

void MAX30205_Active(void)
{
	uint8_t n;

	n = MAX30205_ReadByte(MAX30205_CONFIG);
	MAX30205_WriteByte(MAX30205_CONFIG, n & (~MAX30205_MASK_CONFIG_SHUTDOWN) );
}


ret_code_t MAX30205_ReadBytes(uint8_t Register, uint8_t Length, uint8_t *MAX30205_Rx_Buffer)
{   
	ret_code_t err_code;

	do
	{
	   uint8_t addr8 = (uint8_t)Register;
	   err_code = nrf_drv_twi_tx(&m_twi_max_30205, MAX30205_ADDR, &addr8, 1, true);
	   if(NRF_SUCCESS != err_code)
	   {
		   break;
	   }
	   err_code = nrf_drv_twi_rx(&m_twi_max_30205, MAX30205_ADDR, MAX30205_Rx_Buffer, Length);
	   APP_ERROR_CHECK(err_code);
	}while(0);

	return err_code;
}

ret_code_t MAX30205_WriteBytes(uint8_t Register, uint8_t Length, uint8_t *MAX30205_Tx_Buffer)
{   
	ret_code_t err_code;
	
	do
    {
		uint8_t buffer[1 + Length]; /* Addr + data */
        buffer[0] = (uint8_t)Register;
        memcpy(buffer+1, MAX30205_Tx_Buffer, Length);
        err_code = nrf_drv_twi_tx(&m_twi_max_30205, MAX30205_ADDR, buffer, Length+1, false);
		APP_ERROR_CHECK(err_code);
    }while(0);

	return err_code;
}

uint16_t MAX30205_ReadWord(uint8_t Register)
{   
	ret_code_t err_code;
	uint8_t MAX30205_Rx_Buffer[2] ={0,0};
	uint16_t tmp = 0;   
	
	do
    {
       uint8_t addr8 = (uint8_t)Register;
       err_code = nrf_drv_twi_tx(&m_twi_max_30205, MAX30205_ADDR, &addr8, 1, true);
       if(NRF_SUCCESS != err_code)
       {
           break;
       }
       err_code = nrf_drv_twi_rx(&m_twi_max_30205, MAX30205_ADDR, MAX30205_Rx_Buffer, 2);
	   APP_ERROR_CHECK(err_code);
    }while(0);

	/*!< Store MMA955x_I2C received data */
	tmp = (uint16_t)(MAX30205_Rx_Buffer[0] << 8);
	tmp |= MAX30205_Rx_Buffer[1];

	/* return a Reg value */
	return (uint16_t)tmp;  
}

ret_code_t MAX30205_WriteWord(uint8_t Register, uint16_t RegValue)
{ 
	ret_code_t err_code;
	uint8_t MAX30205_Tx_Buffer[2] ={0,0};

	MAX30205_Tx_Buffer[0] = (uint8_t)(RegValue >> 8);
	MAX30205_Tx_Buffer[1] = (uint8_t)(RegValue);
  
	do
    {
		uint8_t buffer[3]; /* Addr + data */
        buffer[0] = (uint8_t)Register;
        memcpy(buffer+1, MAX30205_Tx_Buffer, 2);
        err_code = nrf_drv_twi_tx(&m_twi_max_30205, MAX30205_ADDR, buffer, 3, false);
		APP_ERROR_CHECK(err_code);
    }while(0);
  
	return err_code;
}

uint8_t MAX30205_ReadByte(uint8_t Register)
{   
	ret_code_t err_code;
	uint8_t MAX30205_Rx_Buffer = 0; 
  
	do
    {
       uint8_t addr8 = (uint8_t)Register;
       err_code = nrf_drv_twi_tx(&m_twi_max_30205, MAX30205_ADDR, &addr8, 1, true); 
       if(NRF_SUCCESS != err_code)
       {
           break;
       }
       err_code = nrf_drv_twi_rx(&m_twi_max_30205, MAX30205_ADDR, &MAX30205_Rx_Buffer, 1);
	   APP_ERROR_CHECK(err_code);
    }while(0);
  
	/*!< Return Register value */
	return (uint8_t)MAX30205_Rx_Buffer;
}

ret_code_t MAX30205_WriteByte(uint8_t Register, uint8_t RegValue)
{   
	ret_code_t err_code;
	uint8_t MAX30205_Tx_Buffer = RegValue;
  
	do
    {
		uint8_t buffer[2]; /* Addr + data */
        buffer[0] = (uint8_t)Register;
        memcpy(buffer+1, &MAX30205_Tx_Buffer, 1);
        err_code = nrf_drv_twi_tx(&m_twi_max_30205, MAX30205_ADDR, buffer, 2, false);
		APP_ERROR_CHECK(err_code);
    }while(0);
    
	return err_code;  
}

void MAX30205_INT_Init(void)
{
	ret_code_t	err_code;
			
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	in_config.pull = NRF_GPIO_PIN_PULLUP;

	err_code = nrf_drv_gpiote_in_init(MAX30205_INTO_PIN, &in_config, max30205_in_pin_handler);	

	APP_ERROR_CHECK(err_code);
}

void Enable_MAX30205_INTO(void)
{
	nrf_drv_gpiote_in_event_enable(MAX30205_INTO_PIN, true);
}

void Disable_MAX30205_INTO(void)
{
	nrf_drv_gpiote_in_event_disable(MAX30205_INTO_PIN);
}
