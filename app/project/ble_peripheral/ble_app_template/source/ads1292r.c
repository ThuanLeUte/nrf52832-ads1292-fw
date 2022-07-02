/* Includes ------------------------------------------------------------------*/
#include "ads1292r.h"
#include <math.h>

/** @defgroup Private_Function_Prototypes
  * @{
  */

/* Private variables ---------------------------------------------------------*/
ADS1292RVariables ADS1292R;

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

void ads1292r_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	nrf_gpio_pin_clear(ADS1292R_CS_PIN);
	ADS1292R_SPI_ReadWriteArray(ADS1292R.WriteBuffer, ADS1292R.ReadBuffer, 9);
	nrf_gpio_pin_set(ADS1292R_CS_PIN);

	ADS1292R_Process();
}


void ADS1292R_Process(void)
{
  	/* (24 status bits + 24 bits × 2 channels) = 72 bits */
  	/* 24 status bits is: (1100 + LOFF_STAT[4:0] + GPIO[1:0] + 13 '0's) */
  	int32_t temp = 0;
	
  	temp = (ADS1292R.ReadBuffer[6] << 16) + (ADS1292R.ReadBuffer[7] << 8) + ADS1292R.ReadBuffer[8]; 
    ADS1292R.RawECG =  TwosComplementConverterInt32(temp, 24);
	
	temp = (ADS1292R.ReadBuffer[3] << 16) + (ADS1292R.ReadBuffer[4] << 8) + ADS1292R.ReadBuffer[5]; 
	ADS1292R.RawResp = TwosComplementConverterInt32(temp, 24);
	
	temp = (ADS1292R.ReadBuffer[0] << 16) + (ADS1292R.ReadBuffer[1] << 8) + ADS1292R.ReadBuffer[2]; 
	ADS1292R.Status = (uint8_t)(temp >> 15);
    
    temp = IIR_LowPassFilter(ADS1292R.RawECG);
    ADS1292R.FilteredECG = IIR_HighPassFilter(temp);
    
    if( ADS1292R.FilteredECG < -32000 ) ADS1292R.FilteredECG = -32000;
    else if( ADS1292R.FilteredECG > 32000 ) ADS1292R.FilteredECG = 32000;
    
    if( ADS1292R.Status == 0x80)
	{
	  	ADS1292R.LeadFailFlag = 0;
        ADS1292R.DispECG = (uint16_t)ADS1292R.FilteredECG;
//		nrf_gpio_pin_clear(LED2);
	}
	else
	{
	  	ADS1292R.LeadFailFlag = 1;
        ADS1292R.DispECG = 0;
//		nrf_gpio_pin_set(LED2);
	}
    
    ADS1292R.nSampleCnt++;
    
    ADS1292R.TxTempBuff[(ADS1292R.nSampleCnt-1)*2]   = (uint8_t)( (ADS1292R.DispECG >> 8) & 0xFF);
    ADS1292R.TxTempBuff[(ADS1292R.nSampleCnt-1)*2+1] = (uint8_t)( (ADS1292R.DispECG & 0xFF) );
    
    if( ADS1292R.nSampleCnt >= 50 ) 
    {
        ADS1292R.nSampleCnt = 0;
        memcpy(ADS1292R.TxBuff, ADS1292R.TxTempBuff, 100);
    }
}

/* fs = 500Hz, [b1, a1] = butter(1, 0.5/fs/2, 'high') */
const float IIR_HPF_CoeffB[2] = {0.999215218041547f,	-0.999215218041547f};
const float IIR_HPF_CoeffA[2] = {1.0f,	-0.998430436083094f};

int32_t IIR_HighPassFilter(int32_t input)
{   
    static float iir_inputbuffer;
	static float iir_outputbuffer;

	iir_outputbuffer = IIR_HPF_CoeffB[0]*(float)input + IIR_HPF_CoeffB[1]*iir_inputbuffer - IIR_HPF_CoeffA[1]*iir_outputbuffer;

	iir_inputbuffer = (float)input;

	return (int32_t)iir_outputbuffer;
}

/* fs = 500Hz, [b2, a2] = butter(1, 250/fs/2, 'low') */
const float IIR_LPF_CoeffB[2] = {0.292893218813452f,	0.292893218813452f};
const float IIR_LPF_CoeffA[2] = {1.0,	-0.414213562373095};

int32_t IIR_LowPassFilter(int32_t input)
{
	static float iir_inputbuffer;
	static float iir_outputbuffer;

	iir_outputbuffer = IIR_LPF_CoeffB[0]*(float)input + IIR_LPF_CoeffB[1]*iir_inputbuffer - IIR_LPF_CoeffA[1]*iir_outputbuffer;

	iir_inputbuffer = (float)input;

	return (int32_t)iir_outputbuffer;
}	



/**
  * @brief  DeInitializes the ADS1292R.
  * @param  None
  * @retval None
  */
void ADS1292R_DeInit(void)
{

}

/**
  * @brief  Initializes the ADS1292R.
  * @param  None
  * @retval None
  */
void ADS1292R_Init(void)
{
	uint8_t m_tx_buf[13] = {
        0x41,       // Write to CONFIG1~ Resister
        0x0A,       // number of registers to be written --> 11-1
        0x02,       // Address = 01h // CONFIG1
        0xF0,       // Address = 02h // CONFIG2  --> No! test signal!
        0x34,       // Address = 03h // LOFF     --> DC lead-off detection turned on
        0x80,       // Address = 04h // CH1SET   --> Respiration signal
        0x00,       // Address = 05h // CH2SET
        0x3C,       // Address = 06h // RLD
        0x0C,       // Address = 07h // LOFF sense
        0x43,       // Address = 08h // LOFF_status
        0x02,       // Address = 09h // Respiration control 1
        0x03,       // Address = 0Ah // Respiration control 2
        0x00        // GPIO 
    }; 
	uint8_t m_rx_buf[14];    /**< RX buffer. */
	uint16_t n;

	/* Configure the ADS1292R Control pins --------------------------------------------*/
	ADS1292R_CtrlLinesConfig();

	/* Configure the ADS1292R_SPI interface ----------------------------------------------*/
	ADS1292R_SPIConfig();

	nrf_gpio_pin_set(ADS1292R_CS_PIN);
	nrf_gpio_pin_clear(ADS1292R_START_PIN);

	nrf_gpio_pin_set(ADS1292R_RESET_PIN);
	nrf_delay_ms(100);
	nrf_gpio_pin_clear(ADS1292R_RESET_PIN);
	nrf_delay_ms(100);
	nrf_gpio_pin_set(ADS1292R_RESET_PIN);
	nrf_delay_ms(100);

	nrf_gpio_pin_clear(ADS1292R_CS_PIN);
	ADS1292R_ReadWriteByte(ADS1292R_RESET);
	nrf_delay_ms(100);
	nrf_gpio_pin_set(ADS1292R_CS_PIN);

	nrf_gpio_pin_clear(ADS1292R_CS_PIN);
	for(n=0;n<10;n++) ADS1292R_ReadWriteByte(0x00);
	nrf_gpio_pin_set(ADS1292R_CS_PIN);

	nrf_delay_ms(10);

	nrf_gpio_pin_clear(ADS1292R_CS_PIN);
	ADS1292R_ReadWriteByte(ADS1292R_SDATAC);         // Stop Read Data Continuously mode
	nrf_gpio_pin_set(ADS1292R_CS_PIN);

	nrf_delay_ms(10);  

	nrf_gpio_pin_clear(ADS1292R_CS_PIN);
	ADS1292R_SPI_ReadWriteArray(m_tx_buf, m_rx_buf, 13);
	nrf_gpio_pin_set(ADS1292R_CS_PIN);

	nrf_delay_ms(10);

	nrf_gpio_pin_clear(ADS1292R_CS_PIN);
	ADS1292R_ReadWriteByte(ADS1292R_RDATAC);  // Enable Read Data Continuous mode
	nrf_gpio_pin_set(ADS1292R_CS_PIN);

	nrf_drv_gpiote_in_event_enable(ADS1292R_BUSY_PIN, true);
}

//----------------------------------------------------------------------------------------------------

/**
  * @brief  Configures ADS1292R control lines in Output Push-Pull mode.
  * @param  None
  * @retval None
  */
void ADS1292R_CtrlLinesConfig(void)
{
	ret_code_t	err_code;

	nrf_gpio_cfg_output(ADS1292R_CS_PIN);
	nrf_gpio_cfg_output(ADS1292R_RESET_PIN);
	nrf_gpio_cfg_output(ADS1292R_START_PIN);

	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	in_config.pull = NRF_GPIO_PIN_PULLUP;
	err_code = nrf_drv_gpiote_in_init(ADS1292R_BUSY_PIN, &in_config, ads1292r_in_pin_handler);
	APP_ERROR_CHECK(err_code);
    
    NVIC_SetPriority(GPIOTE_IRQn, 3);
}

/**
  * @brief  Configures the ADS1292R_SPI interface.
  * @param  None
  * @retval None
  */
void ADS1292R_SPIConfig(void)
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

	spi_config.sck_pin  = SPIM0_SCK_PIN;
	spi_config.mosi_pin = SPIM0_MOSI_PIN;
	spi_config.miso_pin = SPIM0_MISO_PIN;
	spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
	spi_config.irq_priority = APP_IRQ_PRIORITY_LOW;
	spi_config.orc   = 0xFF;
	spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
	spi_config.mode   = NRF_DRV_SPI_MODE_1;
	spi_config.bit_order  = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
}

void ADS1292R_ReSPIConfig(void)
{
//  SPI_InitTypeDef    SPI_InitStructure;
//
//  /* SPI Config */
//  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//
//  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
//  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//  SPI_InitStructure.SPI_CRCPolynomial = 7;
//  SPI_Init(ADS1292R_SPI, &SPI_InitStructure);
}

uint8_t ADS1292R_ReadWriteByte(uint8_t tx_data)
{
	uint8_t	rx_data = 0;

	uint32_t err_code = nrf_drv_spi_transfer(&spi, (uint8_t *)&tx_data, 1, (uint8_t *)&rx_data, 1);

	APP_ERROR_CHECK(err_code);

	return rx_data;
}

void ADS1292R_SPI_ReadWriteArray(uint8_t * p_tx_data, uint8_t * p_rx_data, uint8_t  len)
{
	uint32_t err_code = nrf_drv_spi_transfer(&spi, p_tx_data, len, p_rx_data, len);

	APP_ERROR_CHECK(err_code);
}

void ADS1292R_HWPowerDown(void)
{
	nrf_gpio_pin_clear(ADS1292R_RESET_PIN);
	nrf_delay_ms(10);
}

void ADS1292R_HWPowerUp(void)
{
	nrf_gpio_pin_set(ADS1292R_RESET_PIN);
	nrf_delay_ms(10);
}

void ADS1292R_EnterStanbyMode(void)
{
	nrf_gpio_pin_clear(ADS1292R_CS_PIN);
	ADS1292R_ReadWriteByte(ADS1292R_STANDBY);
	nrf_gpio_pin_set(ADS1292R_CS_PIN);
}

void ADS1292R_WakeUp(void)
{
	nrf_gpio_pin_clear(ADS1292R_CS_PIN);
	ADS1292R_ReadWriteByte(ADS1292R_WAKEUP);
	nrf_gpio_pin_set(ADS1292R_CS_PIN);
}

