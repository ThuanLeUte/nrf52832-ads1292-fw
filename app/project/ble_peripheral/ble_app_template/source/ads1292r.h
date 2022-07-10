/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADS1292R_H
#define __ADS1292R_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"   

/**
 * @brief Uncomment the line below if you want to use user defined Delay function
 *        (for precise timing), otherwise default _delay_ function defined within
 *         this driver is used (less precise timing).  
 */                                 
 
 
 /** 
  * @brief  ADS1292R Control pins  
  */ 
  
#define ADS1292R_CS_PIN        	31  	/* OUTPUT */
#define ADS1292R_RESET_PIN    	19  	/* OUTPUT */
#define ADS1292R_START_PIN   	30 		/* OUTPUT */
#define ADS1292R_BUSY_PIN      	20     	/* INPUT */

#define ADS1292R_WAKEUP			0x02
#define ADS1292R_STANDBY		0x04
#define ADS1292R_RESET			0x06
#define ADS1292R_START			0x08
#define ADS1292R_STOP			0x0A
#define ADS1292R_OFFSETCAL    	0x1A                                      
#define ADS1292R_RDATAC			0x10
#define ADS1292R_SDATAC			0x11
#define ADS1292R_RDATA			0x12

#define ADS1292R_DUMMY_BYTE  	0xFF

typedef struct _ADS1292RVariables
{ 
    uint8_t StartFlag;
	uint8_t ReadBuffer[9];
	uint8_t WriteBuffer[15];
    uint8_t TxBuff[100];
    uint8_t TxTempBuff[100];
    
    uint8_t LeadFailFlag;
    int32_t RawECG;
	int32_t RawResp;
	uint8_t Status;
    int32_t FilteredECG;
    int16_t DispECG;
    uint8_t nSampleCnt;
} ADS1292RVariables;

/** @defgroup STM320518_EVAL_LCD_Exported_Functions
  * @{
  */ 

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

void ADS1292R_Process(void);
int32_t IIR_LowPassFilter(int32_t input);
int32_t IIR_HighPassFilter(int32_t input);

void ADS1292R_DeInit(void);
void ADS1292R_Init(void);
void ADS1292R_CtrlLinesConfig(void);
void ADS1292R_SPIConfig(void);
uint8_t ADS1292R_ReadWriteByte(uint8_t tx_data);
void ADS1292R_SPI_ReadWriteArray(uint8_t * p_tx_data, uint8_t * p_rx_data, uint8_t  len);
void ADS1292R_HWPowerDown(void);
void ADS1292R_HWPowerUp(void);
void ADS1292R_EnterStanbyMode(void);
void ADS1292R_WakeUp(void);

#endif 
