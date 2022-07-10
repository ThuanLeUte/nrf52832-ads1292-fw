#ifndef __MAX30205_H
#define __MAX30205_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef enum
{
  MAX30205_OK = 0,
  MAX30205_FAIL
}MAX30205_Status_TypDef;

typedef struct _MAX30205Variables
{ 
	uint8_t INT_Flag;
	uint8_t TxBuff[4];
	int16_t Temperature;
	float fTemperature;
} MAX30205Variables;

/***********************************************************************************************\
* Public type definitions
\***********************************************************************************************/
#define MAX30205_INTO_PIN      	16

#define MAX30205_ADDR                           (0x90 >> 1)

#define MAX30205_POLLING_MODE
//#define MAX30205_INTERRUPT_MODE

#define MAX30205_TEMP                           0x00

#define MAX30205_CONFIG                         0x01
  #define MAX30205_MASK_CONFIG_ONESHOT          BIT(7)
  #define MAX30205_MASK_CONFIG_TIMEOUT          BIT(6)
  #define MAX30205_MASK_CONFIG_DATA_FORMAT      BIT(5)
  #define MAX30205_MASK_CONFIG_FAULT_QUEUE1     BIT(4)
  #define MAX30205_MASK_CONFIG_FAULT_QUEUE0     BIT(3)
  #define MAX30205_MASK_CONFIG_OS_POLARITY      BIT(2)
  #define MAX30205_MASK_CONFIG_INT              BIT(1)
  #define MAX30205_MASK_CONFIG_SHUTDOWN         BIT(0)

#define MAX30205_THYST                          0x02

#define MAX30205_TOS                            0x03

void max30205_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

void MAX30205_Process(void);
void MAX30205_Init(void);
void MAX30205_OneShot(void);
void MAX30205_Shutdown(void);
void MAX30205_Active(void);

ret_code_t MAX30205_ReadBytes(uint8_t Register, uint8_t Length, uint8_t *MAX30205_Rx_Buffer);//
ret_code_t MAX30205_WriteBytes(uint8_t Register, uint8_t Length, uint8_t *MAX30205_Tx_Buffer);
uint16_t MAX30205_ReadWord(uint8_t Register);
ret_code_t MAX30205_WriteWord(uint8_t Register, uint16_t RegValue);
uint8_t MAX30205_ReadByte(uint8_t Register);//
ret_code_t MAX30205_WriteByte(uint8_t Register, uint8_t RegValue);//

void MAX30205_INT_Init(void);
void Enable_MAX30205_INTO(void);
void Disable_MAX30205_INTO(void);


#endif  
