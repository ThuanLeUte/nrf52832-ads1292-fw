#ifndef __SERIALFLASH_H
#define __SERIALFLASH_H

#include "main.h"
#include <stdlib.h>

typedef struct _SerialFlashVariables
{ 
	uint8_t ManufacturerID;
	uint8_t MemoryType;
	uint8_t MemoryDensity;
	uint8_t DeviceID;
	uint8_t SW;
	uint8_t TxBuf;
	uint8_t RxBuf;
	uint8_t buffer[15];
	uint16_t data_256[256];	        /* global array to store read data */
	uint16_t block_protection_10[10];	/* global array to store block_protection data */
	uint32_t DeviceAddress;
} SerialFlashVariables;

/** 
  * @brief  FLASH SPI Interface pins 
  */ 
     
#define FLASH_SPI_MOSI_PIN					12	/* OUTPUT */
#define FLASH_SPI_SCK_PIN					10	/* OUTPUT */
#define FLASH_SPI_MISO_PIN					11	/* INPUT */
#define FLASH_CE_PIN                		9   /* OUTPUT */


//#define ManufacturerID_ADD      0x00
//#define DeviceID_ADD            0x01        

#define FLASH_BLOCK_SIZE		        0x10000		/* 64KB */ 
#define FLASH_SECTOR_SIZE		        0x1000		/* 4KB  */
#define FLASH_PAGE_SIZE		        	0x100		/* 256B */ 

#define BLOCK_ADD(N)                    ((uint32_t)(N*0x00010000)) 
#define SECTOR_ADD(N)                   ((uint32_t)(N*0x00001000)) 
#define PAGE_ADD(N)                     ((uint32_t)(N*0x00000100)) 

#define FLASH_SYSTEM_INFO_ADD           0x00000000      /* System Information Address = Sector 0 (0x000000000 - 0x000000FFF) */
    #define FLASH_SYSTEM_DEVICE_STR_BYTE            0x00001000 
    #define FLASH_SYSTEM_DEVICE_NUMBER_HBYTE        0x00001001 
    #define FLASH_SYSTEM_DEVICE_NUMBER_LBYTE        0x00001002
    
    #define FLASH_SYSTEM_HW_VERSION_HBYTE           0x00001004
    #define FLASH_SYSTEM_HW_VERSION_LBYTE           0x00001005
    
    #define FLASH_SYSTEM_SW_VERSION_HBYTE           0x00001007
    #define FLASH_SYSTEM_SW_VERSION_LBYTE           0x00001008
    
    #define FLASH_SYSTEM_FW_YEAR_HBYTE              0x00001009
    #define FLASH_SYSTEM_FW_MONTH_HBYTE             0x0000100A
    #define FLASH_SYSTEM_FW_DATE_LBYTE              0x0000100B
    

/* Pin Function Prototypes */

void SerialFlash_SetSIO0_Output(void);
void SerialFlash_SetSIO0_Input(void);
void SerialFlash_SetSIO1_Output(void);
void SerialFlash_SetSIO1_Input(void);
void SerialFlash_SIO0_Low(void);
void SerialFlash_SIO1_Low(void);
void SerialFlash_SIO0_High(void);
void SerialFlash_SIO1_High(void);
void SerialFlash_SCK_High(void);
void SerialFlash_SCK_Low(void);
void SerialFlash_CE_High(void);
void SerialFlash_CE_Low(void);


/* Function Prototypes */

void SerialFlash_Init(void);


void SerialFlash_Send_Byte(uint8_t out);
void SerialFlash_Send_Double_Byte(uint8_t out);
uint8_t SerialFlash_Get_Byte(void);
uint8_t SerialFlash_Get_Double_Byte(void);

void SerialFlash_Double_NoOp(void);
void SerialFlash_SPI_NoOp(void);

uint8_t SerialFlash_SPI_Read_Status_Register(void);
uint16_t SerialFlash_SPI_Read_Configuration_Register(void);
void SerialFlash_SPI_Write_Status_Register(uint8_t Status_Reg, uint8_t Config_Reg_1, uint8_t Config_Reg_2);
void SerialFlash_SPI_WREN(void);
void SerialFlash_SPI_WRDI(void);
void SerialFlash_SPI_Wait_Busy(void);
void SerialFlash_Jedec_ID_Read(void);
uint8_t SerialFlash_SPI_Read(uint32_t Dst);
void SerialFlash_Read_Cont(uint32_t Dst, uint32_t no_bytes);
//uint8_t SerialFlash_HighSpeed_Read(uint32_t Dst);
uint8_t SerialFlash_SPI_HighSpeed_Read(uint32_t Dst);
//void SerialFlash_HighSpeed_Read_Cont(uint32_t Dst, uint32_t no_bytes);
void SerialFlash_SPI_HighSpeed_Read_Cont(uint32_t Dst, uint32_t no_bytes);
//void SerialFlash_HighSpeed_NotInMode_Read_Cont(uint32_t Dst, uint32_t no_bytes);
//void SerialFlash_HighSpeed_InMode_Read_Cont(uint32_t Dst, uint32_t no_bytes);
//uint8_t SerialFlash_HighSpeed_InMode_Read(uint32_t Dst);
//uint8_t SerialFlash_HighSpeed_NotInMode_Read(uint32_t Dst);

//void SerialFlash_ResetEn(void);
//void SerialFlash_SPI_ResetEn(void);
//void SerialFlash_Reset(void);
//void SerialFlash_SPI_Reset(void);
//void SerialFlash_En_QIO(void);
//void SerialFlash_Reset_QIO(void);


//void SerialFlash_Set_Burst(uint8_t byte);
//void SerialFlash_Read_Burst(uint32_t Dst, uint8_t burstlength);
//void SerialFlash_Read_PI(uint8_t Dst, uint8_t datalength);
//void SerialFlash_Read_Index(uint32_t Dst, uint8_t datalength);
//void SerialFlash_Read_BlockIndex(uint8_t Dst, uint8_t datalength);

//void SerialFlash_SPI_Quad_IO_InMode_Read_Cont(uint32_t Dst, uint32_t no_bytes);
//uint8_t SerialFlash_SPI_Quad_IO_InMode_Read(uint32_t Dst);
//uint8_t SerialFlash_SPI_Quad_IO_NotInMode_Read(uint32_t Dst);
//void SerialFlash_SPI_Quad_IO_NotInMode_Read_Cont(uint32_t Dst, uint32_t no_bytes);
//uint8_t SerialFlash_SPI_Double_IO_InMode_Read(uint32_t Dst);
//uint8_t SerialFlash_SPI_Double_IO_NotInMode_Read(uint32_t Dst);
//void SerialFlash_SPI_Double_IO_NotInMode_Read_Cont(uint32_t Dst, uint32_t no_bytes);
//void SerialFlash_SPI_Double_IO_InMode_Read_Cont(uint32_t Dst, uint32_t no_bytes);

void SerialFlash_SPI_Sector_Erase(uint32_t Dst);
void SerialFlash_SPI_Block32k_Erase(uint32_t Dst);
void SerialFlash_SPI_Block_Erase(uint32_t Dst);
void SerialFlash_SPI_Chip_Erase(void);
//void SerialFlash_Page_Program(uint32_t Dst);
void SerialFlash_SPI_Page_Program(uint32_t Dst);
//void SerialFlash_Byte_Program(uint32_t Dst, uint8_t byte);
void SerialFlash_SPI_Byte_Program(uint32_t Dst, uint8_t byte);
//void SerialFlash_Write_Suspend(void);
//void SerialFlash_SPI_Write_Suspend(void);
//void SerialFlash_Write_Resume(void);
//void SerialFlash_SPI_Write_Resume(void);
//void SerialFlash_ReadSID(uint8_t *security_ID, uint32_t Dst, uint32_t security_length);
//void SerialFlash_ProgSID(uint8_t *security_ID, uint32_t Dst, uint32_t security_length);
//void SerialFlash_LockSID(void);
//void SerialFlash_ReadBlockProtection(void);
//void SerialFlash_WriteBlockProtection(void);
//void SerialFlash_SPI_WriteBlockProtection(void);
//void SerialFlash_LockBlockProtection(void);
//void SerialFlash_SPI_LockBlockProtection(void);
//void SerialFlash_Global_Block_Protection_Unlock(void);
//void SerialFlash_SPI_Global_Block_Protection_Unlock(void);


//void SerialFlash_Test(void);

#endif
