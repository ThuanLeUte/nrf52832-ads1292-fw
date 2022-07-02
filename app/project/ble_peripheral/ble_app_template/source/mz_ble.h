#ifndef __MZ_BLE_H
#define __MZ_BLE_H

#include "main.h"   
                                     
/* BLE Header */
#define BLE_BATTERY_HEADER          0xA1

#define BLE_TX_LENGTH        		168     


typedef struct _BLEVariables
{ 
	uint8_t TxFlag;
    uint8_t TxBuff[BLE_TX_LENGTH];	
	uint8_t TxTempBuff[BLE_TX_LENGTH];	
    uint8_t TxSuccessFlag;
	uint8_t ConnectedFlag;	
    uint8_t PowerOffFlag;
    uint8_t PacketIdx;
	uint16_t TX_LENGTH;
} BLEVariables;

void BLE_DataParsing(void);

#endif
