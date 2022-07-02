/* Includes ------------------------------------------------------------------*/
#include "mz_ble.h"

BLEVariables BLE;

extern UART1Variables UART1;
extern SystemVariables System;
extern ADS1292RVariables ADS1292R;
extern MPU9250Variables MPU9250;
extern BatteryVariables Battery;


void BLE_DataParsing(void)
{  
#if 1    
    uint16_t xtemp = 0;
    
    BLE.TxTempBuff[0] = 0xFF;
    BLE.TxTempBuff[1] = 0xFF;

    BLE.TxTempBuff[2] = 0;	    // Reserved
    BLE.TxTempBuff[3] = 0;      // Reserved    
    BLE.TxTempBuff[4] = 0;      // Reserved		
    
    if( BLE.PacketIdx == 0 )
    {
        BLE.TxTempBuff[5] = BLE_BATTERY_HEADER;
        xtemp = Battery.Voltage;
    }
    else if( BLE.PacketIdx == 1 )
    {
        BLE.TxTempBuff[5] = 0;
        xtemp = 0;
    }
    else if( BLE.PacketIdx == 2 )
    {
        BLE.TxTempBuff[5] = 0;
        xtemp = 0;
    }
    else if( BLE.PacketIdx == 3 )
    {
        BLE.TxTempBuff[5] = 0;
        xtemp = 0;
    }
    else if( BLE.PacketIdx == 4 )
    {
        BLE.TxTempBuff[5] = 0;
        xtemp = 0;
    }
    else if( BLE.PacketIdx == 5 )
    {
        BLE.TxTempBuff[5] = 0;
        xtemp = 0;
    }
    else if( BLE.PacketIdx == 6 )
    {
        BLE.TxTempBuff[5] = 0;
        xtemp = 0;
    }
    else if( BLE.PacketIdx == 7 )
    {
        BLE.TxTempBuff[5] = 0;
        xtemp = 0;
    }
    else if( BLE.PacketIdx == 8 )
    {
        BLE.TxTempBuff[5] = 0;
        xtemp = 0;
    }
    else if( BLE.PacketIdx == 9 )
    {
        BLE.TxTempBuff[5] = 0;
        xtemp = 0;
    }
    
    BLE.TxTempBuff[6] = (uint8_t)( (xtemp >>7) & 0x7F);
    BLE.TxTempBuff[7] = (uint8_t)( (xtemp & 0x007F) );
    
    BLE.PacketIdx++;
    if( BLE.PacketIdx >= 10 ) BLE.PacketIdx = 0;
    
    // ADS1292R Data - 8~107
    for(uint8_t n = 0; n<50; n++)
    {
        BLE.TxTempBuff[n*2+8] = ADS1292R.TxBuff[n*2];
        BLE.TxTempBuff[n*2+9] = ADS1292R.TxBuff[n*2+1];
    }
    
    // MPU9250 Data
    for(uint8_t n = 0; n<10; n++)
    {
        // 108 ~ 127
        BLE.TxTempBuff[n*2+108] = MPU9250.xTxBuff[n*2];
        BLE.TxTempBuff[n*2+109] = MPU9250.xTxBuff[n*2+1];
        
        // 128 ~ 147
        BLE.TxTempBuff[n*2+128] = MPU9250.yTxBuff[n*2];
        BLE.TxTempBuff[n*2+129] = MPU9250.yTxBuff[n*2+1];
        
        // 148 ~ 167
        BLE.TxTempBuff[n*2+148] = MPU9250.zTxBuff[n*2];
        BLE.TxTempBuff[n*2+149] = MPU9250.zTxBuff[n*2+1];
    }
    

//    memcpy(BLE.TxBuff, BLE.TxTempBuff, BLE_TX_LENGTH);
    BLE.TxFlag = 1;
#endif    
}

