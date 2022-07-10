#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_delay.h"
#include "nrfx_spim.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_timer.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_wdt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "adc.h"
#include "keypad.h"
#include "mz_ble.h"
#include "timer.h"
#include "ads1292r.h"
#include "mpu9250.h"

/* System Information */
#define BLE_HW_VERSION              100
#define BLE_FW_VERSION    	        100

/* UART */
#define UART_RX_PIN                 8
#define UART_TX_PIN                 6
#define UART_CTS_PIN                7
#define UART_RTS_PIN                5

/* GPIO */
#define PS_HOLD						24 	/* OUTPUT */
#define LDO_AVCC_EN					23	/* OUTPUT */
#define LED1						17	/* OUTPUT */
#define LED2						18	/* OUTPUT */
#define BLE_CONNECT_PIN				LED1

/* I2C */
#define I2C1_SCL_PIN				27 	
#define I2C1_SDA_PIN				26

/* SPI */
#define SPI0_SCK_PIN                29  // SPI clock GPIO pin number.
#define SPI0_MOSI_PIN               25  // SPI Master Out Slave In GPIO pin number.
#define SPI0_MISO_PIN               28  // SPI Master In Slave Out GPIO pin number.
#define SPI0_SS_PIN                 22  // SPI Slave Select GPIO pin number.

/* Standard Bits */
#define BIT0                (0x0001)
#define BIT1                (0x0002)
#define BIT2                (0x0004)
#define BIT3                (0x0008)
#define BIT4                (0x0010)
#define BIT5                (0x0020)
#define BIT6                (0x0040)
#define BIT7                (0x0080)
#define BIT8                (0x0100)
#define BIT9                (0x0200)
#define BIT10               (0x0400)
#define BIT11               (0x0800)
#define BIT12               (0x1000)
#define BIT13               (0x2000)
#define BIT14               (0x4000)
#define BIT15               (0x8000)

#define BITS_PER_LONG           32
#define DIV_ROUND_UP(n,d)       (((n) + (d) - 1) / (d))
   
/*
 * Create a contiguous bitmask starting at bit position @l and ending at
 * position @h. For example
 * GENMASK_ULL(39, 21) gives us the 64bit vector 0x000000ffffe00000.
 */
#define GENMASK(h, l) \
         (((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))
#define GENMASK_ULL(h, l) \
         (((~0ULL) << (l)) & (~0ULL >> (BITS_PER_LONG_LONG - 1 - (h))))
   

#define BIT(nr)                 (1UL << (nr))
#define BIT_ULL(nr)             (1ULL << (nr))
#define BIT_MASK(nr)            (1UL << ((nr) % BITS_PER_LONG))
#define BIT_WORD(nr)            ((nr) / BITS_PER_LONG)
#define BIT_ULL_MASK(nr)        (1ULL << ((nr) % BITS_PER_LONG_LONG))
#define BIT_ULL_WORD(nr)        ((nr) / BITS_PER_LONG_LONG)
#define BITS_PER_BYTE           8
#define BITS_TO_LONGS(nr)       DIV_ROUND_UP(nr, BITS_PER_BYTE * sizeof(long))

#define UART1_TX_LENGTH      		42
#define UART1_RX_LENGTH          	20	

typedef struct _UART1Variables
{ 
	uint8_t RxData;
	uint8_t TxFlag;
	uint8_t RxFlag;
	uint8_t TxBuff[UART1_TX_LENGTH];
	uint8_t RxBuff[UART1_RX_LENGTH];
	uint8_t RxIndex;
} UART1Variables;

typedef struct _SystemVariables
{ 
    uint8_t Start;
	uint8_t Device_Number_Str;
	uint16_t Device_Number;
    uint16_t HW_Version;
    uint16_t SW_Version;
} SystemVariables;

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ClearMem(unsigned char * pMEM, unsigned int nSize);
int32_t TwosComplementConverterInt32(uint32_t input, uint8_t bitLength);
    
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
