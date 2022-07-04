#include "serialflash.h"

 /*Software Driver

SST26VF032B 32 Mbit(4M x 8) Serial Quad I/O (SQI) Flash Memory

Nov 11, 2011, Rev. 1.0

ABOUT THE SOFTWARE
This application note provides software driver examples for SST26VF032B,
Serial Flash. Extensive comments are included in each routine to describe
the function of each routine.  The interface coding uses polling method
rather than the SPI protocol to interface with these serial devices.  The
functions are differentiated below in terms of the communication protocols
(uses Mode 0) and specific device operation instructions. 


ABOUT THE SST26VF032B

Companion product datasheets for the SST26VF032B should be reviewed in
conjunction with this application note for a complete understanding
of the device.


Device Communication Protocol(pinout related) functions:

Functions                    		Function
------------------------------------------------------------------
init					Initializes clock to set up mode 0.
Send_Byte				Sends one byte using SI pin to send and
					shift out 1-bit per clock rising edge
Get_Byte				Receives one byte using SO pin to receive and shift
					in 1-bit per clock falling edge

SendSQI_Byte		 Sends one byte using SI pin, SO pin, SIO2 pin and SIO3 pin to send and
					shift out 4-bit per clock rising edge

GetSQI_Byte			 Receives one byte using SI pin, SO pin, SIO2 pin and SIO3 pin to receive and shift
					in 4-bit per clock falling edge

Note:  The pin names of the SST26VF032 are used in this application note. The associated test code
will not compile unless these pinouts (SCK, SI, SO, SIO2, SIO3, CE) are pre-defined on your
software which should reflect your hardware interfaced.


Device Operation Instruction functions:

Functions                    		Function
------------------------------------------------------------------

NoOp				No Operation
RSTEN				Enables acceptance of the RST (Reset) operation command
RST					Resets the device in to normal operating Ready mode
EQIO				Enables Quad I/O operation
RSTQIO				Resets the device to 1-bit SPI protocol operation

Read					Reads one byte from the serial flash and returns byte(max of 33 MHz CLK frequency)
Read_Cont				Reads multiple bytes(max of 33 MHz CLK frequency)
HighSpeed_Read			Reads one byte from the serial flash and returns byte(max of 80 MHz CLK frequency)
HighSpeed_Read_Cont		Reads multiple bytes(max of 80 MHz CLK frequency)
SQOR                          SPI Quad Output Read
SQIOR                         SPI Quad I/O Read
SDOR                          SPI Dual output read
SDIOR                         SPI Dual I/O Read
Set_Burst                     Specifies the number of bytes (8,16,32 or 64 bytes) to output during a Read Burst command
Read_Burst                    Reads multiple bytes as specified by Set_Burst
RBSPI                         SPI Read Burst with Wrap
  
Jedec_ID_Read			Reads the Jedec ID using SPI protocol
Quad J-ID               	Reads the Jedec ID using Quad I/O protocal
SFDP                          Serial Flash Discoverable Parameter

Sector_Erase			Erases one sector (4 KB) of the serial flash
Block_Erase				Erases 32 KByte block memory of the serial flash
Chip_Erase				Erases entire serial flash

Page_Program			Programs 1 to 256 Data Bytes
SPI Quad PP                     SPI Quad Page Program
Write Suspend		   Suspends Program/Erase  operation
Write Resume		   Resumes Program/Erase operation
nVWLDR                    non-Volatile Write Lock-Down Register
ULBPR                     Global Block Protection Unlock
Read SID			   Read Security ID
Prog SID			   Program User Security ID area
Lock SID			   Lockout Security ID Programming

RDSR					Reads the status register of the serial flash
WRSR                                    Write to the status register and configuration register
RDCR                                    Read Configuration Register
WREN					Write enables the serial flash
WRDI					Write disables the serial flash

RBPR					Read Block Protection Register
WBPR					Write Block Protection Register
LBPR					Lock Block Protection Register

Wait_Busy				Polls status register and waits until busy bit is low



*/

SerialFlashVariables SerialFlash;

/************************************************************************/
/* PROCEDURE: init							*/
/*									*/
/* This procedure initializes the SCK to low. Must be called prior to 	*/
/* setting up mode 0.							*/
/*									*/
/* Input:								*/
/*		None							*/
/*									*/
/* Output:								*/
/*		SCK							*/
/************************************************************************/
void SerialFlash_Init(void)
{
	uint8_t tempdata8 = 0;
	uint16_t tempdata16 = 0;
	
	nrf_gpio_cfg_output(FLASH_CE_PIN);
	nrf_gpio_cfg_output(FLASH_SPI_SCK_PIN);
	nrf_gpio_cfg_output(FLASH_SPI_MOSI_PIN);
	nrf_gpio_cfg_input(FLASH_SPI_MISO_PIN, NRF_GPIO_PIN_NOPULL);
	
//	nrf_gpio_pin_dir_set(FLASH_CE_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
//	nrf_gpio_pin_dir_set(FLASH_SPI_SCK_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
//	nrf_gpio_pin_dir_set(FLASH_SPI_MOSI_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
//	nrf_gpio_pin_dir_set(FLASH_SPI_MISO_PIN, NRF_GPIO_PIN_DIR_INPUT);

	nrf_gpio_pin_clear(FLASH_SPI_SCK_PIN);
	nrf_gpio_pin_clear(FLASH_SPI_MOSI_PIN);
	nrf_gpio_pin_set(FLASH_CE_PIN);
	for( volatile uint16_t i = 0; i < 1000; i++ ) {}

	SerialFlash_SPI_WREN();
	SerialFlash_SPI_Write_Status_Register(0x00, 0x00, 0x00);	
	SerialFlash_SPI_Wait_Busy();	
	tempdata8 = SerialFlash_SPI_Read_Status_Register();
	tempdata16 = SerialFlash_SPI_Read_Configuration_Register();
		
	SerialFlash_Jedec_ID_Read();
		
//		while(app_uart_put(SerialFlash.ManufacturerID) != NRF_SUCCESS);	
//		while(app_uart_put(SerialFlash.MemoryType) != NRF_SUCCESS);	
//		while(app_uart_put(SerialFlash.MemoryDensity) != NRF_SUCCESS);	
		
//	SerialFlash_SPI_WREN();
//	SerialFlash_SPI_Chip_Erase();
//	SerialFlash_SPI_Wait_Busy();
//	SerialFlash_SPI_WRDI();  
}

/************************************************************************/
/* PROCEDURE: SetSIO0_Output, SetSIO0_Input				*/
/*									*/
/* This procedure sets the Ports for SQI communicaiton          	*/
/************************************************************************/


void SerialFlash_SetSIO0_Output(void)
{
	nrf_gpio_cfg_output(FLASH_SPI_MOSI_PIN);
}

void SerialFlash_SetSIO0_Input(void)
{
	nrf_gpio_cfg_input(FLASH_SPI_MOSI_PIN, NRF_GPIO_PIN_NOPULL);
}

void SerialFlash_SetSIO1_Output(void)
{
	nrf_gpio_cfg_output(FLASH_SPI_MISO_PIN);
}

void SerialFlash_SetSIO1_Input(void)
{
	nrf_gpio_cfg_input(FLASH_SPI_MISO_PIN, NRF_GPIO_PIN_NOPULL);
}

/************************************************************************/
/* PROCEDURE: SerialFlash_SCK_High					*/
/*									*/
/* This procedure set SCK = High.					*/
/************************************************************************/
void SerialFlash_SCK_High(void)
{      
	nrf_gpio_pin_set(FLASH_SPI_SCK_PIN);
}

/************************************************************************/
/* PROCEDURE: SerialFlash_SCK_Low					*/
/*                                                      		*/
/* This procedure drives the SCK of the device to low.  		*/
/************************************************************************/
void SerialFlash_SCK_Low(void)
{
	nrf_gpio_pin_clear(FLASH_SPI_SCK_PIN);
}

/************************************************************************/
/* PROCEDURE: SerialFlash_CE_High					*/
/*									*/
/* This procedure set CE = High.					*/
/************************************************************************/
void SerialFlash_CE_High(void)
{
	nrf_gpio_pin_set(FLASH_CE_PIN);
}

/************************************************************************/
/* PROCEDURE: SerialFlash_CE_Low					*/
/*									*/
/* This procedure drives the CE of the device to low.  			*/
/************************************************************************/
void SerialFlash_CE_Low(void)
{
	nrf_gpio_pin_clear(FLASH_CE_PIN);
}

/************************************************************************/
/* PROCEDURE: SerialFlash_SIO0_High					*/
/*									*/
/* This procedure set SIO0 = High.					*/
/************************************************************************/
void SerialFlash_SIO0_High(void)
{
	nrf_gpio_pin_set(FLASH_SPI_MOSI_PIN);
}

/************************************************************************/
/* PROCEDURE: SerialFlash_SIO0_Low					*/
/*									*/
/* This procedure drives the SIO0 of the device to low. 		*/
/************************************************************************/
void SerialFlash_SIO0_Low(void)
{
	nrf_gpio_pin_clear(FLASH_SPI_MOSI_PIN);
}

/************************************************************************/
/* PROCEDURE: SerialFlash_SIO1_High					*/
/*									*/
/* This procedure set SIO1 = High.					*/
/************************************************************************/
void SerialFlash_SIO1_High(void)
{
	nrf_gpio_pin_set(FLASH_SPI_MISO_PIN);
}

/************************************************************************/
/* PROCEDURE: SerialFlash_SIO1_Low					*/
/*									*/
/* This procedure drives the SIO1 of the device to low.			*/
/************************************************************************/
void SerialFlash_SIO1_Low(void)
{
	nrf_gpio_pin_clear(FLASH_SPI_MISO_PIN);
}

/************************************************************************/
/* PROCEDURE: SerialFlash_Send_Byte							*/
/*									*/
/* This procedure outputs a byte shifting out 1-bit per clock rising	*/
/* edge on the the SI pin (SIO0 pin) LSB 1st.				*/
/************************************************************************/
void SerialFlash_Send_Byte(uint8_t out)
{

  uint8_t i = 0;
  SerialFlash_SetSIO0_Output();

  for (i = 0; i < 8; i++)
  {
    if ((out & 0x80) == 0x80)	        /* check if MSB is high */
      SerialFlash_SIO0_High();
    else
      SerialFlash_SIO0_Low();		/* if not, set to low */
    
    SerialFlash_SCK_High();		/* toggle clock high */
    out = (out << 1);		        /* shift 1 place for next bit */
    SerialFlash_SCK_Low();		/* toggle clock low */
  }
}

/************************************************************************/
/* PROCEDURE: Send_Double_Byte						*/
/*									*/
/* This procedure outputs a byte shifting out 2-bit per clock rising	*/
/* edge on the the SI pin and SO pin (SIO0,SIO1) MSB 1st.		*/
/************************************************************************/
 void SerialFlash_Send_Double_Byte(uint8_t out)
{
  SerialFlash_SetSIO0_Output();
  SerialFlash_SetSIO1_Output();

  if ((out & 0x80) ==0x80)
    {SerialFlash_SIO1_High();}
  else
    {SerialFlash_SIO1_Low();}

  if ((out & 0x40) ==0x40)
    {SerialFlash_SIO0_High();}
  else
    {SerialFlash_SIO0_Low();}

  SerialFlash_SCK_High();		/* toggle clock high */

  SerialFlash_SCK_Low();		/* toggle clock low */

  if ((out & 0x20) ==0x20)
    {SerialFlash_SIO1_High();}
  else
    {SerialFlash_SIO1_Low();}

  if ((out & 0x10) ==0x10)
    {SerialFlash_SIO0_High();}
  else
    {SerialFlash_SIO0_Low();}

  SerialFlash_SCK_High();		/* toggle clock high */

  SerialFlash_SCK_Low();		/* toggle clock low */

  if ((out & 0x08) ==0x08)
    {SerialFlash_SIO1_High();}
  else
    {SerialFlash_SIO1_Low();}

  if ((out & 0x04) ==0x04)
    {SerialFlash_SIO0_High();}
  else
    {SerialFlash_SIO0_Low();}

  SerialFlash_SCK_High();		/* toggle clock high */

  SerialFlash_SCK_Low();		/* toggle clock low */

  if ((out & 0x02) ==0x02)
    {SerialFlash_SIO1_High();}
  else
    {SerialFlash_SIO1_Low();}

  if ((out & 0x01) ==0x01)
    {SerialFlash_SIO0_High();}
  else
    {SerialFlash_SIO0_Low();}

  SerialFlash_SCK_High();		/* toggle clock high */

  SerialFlash_SIO0_High();  //Set them as Inputs
  SerialFlash_SIO1_High();

  SerialFlash_SCK_Low();		/* toggle clock low */
}

/************************************************************************/
/* PROCEDURE: Get_Byte							*/
/*									*/
/* This procedure inputs a byte shifting in 1-bit per clock falling	*/
/* edge on the SIO1 pin(LSB 1st).					*/
/************************************************************************/
uint8_t SerialFlash_Get_Byte(void)
{
  uint8_t i = 0, in = 0, temp = 0;

  SerialFlash_SetSIO1_Input();

  for (i = 0; i < 8; i++)
  {
    in = (in << 1);		/* shift 1 place to the left or shift in 0 */
    temp = nrf_gpio_pin_read(FLASH_SPI_MISO_PIN);		
    SerialFlash_SCK_High();	/* toggle clock high */
    if (temp == 1)		/* check to see if bit is high */
      in = in | 0x01;		/* if high, make bit high */

    SerialFlash_SCK_Low();	/* toggle clock low */
  }
  return in;
}

/************************************************************************/
/* PROCEDURE: Get_Double_Byte						*/
/*									*/
/* This procedure inputs a byte shifting in 2-bit per clock falling	*/
/* edge on the SIO1 pin and SIO0 pin(MSB 1st).				*/
/************************************************************************/
uint8_t SerialFlash_Get_Double_Byte(void)
{
  uint8_t i = 0, in = 0, temp = 0, temp1=0;
  
  SerialFlash_SetSIO0_Input();
  SerialFlash_SetSIO1_Input();

  for (i = 0; i < 4; i++)
  {
    in = (in << 1);			/* shift 1 place to the left or shift in 0 */
    temp = nrf_gpio_pin_read(FLASH_SPI_MISO_PIN);	
    temp1 = nrf_gpio_pin_read(FLASH_SPI_MOSI_PIN);	
    SerialFlash_SCK_High();		/* toggle clock high */

    if (temp == 1)			/* check to see if bit is high */
    {in = in | 0x01;}		        /* if high, make bit high */

    in=(in << 1);

    if (temp1 == 1)
    {in = in | 0x01;}

    SerialFlash_SCK_Low();		/* toggle clock low */
  }
  
  return in;
}

/************************************************************************/
/* PROCEDURE: NoOp                                              	*/
/*									*/
/* No operation is performed.                                           */
/************************************************************************/
void SerialFlash_Double_NoOp(void)
{
  SerialFlash_CE_Low();				/* enable device */
  SerialFlash_Send_Double_Byte(0x00);
  SerialFlash_CE_High();			/* disable device */
}

void SerialFlash_SPI_NoOp(void)
{
  SerialFlash_CE_Low();				/* enable device */
  SerialFlash_Send_Byte(0x00);
  SerialFlash_CE_High();			/* disable device */
}

///************************************************************************/
///* PROCEDURE: Read_Status_Register					*/
///*									*/
///* This procedure reads the status register and returns the byte.	*/
///************************************************************************/

uint8_t SerialFlash_SPI_Read_Status_Register(void)
{
  uint8_t byte = 0;
  
  SerialFlash_CE_Low();			/* enable device */
  SerialFlash_Send_Byte(0x05);		/* send RDSR command */
  byte = SerialFlash_Get_Byte();	/* receive byte */
  SerialFlash_CE_High();		/* disable device */
  
  return byte;
}

///************************************************************************/
///* PROCEDURE: Read_Configuration_Register				*/
///*									*/
///* This procedure reads the configuration register and returns the byte.*/
///************************************************************************/

uint16_t SerialFlash_SPI_Read_Configuration_Register(void)
{
	uint8_t byte = 0;
	uint16_t word = 0;

	SerialFlash_CE_Low();					/* enable device */
	SerialFlash_Send_Byte(0x15);		/* send RDCR command */
	byte = SerialFlash_Get_Byte();		/* receive byte */
	word = byte << 8;
	byte = SerialFlash_Get_Byte();		/* receive byte */
	word = word | byte;
	SerialFlash_CE_High();				/* disable device */

	return word;
}

///************************************************************************/
///* PROCEDURE: Write_Status_Register					*/
///*									*/
///* This procedure resumes Program/Erase operation.			*/
///************************************************************************/

void SerialFlash_SPI_Write_Status_Register(uint8_t Status_Reg, uint8_t Config_Reg_1, uint8_t Config_Reg_2)
{	  
	SerialFlash_CE_Low();				/* enable device */
	SerialFlash_Send_Byte(0x01);
	SerialFlash_Send_Byte(Status_Reg);
	SerialFlash_Send_Byte(Config_Reg_1);
	SerialFlash_Send_Byte(Config_Reg_2);
	SerialFlash_CE_High();			/* disable device */
}

/************************************************************************/
/* PROCEDURE: WREN							*/
/*									*/
/* This procedure enables the Write Enable Latch.               	*/
/************************************************************************/

void SerialFlash_SPI_WREN(void)
{
	SerialFlash_CE_Low();					/* enable device */
	SerialFlash_Send_Byte(0x06);		/* send WREN command */
	SerialFlash_CE_High();				/* disable device */
}

///************************************************************************/
///* PROCEDURE: WRDI							*/
///*									*/
///* This procedure disables the Write Enable Latch.			*/
///************************************************************************/

void SerialFlash_SPI_WRDI(void)
{
	SerialFlash_CE_Low();				/* enable device */
	SerialFlash_Send_Byte(0x04);	/* send WRDI command */
	SerialFlash_CE_High();			/* disable device */
}

/************************************************************************/
/* PROCEDURE: Wait_Busy							*/
/*									*/
/* This procedure waits until device is no longer busy (can be used by	*/
/* Byte-Program, Page-Program, Sector-Erase, Block-Erase and Chip-Erase).*/
/************************************************************************/

void SerialFlash_SPI_Wait_Busy()
{
  while ((SerialFlash_SPI_Read_Status_Register() & 0x01) == 0x01)	// waste time until not busy
    SerialFlash_SPI_Read_Status_Register();
}

///************************************************************************/
///* PROCEDURE: QuadJ_ID							*/
///*									*/
///* This procedure Reads the manufacturer's ID, device Type and device ID.  It will 	*/
///* use AFh as the command to read the ID.                               */
///* Returns:								*/
///*	ID1(Manufacture's ID = BFh, Device Type =26h , Device ID = 02h)	*/
///*									*/
///************************************************************************/

void SerialFlash_Jedec_ID_Read(void)
{
	SerialFlash_CE_Low();                                 					/* enable device */
	SerialFlash_Send_Byte(0x9F);                          				/* send JEDEC ID command (9Fh) */
	SerialFlash.ManufacturerID = SerialFlash_Get_Byte();  	/* receive byte */
	SerialFlash.MemoryType = SerialFlash_Get_Byte();      	/* receive byte */
	SerialFlash.MemoryDensity = SerialFlash_Get_Byte();  	/* receive byte */
	SerialFlash_CE_High();                               						/* disable device */
}

/************************************************************************/
/* PROCEDURE:	Read							*/
/*									*/
/* This procedure reads one address of the device.  It will return the 	*/
/* byte read in variable byte.						*/
/* Input:								*/
/*		Dst:	Destination Address 000000H - 7FFFFFH		*/
/************************************************************************/
uint8_t SerialFlash_SPI_Read(uint32_t Dst)
{
	uint8_t byte = 0;

	SerialFlash_CE_Low();                                 			/* enable device */
	SerialFlash_Send_Byte(0x03);                          		/* read command */
	SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
	SerialFlash_Send_Byte(Dst & 0xFF);
	byte = SerialFlash_Get_Byte();
	SerialFlash_CE_High();                                			/* disable device */

	return byte;                                         		 				/* return one byte read */
}

/************************************************************************/
/* PROCEDURE:	Read_Cont						*/
/*									*/
/* This procedure reads multiple addresses of the device and stores	*/
/* data into 256 byte buffer. Maximum number of bytes read is limited 256 bytes*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/*      	no_bytes	Number of bytes to read	(max = 256)	*/
/************************************************************************/
void SerialFlash_Read_Cont(uint32_t Dst, uint32_t no_bytes)
{
	uint32_t i = 0;
	
	SerialFlash_CE_Low();				        						/* enable device */
	SerialFlash_Send_Byte(0x03); 			       				 /* read command */
	SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
	SerialFlash_Send_Byte(Dst & 0xFF);
	if (no_bytes>256)
	{no_bytes=256;}
	for (i = 0; i < no_bytes; i++)		        						/* read until no_bytes is reached */
	{
		SerialFlash.data_256[i] = SerialFlash_Get_Byte();
	}
	SerialFlash_CE_High();											/* disable device */
}

/************************************************************************/
/* PROCEDURE:	HighSpeed_Read						*/
/*									*/
/* This procedure reads one address of the device.  It will return the 	*/
/* byte read in variable byte.						*/
/* Input:								*/
/*		Dst:	Destination Address 000000H - 7FFFFFH		*/
/************************************************************************/

//uint8_t SerialFlash_HighSpeed_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();                                 /* enable device */
//  SerialFlash_SendSQI_Byte(0x0B);                       /* read command */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xAF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  byte = SerialFlash_GetSQI_Byte();
//  SerialFlash_CE_High();                                /* disable device */
//  
//  return byte;                                          /* return one byte read */
//}

uint8_t SerialFlash_SPI_HighSpeed_Read(uint32_t Dst)
{
  uint8_t byte = 0;

  SerialFlash_CE_Low();                                 /* enable device */
  SerialFlash_Send_Byte(0x0B);                          /* read command */
  SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16));      /* send 3 address bytes */
  SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
  SerialFlash_Send_Byte(Dst & 0xFF);
  SerialFlash_Send_Byte(0xFF);                          /*dummy byte*/
  byte = SerialFlash_Get_Byte();
  SerialFlash_CE_High();                                /* disable device */
  
  return byte;                                          /* return one byte read */
}

/************************************************************************/
/* PROCEDURE:	HighSpeed_Read_Cont					*/
/*									*/
/* This procedure reads multiple addresses of the device and stores	*/
/* data into 256 byte buffer. Maximum number of bytes read is limited to 256 bytes*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/*      	no_bytes	Number of bytes to read	(max = 256)	*/
/************************************************************************/

//void SerialFlash_HighSpeed_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i = 0;
//  
//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_SendSQI_Byte(0x0B); 			/* read command */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xAF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  if (no_bytes>256)
//    {no_bytes=256;}

//  for (i = 0; i < no_bytes; i++)		        /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_GetSQI_Byte();
//  }
//  SerialFlash_CE_High();				/* disable device */
//}

void SerialFlash_SPI_HighSpeed_Read_Cont(uint32_t Dst, uint32_t no_bytes)
{
	uint32_t i = 0;

	SerialFlash_CE_Low();				       							/* enable device */
	SerialFlash_Send_Byte(0x0B); 			        			/* read command */
	SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
	SerialFlash_Send_Byte(Dst & 0xFF);
	SerialFlash_Send_Byte(0xFF);                	        		/*dummy byte*/
	if (no_bytes>256)
	{no_bytes=256;}

	for (i = 0; i < no_bytes; i++)		        						/* read until no_bytes is reached */
	{
		SerialFlash.data_256[i] = SerialFlash_Get_Byte();
	}
	SerialFlash_CE_High();											/* disable device */
}

//uint8_t SerialFlash_HighSpeed_InMode_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();                           	/* enable device */
//  SerialFlash_SendSQI_Byte(0x0B);
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xAF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  byte = SerialFlash_GetSQI_Byte();
//  SerialFlash_CE_High();                      	        /* disable device */
//  
//  return byte;                    	                /* return one byte read */
//}

//uint8_t SerialFlash_HighSpeed_NotInMode_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();                       	        /* enable device */
//  SerialFlash_SendSQI_Byte(0x0B); 
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xCF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  byte = SerialFlash_GetSQI_Byte();
//  SerialFlash_CE_High();                                /* disable device */
//  
//  return byte;                                          /* return one byte read */
//}



//void SerialFlash_HighSpeed_InMode_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i = 0;
//  
//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xAF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  if (no_bytes>256)
//    {no_bytes=256;}

//  for (i = 0; i < no_bytes; i++)		        /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_GetSQI_Byte();
//  }
//  SerialFlash_CE_High();				/* disable device */
//}

//void SerialFlash_HighSpeed_NotInMode_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i = 0;
//  
//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xCF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  if (no_bytes>256)
//    {no_bytes=256;}

//  for (i = 0; i < no_bytes; i++)		        /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_GetSQI_Byte();
//  }
//  SerialFlash_CE_High();				/* disable device */
//}



///************************************************************************/
///* PROCEDURE:	SPI_Quad_Output_Read					*/
///*									*/
///* This procedure reads one address of the device.  It will return the 	*/
///* byte read in variable byte.						*/
///* Input:								*/
///*		Dst:	Destination Address 000000H - 7FFFFFH		*/
///************************************************************************/
//uint8_t SerialFlash_SPI_Quad_Output_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();                                 /* enable device */
//  SerialFlash_Send_Byte(0x6B);                          /* read command */
//  SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16));      /* send 3 address bytes */
//  SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Byte(Dst & 0xFF);
//  SerialFlash_Send_Byte(0xFF);                          /*dummy byte*/
//  byte = SerialFlash_GetSQI_Byte();
//  SerialFlash_CE_High();                                /* disable device */
//  
//  return byte;                                          /* return one byte read */
//}

///************************************************************************/
///* PROCEDURE:	SPI_Quad_Output_Read_Cont				*/
///*									*/
///* This procedure reads multiple addresses of the device and stores	*/
///* data into 256 byte buffer. Maximum number of bytes read is limited to 256 bytes*/
///*									*/
///* Input:								*/
///*		Dst:		Destination Address 000000H - 7FFFFFH	*/
///*      	no_bytes	Number of bytes to read	(max = 256)	*/
///************************************************************************/
//void SerialFlash_SPI_Quad_Output_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i = 0;
//  
//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_Send_Byte(0x6B); 			        /* read command */
//  SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Byte(Dst & 0xFF);
//  SerialFlash_Send_Byte(0xFF);                          /*dummy byte*/

//  if (no_bytes>256)
//    {no_bytes=256;}

//  for (i = 0; i < no_bytes; i++)		        /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_GetSQI_Byte();
//  }
//  SerialFlash_CE_High();				/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	SPI_Quad_IO_Read					*/
///*									*/
///* This procedure reads one address of the device.  It will return the 	*/
///* byte read in variable byte.						*/
///*									*/
///* Input:								*/
///*		Dst:	Destination Address 000000H - 7FFFFFH		*/
///************************************************************************/
//uint8_t SerialFlash_SPI_Quad_IO_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();                                 /* enable device */
//  SerialFlash_Send_Byte(0xEB);                          /* read command */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xAF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  byte = SerialFlash_GetSQI_Byte();
//  SerialFlash_CE_High();                                /* disable device */
//  
//  return byte;                                          /* return one byte read */
//}
//uint8_t SerialFlash_SPI_Quad_IO_InMode_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();                       	        /* enable device */
//  SerialFlash_Send_Byte(0xEB);
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xAF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  byte = SerialFlash_GetSQI_Byte();
//  SerialFlash_CE_High();                                /* disable device */
//  
//  return byte;                                          /* return one byte read */
//}

//uint8_t SerialFlash_SPI_Quad_IO_NotInMode_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();                       	        /* enable device */
//  SerialFlash_Send_Byte(0xEB);
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  byte = SerialFlash_GetSQI_Byte();
//  SerialFlash_CE_High();                                /* disable device */
//  
//  return byte;                                          /* return one byte read */
//}

///************************************************************************/
///* PROCEDURE:	SPI_Quad_IO_Read_Cont					*/
///*									*/
///* This procedure reads multiple addresses of the device and stores	*/
///* data into 256 byte buffer. Maximum number of bytes read is limited to 256 bytes*/
///*									*/
///* Input:								*/
///*		Dst:		Destination Address 000000H - 7FFFFFH	*/
///*      	no_bytes	Number of bytes to read	(max = 256)	*/
///************************************************************************/
//void SerialFlash_SPI_Quad_IO_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i = 0;
//  
//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_Send_Byte(0xEB);                          /* read command */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xAF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/

//  if (no_bytes>256)
//    {no_bytes=256;}

//  for (i = 0; i < no_bytes; i++)		        /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_GetSQI_Byte();
//  }
//  SerialFlash_CE_High();				/* disable device */
//}

//void SerialFlash_SPI_Quad_IO_InMode_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i = 0;
//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xAF);		        /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);		        /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);		        /*dummy byte*/

//  if (no_bytes>256)
//    {no_bytes=256;}

//  for (i = 0; i < no_bytes; i++)		        /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_GetSQI_Byte();
//  }
//  SerialFlash_CE_High();				/* disable device */
//}

//void SerialFlash_SPI_Quad_IO_NotInMode_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i = 0;
//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/
//  SerialFlash_SendSQI_Byte(0xFF);                       /*dummy byte*/

//  if (no_bytes>256)
//    {no_bytes=256;}

//  for (i = 0; i < no_bytes; i++)		        /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_GetSQI_Byte();
//  }
//  SerialFlash_CE_High();				/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	SPI_Double_Output_Read					*/
///*									*/
///* This procedure reads one address of the device.  It will return the 	*/
///* byte read in variable byte.						*/
///* Input:								*/
///*		Dst:	Destination Address 000000H - 7FFFFFH		*/
///************************************************************************/
//uint8_t SerialFlash_SPI_Double_Output_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_Send_Byte(0x3B); 		        /* read command */
//  SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Byte(Dst & 0xFF);
//  SerialFlash_Send_Byte(0xFF);		        /*dummy byte*/
//  byte = SerialFlash_Get_Double_Byte();
//  SerialFlash_CE_High();			/* disable device */
//  
//  return byte;			                /* return one byte read */
//}

///************************************************************************/
///* PROCEDURE:	SPI_Double_Output_Read_Cont				*/
///*									*/
///* This procedure reads multiple addresses of the device and stores	*/
///* data into 256 byte buffer. Maximum number of bytes read is limited to 256 bytes*/
///*									*/
///* Input:								*/
///*		Dst:		Destination Address 000000H - 7FFFFFH	*/
///*      	no_bytes	Number of bytes to read	(max = 256)	*/
///*									*/
///* Returns:								*/
///*		Nothing							*/
///*									*/
///************************************************************************/
//void SerialFlash_SPI_Double_Output_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i = 0;
//  
//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_Send_Byte(0x3B); 			        /* read command */
//  SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Byte(Dst & 0xFF);
//  SerialFlash_Send_Byte(0xFF);                	        /*dummy byte*/

//  if (no_bytes>256)
//    {no_bytes=256;}

//  for (i = 0; i < no_bytes; i++)		        /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_Get_Double_Byte();
//  }
//  SerialFlash_CE_High();				/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	SPI_Double_IO_Read					*/
///*									*/
///* This procedure reads one address of the device.  It will return the 	*/
///* byte read in variable byte.						*/
///* Input:								*/
///*		Dst:	Destination Address 000000H - 7FFFFFH		*/
///************************************************************************/
//uint8_t SerialFlash_SPI_Double_IO_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_Send_Byte(0xBB); 		        /* read command */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Double_Byte(Dst & 0xFF);
//  SerialFlash_Send_Double_Byte(0xAF);		        /*dummy byte*/
//  byte = SerialFlash_Get_Double_Byte();
//  SerialFlash_CE_High();			/* disable device */
//  
//  return byte;			                /* return one byte read */
//}

//uint8_t SerialFlash_SPI_Double_IO_InMode_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();			        /* enable device */
//   SerialFlash_Send_Byte(0xBB); 		/* read command */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Double_Byte(Dst & 0xFF);
//  SerialFlash_Send_Double_Byte(0xAF);		        /*dummy byte*/
//  byte = SerialFlash_Get_Double_Byte();
//  SerialFlash_CE_High();			/* disable device */
//  
//  return byte;			                /* return one byte read */
//}

//uint8_t SerialFlash_SPI_Double_IO_NotInMode_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_Send_Byte(0xBB); 		        /* read command */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Double_Byte(Dst & 0xFF);
//  SerialFlash_Send_Double_Byte(0xFF);		        /*dummy byte*/
//  byte = SerialFlash_Get_Double_Byte();
//  SerialFlash_CE_High();			/* disable device */
//  
//  return byte;			                /* return one byte read */
//}

///************************************************************************/
///* PROCEDURE:	SPI_Double_IO_Read_Cont					*/
///*									*/
///* This procedure reads multiple addresses of the device and stores	*/
///* data into 256 byte buffer. Maximum number of bytes read is limited to 256 bytes*/
///*									*/
///* Input:								*/
///*		Dst:		Destination Address 000000H - 7FFFFFH	*/
///*      	no_bytes	Number of bytes to read	(max = 256)	*/
///************************************************************************/
//void SerialFlash_SPI_Double_IO_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i=0;
//  
//  SerialFlash_CE_Low();                         /* enable device */
//  SerialFlash_Send_Byte(0xBB);                  /* read command */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFFFF) >> 16));   /* send 3 address bytes */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Double_Byte(Dst & 0xFF);
//  SerialFlash_Send_Double_Byte(0xAF);                       /*dummy byte*/

//  if (no_bytes>256)
//    {no_bytes=256;}

//  for ( i = 0; i < no_bytes; i++)               /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_Get_Double_Byte();
//  }
//  SerialFlash_CE_High();                        /* disable device */
//}

//void SerialFlash_SPI_Double_IO_InMode_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i=0;
//  
//  SerialFlash_CE_Low();                                 /* enable device */
//                                                        /* read command */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFFFF) >> 16));	        /* send 3 address bytes */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Double_Byte(Dst & 0xFF);
//  SerialFlash_Send_Double_Byte(0xAF);                               /*dummy byte*/

//  if (no_bytes>256)
//    {no_bytes=256;}

//  for ( i = 0; i < no_bytes; i++)                       /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_Get_Double_Byte();
//  }
//  SerialFlash_CE_High();                                /* disable device */
//}

//void SerialFlash_SPI_Double_IO_NotInMode_Read_Cont(uint32_t Dst, uint32_t no_bytes)
//{
//  uint32_t i=0;
//  
//  SerialFlash_CE_Low();                         /* enable device */
//                                                /* read command */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_Send_Double_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Double_Byte(Dst & 0xFF);
//  SerialFlash_Send_Double_Byte(0xFF);                       /*dummy byte*/

//  if (no_bytes>256)
//    {no_bytes=256;}

//  for ( i = 0; i < no_bytes; i++)               /* read until no_bytes is reached */
//  {
//    SerialFlash.data_256[i] = SerialFlash_Get_Double_Byte();
//  }
//  SerialFlash_CE_High();                        /* disable device */
//}


///************************************************************************/
///* PROCEDURE:	Set_Burst						*/
///*									*/
///* This procedure sets the burst length to either 8bytes or 16bytes or 32bytes or 64bytes.			*/
///* Input:								*/
///*		byte:	00h,01h,02h or 03h for setting burst length	*/
///************************************************************************/
//void SerialFlash_Set_Burst(uint8_t byte)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_SendSQI_Byte(0xC0); 		/* send Byte Program command */
//  SerialFlash_SendSQI_Byte(byte);		/* send byte to be programmed */
//  SerialFlash_CE_High();			/* disable device */
//}

//void SerialFlash_SPI_Set_Burst(uint8_t byte)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_Send_Byte(0xC0); 			/* send Byte Program command */
//  SerialFlash_Send_Byte(byte);			/* send byte to be programmed */
//  SerialFlash_CE_High();			/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	Read_Burst                                          	*/
///*									*/
///* This procedure reads multiple (burst) address of the device.  The data is stored in an array.*/
///* Input:								*/
///*		Dst:	Destination Address 000000H - 7FFFFFH		*/
///************************************************************************/
//void SerialFlash_Read_Burst(uint32_t Dst, uint8_t burstlength)
//{
//  uint16_t i;
//  i=0;

//  SerialFlash_CE_Low();                                   /* enable device */
//  SerialFlash_SendSQI_Byte(0x0C);                         /* read command */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));     /* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);                   //Dummy cycle
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);                   //Dummy cycle
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);                   //Dummy cycle

//  //if (burstlength>256)
//  //{burstlength=256;}

//  for (i=0;i<=(burstlength-1);i++)
//  {
//    SerialFlash.data_256[i]=SerialFlash_GetSQI_Byte();
//  }

//  SerialFlash_CE_High();                                  /* disable device */
//}

//void SerialFlash_SPI_Read_Burst(uint32_t Dst, uint8_t burstlength)
//{
//  uint16_t i;
//  i=0;

//  SerialFlash_CE_Low();                                 /* enable device */
//  SerialFlash_Send_Byte(0xEC);                          /* read command */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));   /* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);                 //Dummy cycle
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);                 //Dummy cycle
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);                 //Dummy cycle

//  //if (burstlength>256)
//  //	{burstlength=256;}

//  for (i=0;i<(burstlength);i++)
//  {
//    SerialFlash.data_256[i]=SerialFlash_GetSQI_Byte();
//  }

//  SerialFlash_CE_High();                                /* disable device */

//}

///************************************************************************/
///* PROCEDURE:	Page_Program						*/
///*									*/
///* This procedure does page programming.  The destination               */
///* address should be provided.                                  	*/
///* The data array of 256 bytes contains the data to be programmed.      */
///* Since the size of the data array is 256 bytes rather than 256 bytes, this page program*/
///* procedure programs only 256 bytes                                    */
///* Assumption:  Address being programmed is already erased and is NOT	*/
///*		block protected.					*/
///* Input:								*/
///*		Dst:		Destination Address 000000H - 7FFFFFH	*/
///*		data_256[256] containing 256 bytes of data will be programmed using this function */
///************************************************************************/
//void SerialFlash_Page_Program(uint32_t Dst)
//{
//  uint16_t i;
//  i=0;

//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_SendSQI_Byte(0x02); 			/* send Byte Program command */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  for (i=0;i<=255;i++)
//  {	
//    SerialFlash_SendSQI_Byte(SerialFlash.data_256[i]);	        /* send byte to be programmed */
//  }
//  SerialFlash_CE_High();				/* disable device */
//}

void SerialFlash_SPI_Page_Program(uint32_t Dst)
{
	uint16_t i = 0;

	SerialFlash_CE_Low();				        							/* enable device */
	SerialFlash_Send_Byte(0x02); 			        				/* send Byte Program command */
	SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16));		/* send 3 address bytes */
	SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
	SerialFlash_Send_Byte(Dst & 0xFF);
	for (i=0;i<256;i++)
	{	
		SerialFlash_Send_Byte(SerialFlash.data_256[i]);	/* send byte to be programmed */
	}
	SerialFlash_CE_High();												/* disable device */
}

//void SerialFlash_Byte_Program(uint32_t Dst, uint8_t byte)
//{
//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_SendSQI_Byte(0x02); 			/* send Byte Program command */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(byte);			/* send byte to be programmed */
//  SerialFlash_CE_High();			        /* disable device */
//}

void SerialFlash_SPI_Byte_Program(uint32_t Dst, uint8_t byte)
{
  SerialFlash_CE_Low();				        /* enable device */
  SerialFlash_Send_Byte(0x02); 			        /* send Byte Program command */
  SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
  SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
  SerialFlash_Send_Byte(Dst & 0xFF);
  SerialFlash_Send_Byte(byte);			        /* send byte to be programmed */
  SerialFlash_CE_High();			        /* disable device */
}

///************************************************************************/
///* PROCEDURE:	SPI_Quad_Page_Program					*/
///*									*/
///* This procedure does page programming.  The destination               */
///* address should be provided.                                          */
///* The data array of 256 bytes contains the data to be programmed.      */
///* Since the size of the data array is 256 bytes rather than 256 bytes, this page program*/
///* procedure programs only 256 bytes                                    */
///* Assumption:  Address being programmed is already erased and is NOT	*/
///*		block protected.					*/
///* Input:								*/
///*		Dst:		Destination Address 000000H - 7FFFFFH	*/
///*		data_256[256] containing 256 bytes of data will be programmed using this function */
///************************************************************************/
//void SerialFlash_SPI_Quad_Page_Program(uint32_t Dst)
//{
//  uint16_t i;
//  i=0;

//  SerialFlash_CE_Low();				        /* enable device */
//  SerialFlash_Send_Byte(0x32); 			        /* send Byte Program command */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_SendSQI_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  for (i=0;i<=255;i++)
//  {	
//    SerialFlash_SendSQI_Byte(SerialFlash.data_256[i]);	        /* send byte to be programmed */
//  }
//  SerialFlash_CE_High();				/* disable device */
//}


/************************************************************************/
/* PROCEDURE: Chip_Erase																					*/
/*																																*/
/* This procedure erases the entire Chip.																*/
/************************************************************************/

void SerialFlash_SPI_Chip_Erase(void)
{
  SerialFlash_CE_Low();						/* enable device */
  SerialFlash_Send_Byte(0xC7);			/* send Chip Erase command (C7h) */
  SerialFlash_CE_High();						/* disable device */
}

/************************************************************************/
/* PROCEDURE: Sector_Erase																				*/
/*																																*/
/* This procedure Sector Erases the Chip.																*/
/* Input:																													*/
/*		Dst:		Destination Address 000000H - 7FFFFFH												*/
/************************************************************************/

void SerialFlash_SPI_Sector_Erase(uint32_t Dst)
{
  SerialFlash_CE_Low();				        						/* enable device */
  SerialFlash_Send_Byte(0x20);			        				/* send Sector Erase command */
  SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
  SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
  SerialFlash_Send_Byte(Dst & 0xFF);
  SerialFlash_CE_High();												/* disable device */
}

/************************************************************************/
/* PROCEDURE: Block_Erase						*/
/*									*/
/* This procedure Block Erases 8Kbyte, 32 KByte or 64 KByte of the Chip.*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/************************************************************************/

void SerialFlash_SPI_Block32k_Erase(uint32_t Dst)
{
  SerialFlash_CE_Low();				        						/* enable device */
  SerialFlash_Send_Byte(0x52);			        				/* send Block Erase command */
  SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
  SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
  SerialFlash_Send_Byte(Dst & 0xFF);
  SerialFlash_CE_High();												/* disable device */
}

void SerialFlash_SPI_Block_Erase(uint32_t Dst)
{
  SerialFlash_CE_Low();				        						/* enable device */
  SerialFlash_Send_Byte(0xD8);			        				/* send Block Erase command */
  SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
  SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
  SerialFlash_Send_Byte(Dst & 0xFF);
  SerialFlash_CE_High();												/* disable device */
}

///************************************************************************/
///* PROCEDURE: ResetEn                                                   */
///*									*/
///* This procedure Enables acceptance of the RST (Reset) operation.	*/
///************************************************************************/
//void SerialFlash_ResetEn(void)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_SendSQI_Byte(0x66);
//  SerialFlash_CE_High();			/* disable device */
//}

//void SerialFlash_SPI_ResetEn(void)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_Send_Byte(0x66);
//  SerialFlash_CE_High();			/* disable device */
//}

///************************************************************************/
///* PROCEDURE: Reset                                     		*/
///*									*/
///* This procedure resets the device in to normal operating Ready mode.	*/
///*									*/
///************************************************************************/
//void SerialFlash_Reset()
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_SendSQI_Byte(0x99);
//  SerialFlash_CE_High();			/* disable device */
//}

//void SerialFlash_SPI_Reset()
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_Send_Byte(0x99);
//  SerialFlash_CE_High();			/* disable device */
//}

///************************************************************************/
///* PROCEDURE: En_QIO                                    		*/
///*									*/
///* This procedure enables quad I/O operation.           		*/
///************************************************************************/
//void SerialFlash_En_QIO(void)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_Send_Byte(0x38);
//  SerialFlash_CE_High();			/* disable device */
//}

///************************************************************************/
///* PROCEDURE: Reset_QIO                                 		*/
///*									*/
///* This procedure resets the device to 1-bit SPI protocol operation.    */
///************************************************************************/
//void SerialFlash_Reset_QIO(void)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_SendSQI_Byte(0xff);
//  SerialFlash_CE_High();			/* disable device */
//}

//void SerialFlash_SPI_Reset_QIO(void)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_Send_Byte(0xff);
//  SerialFlash_CE_High();			/* disable device */
//}

///************************************************************************/
///* PROCEDURE: Write_Suspend						*/
///*									*/
///* This procedure suspends Program/Erase operation.			*/
///************************************************************************/
//void SerialFlash_Write_Suspend(void)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_SendSQI_Byte(0xb0);
//  SerialFlash_CE_High();			/* disable device */
//}
//void SerialFlash_SPI_Write_Suspend(void)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_Send_Byte(0xb0);
//  SerialFlash_CE_High();			/* disable device */
//}

///************************************************************************/
///* PROCEDURE: Write_Resume						*/
///*									*/
///* This procedure resumes Program/Erase operation.			*/
///************************************************************************/
//void SerialFlash_Write_Resume(void)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_SendSQI_Byte(0x30);
//  SerialFlash_CE_High();			/* disable device */
//}
//void SerialFlash_SPI_Write_Resume(void)
//{
//  SerialFlash_CE_Low();				/* enable device */
//  SerialFlash_Send_Byte(0x30);
//  SerialFlash_CE_High();			/* disable device */
//}

// /************************************************************************/
///* PROCEDURE:	ReadSID	(Read Security ID)				*/
///*									*/
///* This procedure reads the security ID					*/
///************************************************************************/
//void SerialFlash_ReadSID(uint8_t *security_ID, uint32_t Dst, uint32_t security_length)
//{
//  uint32_t i;
//  i=0;
//          
//  if (security_length>2048)
//    {security_length=2048;}

//  SerialFlash_CE_Low();                               /* enable device */
//  SerialFlash_SendSQI_Byte(0x88);
//  SerialFlash_SendSQI_Byte((Dst>>8) & 0xFF);
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);              //dummy
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);              //dummy
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);              //dummy

//  for (i=0;i<security_length;i++)
//  {
//    *security_ID = SerialFlash_GetSQI_Byte();
//    ++security_ID;
//  }
//  SerialFlash_CE_High();                              /* disable device */
//}

//void SerialFlash_SPI_ReadSID(uint8_t *security_ID, uint32_t Dst, uint32_t security_length)
//{
//  uint32_t i;
//  i=0;
//          
//  if (security_length>2048)
//    {security_length=2048;}

//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_Send_Byte(0x88);
//  SerialFlash_Send_Byte((Dst>>8) & 0xFF);
//  SerialFlash_Send_Byte(Dst & 0xFF);
//  SerialFlash_Send_Byte(Dst & 0xFF);  //dummy

//  for (i=0;i<security_length;i++)
//  {
//    *security_ID = SerialFlash_Get_Byte();
//    security_ID++;
//  }
//  SerialFlash_CE_High();			/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	ProgSID	(Program Security ID)                           */
///*									*/
///* This procedure programs the security ID				*/
///*									*/
///************************************************************************/
//void SerialFlash_ProgSID(uint8_t *security_ID, uint32_t Dst, uint32_t security_length)
//{ 
//  uint32_t i;

//  i=0;

//  if (security_length>256)
//    {security_length=256;}

//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_SendSQI_Byte(0xa5);
//  SerialFlash_SendSQI_Byte((Dst>>8) & 0xFF);
//  SerialFlash_SendSQI_Byte(Dst & 0xFF);

//  for (i=0;i<security_length;i++)
//  {
//    SerialFlash_SendSQI_Byte(*security_ID);
//    security_ID++;
//  }
//  SerialFlash_CE_High();			/* disable device */
//}

//void SerialFlash_SPI_ProgSID(uint8_t *security_ID, uint32_t Dst, uint32_t security_length)
//{
//  uint32_t i;

//  i=0;
//        
//  if (security_length>256)
//    {security_length=256;}

//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_Send_Byte(0xa5);
//  SerialFlash_Send_Byte((Dst>>8) & 0xFF);
//  SerialFlash_Send_Byte(Dst & 0xFF);

//  for (i=0;i<security_length;i++)
//  {
//    SerialFlash_Send_Byte(*security_ID);
//    security_ID++;
//  }

//  SerialFlash_CE_High();			/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	LockSID							*/
///*									*/
///* This procedure Locks the security ID setting				*/
///*									*/
///************************************************************************/
//void SerialFlash_LockSID(void)
//{
//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_SendSQI_Byte(0x85);
//  SerialFlash_CE_High();			/* disable device */
//}

//void SerialFlash_SPI_LockSID(void)
//{
//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_Send_Byte(0x85);
//  SerialFlash_CE_High();			/* disable device */
//}

//  /************************************************************************/
///* PROCEDURE:	ReadBlockProtection			  		*/
///*									*/
///* This procedure reads block protection register			*/
///*									*/
///************************************************************************/
//void SerialFlash_ReadBlockProtection(void)
//{
//  uint16_t i;
//  i=0;

//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_SendSQI_Byte(0x72);
//  SerialFlash_SendSQI_Byte(0xff);
//  for (i=10;i>0;i--)
//  {	
//    SerialFlash.block_protection_10[i-1] = SerialFlash_GetSQI_Byte();
//  }
//  SerialFlash_CE_High();			/* disable device */
//}

//void SerialFlash_SPI_ReadBlockProtection(void)
//{
//  uint8_t i;
//  i=0;

//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_Send_Byte(0x72);

//  for (i=10;i>0;i--)
//  {	
//    SerialFlash.block_protection_10[i-1] = SerialFlash_Get_Byte();
//  }
//  SerialFlash_CE_High();			/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	WriteBlockProtection					*/
///*									*/
///* This procedure writes to block protection register			*/
///*									*/
///************************************************************************/
//void SerialFlash_WriteBlockProtection(void)
//{
//  uint8_t i;
//  i=0;

//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_SendSQI_Byte(0x42); 		/* read command */

//  for (i=10;i>0;i--)
//  {
//    SerialFlash_SendSQI_Byte(SerialFlash.block_protection_10[i-1]);
//  }
//  SerialFlash_CE_High();			/* disable device */
//}

//void SerialFlash_SPI_WriteBlockProtection(void)
//{
//  uint8_t i;
//  i=0;

//  SerialFlash_CE_Low();			        /* enable device */
//  SerialFlash_Send_Byte(0x42); 		        /* read command */

//  for (i=10;i>0;i--)
//  {
//          SerialFlash_Send_Byte(SerialFlash.block_protection_10[i-1]);
//  }
//  SerialFlash_CE_High();			/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	Global Block Protection Unlock				*/
///*									*/
///* This procedure clears all block protection				*/
///************************************************************************/
//void SerialFlash_Global_Block_Protection_Unlock(void)
//{
//  SerialFlash_CE_Low();			/* enable device */
//  SerialFlash_SendSQI_Byte(0x98); 	/* read command */
//  SerialFlash_CE_High();		/* disable device */
//}

//void SerialFlash_SPI_Global_Block_Protection_Unlock(void)
//{
//  SerialFlash_CE_Low();			/* enable device */
//  SerialFlash_Send_Byte(0x98); 		/* read command */
//  SerialFlash_CE_High();		/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	LockBlockProtection					*/
///*									*/
///* This procedure locks the block protection register			*/
///************************************************************************/
//void SerialFlash_LockBlockProtection()
//{
//  SerialFlash_CE_Low();			/* enable device */
//  SerialFlash_SendSQI_Byte(0x8d); 	/* read command */
//  SerialFlash_CE_High();		/* disable device */
//}

//void SerialFlash_SPI_LockBlockProtection()
//{
//  SerialFlash_CE_Low();			/* enable device */
//  SerialFlash_Send_Byte(0x8d); 		/* read command */
//  SerialFlash_CE_High();		/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	Non Volatile Write Lock Protection			*/
///*									*/
///* This procedure writes to block protection register			*/
///*									*/
///************************************************************************/
//void SerialFlash_NonVolWriteLockProtection()
//{
//  uint8_t i;
//  i=0;

//  SerialFlash_CE_Low();		        /* enable device */
//  SerialFlash_SendSQI_Byte(0xE8); 	/* read command */

//  for (i=10;i>0;i--)
//  {
//    SerialFlash_SendSQI_Byte(SerialFlash.block_protection_10[i-1]);
//  }
//  SerialFlash_CE_High();		/* disable device */
//}

//void SerialFlash_SPI_NonVolWriteLockProtection()
//{
//  uint8_t i;
//  i=0;

//  SerialFlash_CE_Low();			/* enable device */
//  SerialFlash_Send_Byte(0xE8); 		/* read command */

//  for (i=10;i>0;i--)
//  {
//    SerialFlash_Send_Byte(SerialFlash.block_protection_10[i-1]);
//  }
//  SerialFlash_CE_High();		/* disable device */
//}

///************************************************************************/
///* PROCEDURE:	SPI_SFDP_Read						*/
///*									*/
///* This procedure reads SFDP Table.					*/
///*									*/
///************************************************************************/
//uint8_t SerialFlash_SPI_SFDP_Read(uint32_t Dst)
//{
//  uint8_t byte = 0;

//  SerialFlash_CE_Low();                                 /* enable device */
//  SerialFlash_Send_Byte(0x5A);                          /* read command */
//  SerialFlash_Send_Byte(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  SerialFlash_Send_Byte(((Dst & 0xFFFF) >> 8));
//  SerialFlash_Send_Byte(Dst & 0xFF);
//  SerialFlash_Send_Byte(0xFF);                	        /*dummy byte*/
//  byte = SerialFlash_Get_Byte();
//  SerialFlash_CE_High();                      	        /* disable device */
//  
//  return byte;                    	                /* return one byte read */
//}

// /****************************************************/
// /* Main Function*/
// /*****************************************************/
//void SerialFlash_Test(void)
//{
//  uint8_t Led_Stat;
//  uint32_t i, j, k, highest_address, tempdatalong;
//  int check,tempcheck,status;
//  int tempdata;
//  
//  i = 0;
//  j = 0;
//  Led_Stat = 0x00;
//  status = 1;  //1 means memory/code works and 0 means fault has occured.
//  check = 0;  //keeps track of code progress.
//  tempcheck = 1;

//  highest_address = 0x1FFFF;
//  SerialFlash.DeviceAddress = 0;
//  SerialFlash.SW = 0;

//  i=0;
//  while (i<=0xFFFF)
//  {
//    STM_EVAL_LEDOn(LED3);
//    i++;
//  }
//  STM_EVAL_LEDOff(LED3);
//  SerialFlash_SPI_WREN();
//  tempdatalong = SerialFlash_SPI_Read_Status_Register();
//  tempdatalong = tempdatalong<<8;
//  tempdatalong = tempdatalong |(0x82);
//  // tempdatalong = tempdatalong & ~(0x02);

//  SerialFlash_SPI_Write_Status_Register(tempdatalong, 2);

//  tempdata = SerialFlash_SPI_Read_Configuration_Register();

//  // Test Jedec ID

//  /************* 1a. Test Jedec ID *********/

//  SerialFlash_Jedec_ID_Read();

//  if ((SerialFlash.ManufacturerID == 0xbf) && (SerialFlash.DeviceType==0x26) && (SerialFlash.DeviceID==0x42)&&(status==1))
//    check = (check + 1);
//  else
//    check = 0;

//  STM_EVAL_LEDOn(LED3);
// 		
///************* 2a. Page program whole chip using SPI protocol and verify its OK. *********/

//  i = 0;
//  while (i<=255)
//  {
//    SerialFlash.data_256[i] = 170;
//    i++;
//  }

//  SerialFlash_SPI_WREN();

//  SerialFlash.block_protection_10[0] = 0x00;
//  SerialFlash.block_protection_10[1] = 0x00;
//  SerialFlash.block_protection_10[2] = 0x00;
//  SerialFlash.block_protection_10[3] = 0x00;
//  SerialFlash.block_protection_10[4] = 0x00;
//  SerialFlash.block_protection_10[5] = 0x00;
//  SerialFlash.block_protection_10[6] = 0x00;
//  SerialFlash.block_protection_10[7] = 0x00;
//  SerialFlash.block_protection_10[8] = 0x00;
//  SerialFlash.block_protection_10[9] = 0x00;

//  SerialFlash_SPI_WriteBlockProtection();
//  SerialFlash_SPI_Wait_Busy();
//  SerialFlash_SPI_WREN();

//  SerialFlash_SPI_Chip_Erase();	//Erase the chip

//  SerialFlash_SPI_Wait_Busy();
//  i = 0;
//  SerialFlash_SPI_WREN();

//  while(i<highest_address)
//  {	
//    SerialFlash_SPI_WREN();
//    SerialFlash_SPI_Page_Program(i);
//    SerialFlash_SPI_Wait_Busy();
//    i = i + 256;
//  }

//  SerialFlash_SPI_WREN();

//  j = 0;
//  while(j<highest_address)
//  {
//    SerialFlash_SPI_HighSpeed_Read_Cont(j, 64);
//    for (i=0;i<64;i++)
//    {
//      if (SerialFlash.data_256[i]==170)
//      {
//        tempcheck &= 1;
//      }
//      else
//      { 
//        tempcheck = 0;
//      }
//    }
//    j = j + 64;
//  }


//  if ((tempcheck == 1)&&(status==1))
//  {
//    check = (check+1);
//    status = status&1;
//  }
//  else
//  {     
//    status = 0;
//  }

//  STM_EVAL_LEDOn(LED4);


///************* 3a. Do Sector Erase and verify that the sector is erased *********/
//  SerialFlash_SPI_WREN();
//  SerialFlash_SPI_Sector_Erase(0);	//Do Sector Erase
//  SerialFlash_SPI_Wait_Busy();

//  j = 0;
//  while (j<(4096))
//  {
//    SerialFlash_SPI_WREN();
//    SerialFlash_SPI_Set_Burst(0x03);
//    SerialFlash_SPI_Read_Burst(j, 64);
//    for (k=0;k<63;k++)
//    {
//      if (SerialFlash.data_256[k]==0xff)			  // Verify that the values are correct
//        tempcheck &= 1;
//      else
//        tempcheck = 0;
//    }
//    j = j + 64;
//  }

//  j = 4096;
//  while (j<(4096*2))
//  {
//    SerialFlash_SPI_WREN();
//    SerialFlash_SPI_Set_Burst(0x03);
//    SerialFlash_SPI_Read_Burst(j, 64);
//    for (k=0;k<63;k++)
//    {
//      if (SerialFlash.data_256[k]==0xAA)			  // Verify that the values are correct
//        tempcheck &= 1;
//      else
//        tempcheck = 0;
//    }
//    j = j + 64;
//  }

//  if ((tempcheck == 1)&&(status==1))
//  {
//    check = (check+1);
//    status = status&1;
//  }
//  else
//    status = 0;


//  STM_EVAL_LEDOn(LED5);

/////************* 4a. Block Erase and verify that the Block is erased *********/

//  SerialFlash_SPI_WREN();
//  SerialFlash_SPI_Block_Erase(0);	//Do Sector Erase
//  SerialFlash_SPI_Wait_Busy();

//  j = 0;
//  while (j<(0x2000))
//  {
//    SerialFlash_SPI_WREN();
//    SerialFlash_SPI_Set_Burst(0x03);
//    SerialFlash_SPI_Read_Burst(j, 64);
//    for (k=0;k<63;k++)
//    {
//      if (SerialFlash.data_256[k]==0xff)			  // Verify that the values are correct
//        tempcheck &= 1;
//      else
//        tempcheck = 0;
//    }
//    j = j + 64;
//  }

//  j = 0x2000;
//  while (j<(0x3000))
//  {
//    SerialFlash_SPI_WREN();
//    SerialFlash_SPI_Set_Burst(0x03);
//    SerialFlash_SPI_Read_Burst(j, 64);
//    for (k=0;k<63;k++)
//    {
//      if (SerialFlash.data_256[k]==0xAA)			  // Verify that the values are correct
//        tempcheck &= 1;
//      else
//        tempcheck = 0;
//    }
//    j = j+64;
//  }


//  if ((tempcheck == 1)&&(status==1))
//  {
//    check = (check+1);
//    status = status&1;
//  }
//  else
//  {
//    status = 0;
//    check |= 0x44;
//  }

//  STM_EVAL_LEDOn(LED6);

///************* 5a. Chip Erase and Verify Chip is erased*********/

//  SerialFlash_SPI_WREN();
//  SerialFlash_SPI_Chip_Erase();	//Do Sector Erase
//  SerialFlash_SPI_Wait_Busy();

//  j=0;
//  while (j<highest_address)
//  {
//    SerialFlash_SPI_WREN();
//    SerialFlash_SPI_Set_Burst(0x03);
//    SerialFlash_SPI_Read_Burst(j, 64);
//    for (k=0;k<63;k++)
//    {
//      if (SerialFlash.data_256[k]==0xFF)			  // Verify that the values are correct
//        tempcheck &= 1;
//      else
//        tempcheck = 0;
//    }
//    j = j + 64;
//  }

//  if ((tempcheck == 1)&&(status==1))
//  {
//    check = (check+1);
//    status = status&1;
//  }
//  else
//  {
//    status = 0;
//  }
//  
//  // Test Quad J-ID

//  STM_EVAL_LEDOn(LED7);
//        
///************* 1b. Test Quad ID *********/
//			
//  SerialFlash.ManufacturerID = 0x00;
//  SerialFlash.DeviceType = 0x00;
//  SerialFlash.DeviceID = 0x00;	 		 

//  SerialFlash_En_QIO();	 	//Serial Quad IO is now enabled
//  SerialFlash_WREN();	  
//  SerialFlash_QuadJ_ID_Read();

//  if ((SerialFlash.ManufacturerID == 0xbf) && (SerialFlash.DeviceType==0x26) && (SerialFlash.DeviceID==0x42) && (status==1))
//  {
//    check = (check+1);
//    status = status&1;
//  }
//  else
//  {
//    status = 0;
//  }

//  STM_EVAL_LEDOn(LED8);

///****** 2b.  Page program whole chip using SQI protocol and verify its OK *****/

//  i = 0;
//  while (i<=255)
//  {
//    SerialFlash.data_256[i] = 170;
//    i++;
//  }

//  SerialFlash_WREN();

//  SerialFlash.block_protection_10[0] = 0x00;
//  SerialFlash.block_protection_10[1] = 0x00;
//  SerialFlash.block_protection_10[2] = 0x00;
//  SerialFlash.block_protection_10[3] = 0x00;
//  SerialFlash.block_protection_10[4] = 0x00;
//  SerialFlash.block_protection_10[5] = 0x00;
//  SerialFlash.block_protection_10[6] = 0x00;
//  SerialFlash.block_protection_10[7] = 0x00;
//  SerialFlash.block_protection_10[8] = 0x00;
//  SerialFlash.block_protection_10[9] = 0x00;

//  SerialFlash_WriteBlockProtection();
//  SerialFlash_Wait_Busy();
//  
//  SerialFlash_WREN();
//  SerialFlash_Chip_Erase();	//Erase the chip
//  SerialFlash_Wait_Busy();

//  SerialFlash_WREN();

//  i = 0;
//  while(i<highest_address)
//  {	
//    SerialFlash_WREN();
//    SerialFlash_Page_Program(i);
//    SerialFlash_Wait_Busy();
//    i = i + 256;
//  }

//  SerialFlash_WREN();

//  j = 0;
//  while(j<highest_address)
//  {
//    SerialFlash_Set_Burst(0x03);
//    SerialFlash_Read_Burst(j, 64); 			//verify that it got programmed.
//    SerialFlash_Wait_Busy();

//    for (i=0;i<64;i++)
//    {
//      if (SerialFlash.data_256[i]==170)
//        tempcheck &= 1;
//      else
//        tempcheck = 0;
//    }
//    j = j + 64;
//  }

//  if ((tempcheck == 1)&&(status==1))
//  {
//    check = (check+1);
//    status = status&1;
//  }
//  else
//  {
//    status = 0;
//  }
//  
//  STM_EVAL_LEDOff(LED3);

///************* 3b.  Sector Erase and Verify sector is erased*********/
//  SerialFlash_WREN();
//  SerialFlash_Sector_Erase(0);	//Do Sector Erase
//  SerialFlash_Wait_Busy();

//  j=0;
//  while (j<(4096))
//  {
//    SerialFlash_WREN();
//    SerialFlash_Set_Burst(0x03);
//    SerialFlash_Read_Burst(j, 64);
//    for (k=0;k<63;k++)
//    {
//      if (SerialFlash.data_256[k]==0xff)			  // Verify that the values are correct
//        tempcheck&=1;
//      else
//        tempcheck=0;
//    }
//    j = j + 64;
//  }

//  j = 4096;
//  while (j<(4096*2))
//  {
//    SerialFlash_WREN();
//    SerialFlash_Set_Burst(0x03);
//    SerialFlash_Read_Burst(j, 64);
//    for (k=0;k<63;k++)
//    {
//      if (SerialFlash.data_256[k]==0xAA)			  // Verify that the values are correct
//        tempcheck &= 1;
//      else
//        tempcheck = 0;
//    }
//    j = j + 64;
//  }

//  if ((tempcheck == 1)&&(status==1))
//  {
//    check = (check+1);
//    status = status&1;
//  }
//  else
//  {
//    status = 0;
//    check |= 0x44;
//  }

//  STM_EVAL_LEDOff(LED4);

///************* 4b. Block Erase and verify that the Block is erased*********/

//  SerialFlash_WREN();
//  SerialFlash_Block_Erase(0);	//Do Sector Erase
//  SerialFlash_Wait_Busy();

//  j = 0;

//  while (j<(0x2000))
//  {
//    SerialFlash_WREN();
//    SerialFlash_Set_Burst(0x03);
//    SerialFlash_Read_Burst(j, 64);
//    for (k=0;k<63;k++)
//    {
//      if (SerialFlash.data_256[k]==0xff)			  // Verify that the values are correct
//        tempcheck&=1;
//      else
//        tempcheck=0;
//    }
//    j = j + 64;
//  }

//  j = 0x2000;
//  while (j<((0x3000)))
//  {
//    SerialFlash_WREN();
//    SerialFlash_Set_Burst(0x03);
//    SerialFlash_Read_Burst(j, 64);
//    for (k=0;k<63;k++)
//    {
//      if (SerialFlash.data_256[k]==0xAA)			  // Verify that the values are correct
//        tempcheck &= 1;
//      else
//        tempcheck=0;
//    }
//    j = j + 64;
//  }

//  if ((tempcheck == 1)&&(status==1))
//  {
//    check = (check+1);
//    status = status&1;
//  }
//  else
//  {
//    status=0;
//    check|=0x44;
//  }

//  STM_EVAL_LEDOff(LED5);
//  
///************* 5b. Chip Erase and verify chip is erased *********/

//  SerialFlash_WREN();
//  SerialFlash_Chip_Erase();	//Do Sector Erase
//  SerialFlash_Wait_Busy();

//  j = 0;
//  while (j<highest_address)
//  {
//    SerialFlash_WREN();
//    SerialFlash_Set_Burst(0x03);
//    SerialFlash_Read_Burst(j, 64);
//    for (k=0;k<63;k++)
//    {
//      if (SerialFlash.data_256[k]==0xff)	// Verify that the values are correct
//        tempcheck &= 1;
//      else
//        tempcheck = 0;
//    }
//    j = j + 64;
//  }

//  if ((tempcheck == 1) && (status==1))
//  {
//    check = (check+1);
//    status = status&1;
//  }
//  else
//  {
//    status = 0;
//  }
//  
//  STM_EVAL_LEDOff(LED6);

//  SerialFlash_ResetEn();
//  SerialFlash_Reset();
///***************** End of checking ******/
//  Led_Stat = check;
//  if (status==0) //means a failure has occured
//  {
//      Led_Stat|=0x80;
//  }
//  else
//  {
//      STM_EVAL_LEDOff(LED7);
//      STM_EVAL_LEDOff(LED8);
//  }

////  while(1)
////  {
////          DisplayLED(Led_Stat);
////  }

//}

