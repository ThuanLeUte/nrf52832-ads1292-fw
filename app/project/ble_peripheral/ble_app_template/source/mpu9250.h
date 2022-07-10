#ifndef __MPU9250_H
#define __MPU9250_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "math.h"
   
typedef enum
{
	MPU9250_OK = 0,
	MPU9250_FAIL
}MPU9250_Status_TypDef;

typedef enum
{
	AK8963_OK = 0,
	AK8963_FAIL
}AK8963_Status_TypDef;

typedef struct _MPU9250Variables
{
	uint8_t INTO_Flag;
	uint8_t WhoAmI;
	uint8_t Mode;
	uint8_t nSampleCnt;
	uint8_t TxBuff[20];
	int16_t AccelCount[3];  /* Stores the 16-bit signed accelerometer sensor output */
	int16_t GyroCount[3];   /* Stores the 16-bit signed gyro sensor output */
	int16_t TempCount;      /* Stores the real internal chip temperature in degrees Celsius */

	int16_t mxAcc[32];  
	int16_t myAcc[32];  
	int16_t mzAcc[32]; 
	
	int32_t dmxAcc[32];  
	int32_t dmyAcc[32];  
	int32_t dmzAcc[32]; 
	int16_t maccx, maccy, maccz;
	int16_t baccx, baccy, baccz;
	
	float maxxyz;
	float xyzbuf;
    int16_t Xaxis, Yaxis, Zaxis;
    uint8_t xTxBuff[20];
    uint8_t yTxBuff[20];
    uint8_t zTxBuff[20];
	
	float GyroBias[3];
	float AccelBias[3];

	float SelfTest[6];

	double ax, ay, az;
	float gx, gy, gz;
	float Temperature;  
	
	float fxTemp, fyTemp, fzTemp;
	
} MPU9250Variables;

typedef struct _AK8963Variables
{
	uint8_t WhoAmI;
	int16_t MagCount[3];    /* Stores the 16-bit signed magnetometer sensor output */
	/* Factory mag calibration and mag bias */
	float MagCalibration[3];
	float MagBias[3];

	float mx, my, mz;
} AK8963Variables;

/***********************************************************************************************\
* Public type definitions
\***********************************************************************************************/
#define MPU9250_INTO_PIN      		15

#define MPU9250_ADDR               (0xD0 >> 1)   /* 7bit Address = 0x68 */

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDR             	(0x18 >> 1)	/* 7bit Address = 0x0C */
#define AK8963_WHO_AM_I        		0x00 // should return 0x48
#define AK8963_INFO               0x01
#define AK8963_ST1              	0x02  // data ready status bit 0
#define AK8963_XOUT_L           	0x03  // data
#define AK8963_XOUT_H           	0x04
#define AK8963_YOUT_L           	0x05
#define AK8963_YOUT_H           	0x06
#define AK8963_ZOUT_L           	0x07
#define AK8963_ZOUT_H           	0x08
#define AK8963_ST2              	0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL             	0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC             	0x0C  // Self test control
#define AK8963_I2CDIS           	0x0F  // I2C disable
#define AK8963_ASAX             	0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY             	0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ             	0x12  // Fuse ROM z-axis sensitivity adjustment value

#define MPU9250_SELF_TEST_X_GYRO    0x00                  
#define MPU9250_SELF_TEST_Y_GYRO    0x01                                                                          
#define MPU9250_SELF_TEST_Z_GYRO    0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define MPU9250_SELF_TEST_X_ACCEL   		0x0D
#define MPU9250_SELF_TEST_Y_ACCEL     	0x0E    
#define MPU9250_SELF_TEST_Z_ACCEL    		0x0F

#define MPU9250_SELF_TEST_A             	0x10

#define MPU9250_XG_OFFSET_H             	0x13  // User-defined trim values for gyroscope
#define MPU9250_XG_OFFSET_L             	0x14
#define MPU9250_YG_OFFSET_H             	0x15
#define MPU9250_YG_OFFSET_L             	0x16
#define MPU9250_ZG_OFFSET_H             	0x17
#define MPU9250_ZG_OFFSET_L             	0x18
#define MPU9250_SMPLRT_DIV              	0x19
#define MPU9250_CONFIG                  	0x1A
#define MPU9250_GYRO_CONFIG             	0x1B
#define MPU9250_ACCEL_CONFIG            	0x1C
#define MPU9250_ACCEL_CONFIG2           	0x1D
#define MPU9250_LP_ACCEL_ODR            	0x1E   
#define MPU9250_WOM_THR                 	0x1F   

#define MPU9250_MOT_DUR                 	0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MPU9250_ZMOT_THR                	0x21  // Zero-motion detection threshold bits [7:0]
#define MPU9250_ZRMOT_DUR               	0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define MPU9250_FIFO_EN                 	0x23
#define MPU9250_I2C_MST_CTRL            	0x24   
#define MPU9250_I2C_SLV0_ADDR           	0x25
#define MPU9250_I2C_SLV0_REG            	0x26
#define MPU9250_I2C_SLV0_CTRL           	0x27
#define MPU9250_I2C_SLV1_ADDR           	0x28
#define MPU9250_I2C_SLV1_REG            	0x29
#define MPU9250_I2C_SLV1_CTRL           	0x2A
#define MPU9250_I2C_SLV2_ADDR           	0x2B
#define MPU9250_I2C_SLV2_REG            	0x2C
#define MPU9250_I2C_SLV2_CTRL           	0x2D
#define MPU9250_I2C_SLV3_ADDR           	0x2E
#define MPU9250_I2C_SLV3_REG            	0x2F
#define MPU9250_I2C_SLV3_CTRL           	0x30
#define MPU9250_I2C_SLV4_ADDR           	0x31
#define MPU9250_I2C_SLV4_REG            	0x32
#define MPU9250_I2C_SLV4_DO             	0x33
#define MPU9250_I2C_SLV4_CTRL           	0x34
#define MPU9250_I2C_SLV4_DI             	0x35
#define MPU9250_I2C_MST_STATUS          	0x36
#define MPU9250_INT_PIN_CFG             	0x37
#define MPU9250_INT_ENABLE              	0x38
#define MPU9250_DMP_INT_STATUS          	0x39  // Check DMP interrupt
#define MPU9250_INT_STATUS              	0x3A
#define MPU9250_ACCEL_XOUT_H            	0x3B
#define MPU9250_ACCEL_XOUT_L            	0x3C
#define MPU9250_ACCEL_YOUT_H            	0x3D
#define MPU9250_ACCEL_YOUT_L            	0x3E
#define MPU9250_ACCEL_ZOUT_H            	0x3F
#define MPU9250_ACCEL_ZOUT_L            	0x40
#define MPU9250_TEMP_OUT_H              	0x41
#define MPU9250_TEMP_OUT_L              	0x42
#define MPU9250_GYRO_XOUT_H             	0x43
#define MPU9250_GYRO_XOUT_L             	0x44
#define MPU9250_GYRO_YOUT_H             	0x45
#define MPU9250_GYRO_YOUT_L             	0x46
#define MPU9250_GYRO_ZOUT_H             	0x47
#define MPU9250_GYRO_ZOUT_L             	0x48
#define MPU9250_EXT_SENS_DATA_00       		0x49
#define MPU9250_EXT_SENS_DATA_01       		0x4A
#define MPU9250_EXT_SENS_DATA_02       		0x4B
#define MPU9250_EXT_SENS_DATA_03       		0x4C
#define MPU9250_EXT_SENS_DATA_04       		0x4D
#define MPU9250_EXT_SENS_DATA_05       		0x4E
#define MPU9250_EXT_SENS_DATA_06       		0x4F
#define MPU9250_EXT_SENS_DATA_07       		0x50
#define MPU9250_EXT_SENS_DATA_08       		0x51
#define MPU9250_EXT_SENS_DATA_09       		0x52
#define MPU9250_EXT_SENS_DATA_10       		0x53
#define MPU9250_EXT_SENS_DATA_11       		0x54
#define MPU9250_EXT_SENS_DATA_12       		0x55
#define MPU9250_EXT_SENS_DATA_13       		0x56
#define MPU9250_EXT_SENS_DATA_14       		0x57
#define MPU9250_EXT_SENS_DATA_15       		0x58
#define MPU9250_EXT_SENS_DATA_16       		0x59
#define MPU9250_EXT_SENS_DATA_17       		0x5A
#define MPU9250_EXT_SENS_DATA_18       		0x5B
#define MPU9250_EXT_SENS_DATA_19       		0x5C
#define MPU9250_EXT_SENS_DATA_20       		0x5D
#define MPU9250_EXT_SENS_DATA_21       		0x5E
#define MPU9250_EXT_SENS_DATA_22       		0x5F
#define MPU9250_EXT_SENS_DATA_23       		0x60
#define MPU9250_MOT_DETECT_STATUS 			0x61
#define MPU9250_I2C_SLV0_DO             	0x63
#define MPU9250_I2C_SLV1_DO             	0x64
#define MPU9250_I2C_SLV2_DO             	0x65
#define MPU9250_I2C_SLV3_DO             	0x66
#define MPU9250_I2C_MST_DELAY_CTRL   		0x67
#define MPU9250_SIGNAL_PATH_RESET    		0x68
#define MPU9250_MOT_DETECT_CTRL       		0x69
#define MPU9250_USER_CTRL               	0x6A  	// Bit 7 enable DMP, bit 3 reset DMP
#define MPU9250_PWR_MGMT_1              	0x6B 	// Device defaults to the SLEEP mode
#define MPU9250_PWR_MGMT_2              	0x6C
#define MPU9250_DMP_BANK                	0x6D  	// Activates a specific bank in the DMP
#define MPU9250_DMP_RW_PNT              	0x6E  	// Set read/write pointer to a specific start address in specified DMP bank
#define MPU9250_DMP_REG                 	0x6F  	// Register in DMP from which to read or to which to write
#define MPU9250_DMP_REG_1               	0x70
#define MPU9250_DMP_REG_2               	0x71 
#define MPU9250_FIFO_COUNTH             	0x72
#define MPU9250_FIFO_COUNTL             	0x73
#define MPU9250_FIFO_R_W                	0x74
#define MPU9250_WHO_AM_I                	0x75 	// Should return 0x71
#define MPU9250_XA_OFFSET_H             	0x77
#define MPU9250_XA_OFFSET_L             	0x78
#define MPU9250_YA_OFFSET_H             	0x7A
#define MPU9250_YA_OFFSET_L             	0x7B
#define MPU9250_ZA_OFFSET_H             	0x7D
#define MPU9250_ZA_OFFSET_L             	0x7E

//reset values
#define WHOAMI_RESET_VAL                	0x71
#define POWER_MANAGMENT_1_RESET_VAL     	0x01
#define DEFAULT_RESET_VALUE             	0x00

#define WHOAMI_DEFAULT_VAL              	0x68

//CONFIG register masks
#define MPU9250_FIFO_MODE_MASK          	0x40
#define MPU9250_EXT_SYNC_SET_MASK       	0x38
#define MPU9250_DLPF_CFG_MASK           	0x07

//GYRO_CONFIG register masks
#define MPU9250_XGYRO_CTEN_MASK         	0x80
#define MPU9250_YGYRO_CTEN_MASK         	0x40
#define MPU9250_ZGYRO_CTEN_MASK         	0x20
#define MPU9250_GYRO_FS_SEL_MASK        	0x18
#define MPU9250_FCHOICE_B_MASK          	0x03

#define MPU9250_GYRO_FULL_SCALE_250DPS  	0
#define MPU9250_GYRO_FULL_SCALE_500DPS  	1
#define MPU9250_GYRO_FULL_SCALE_1000DPS 	2
#define MPU9250_GYRO_FULL_SCALE_2000DPS 	3

//ACCEL_CONFIG register masks
#define MPU9250_AX_ST_EN_MASK           	0x80
#define MPU9250_AY_ST_EN_MASK           	0x40
#define MPU9250_AZ_ST_EN_MASK           	0x20
#define MPU9250_ACCEL_FS_SEL_MASK       	0x18

#define MPU9250_FULL_SCALE_2G           	0
#define MPU9250_FULL_SCALE_4G           	1
#define MPU9250_FULL_SCALE_8G           	2
#define MPU9250_FULL_SCALE_16G          	3

//ACCEL_CONFIG_2 register masks
#define MPU9250_ACCEL_FCHOICE_B_MASK    	0xC0
#define MPU9250_A_DLPF_CFG_MASK         	0x03

//LP_ACCEL_ODR register masks
#define MPU9250_LPOSC_CLKSEL_MASK       	0x0F

//FIFO_EN register masks
#define MPU9250_TEMP_FIFO_EN_MASK       	0x80
#define MPU9250_GYRO_XOUT_MASK          	0x40
#define MPU9250_GYRO_YOUT_MASK          	0x20
#define MPU9250_GYRO_ZOUT_MASK          	0x10
#define MPU9250_ACCEL_MASK              	0x08
#define MPU9250_SLV2_MASK               	0x04
#define MPU9250_SLV1_MASK               	0x02
#define MPU9250_SLV0_MASK               	0x01

//I2C_MST_CTRL register masks
#define MPU9250_MULT_MST_EN_MASK        	0x80
#define MPU9250_WAIT_FOR_ES_MASK        	0x40
#define MPU9250_SLV_3_FIFO_EN_MASK      	0x20
#define MPU9250_I2C_MST_P_NSR_MASK      	0x10
#define MPU9250_I2C_MST_CLK_MASK        	0x0F

//I2C_SLV0_ADDR register masks
#define MPU9250_I2C_SLV0_RNW_MASK       	0x80
#define MPU9250_I2C_ID_0_MASK           	0x7F

//I2C_SLV0_CTRL register masks
#define MPU9250_I2C_SLV0_EN_MASK        	0x80
#define MPU9250_I2C_SLV0_BYTE_SW_MASK   	0x40
#define MPU9250_I2C_SLV0_REG_DIS_MASK   	0x20
#define MPU9250_I2C_SLV0_GRP_MASK       	0x10
#define MPU9250_I2C_SLV0_LENG_MASK      	0x0F

//I2C_SLV1_ADDR register masks
#define MPU9250_I2C_SLV1_RNW_MASK       	0x80
#define MPU9250_I2C_ID_1_MASK           	0x7F

//I2C_SLV1_CTRL register masks
#define MPU9250_I2C_SLV1_EN_MASK        	0x80
#define MPU9250_I2C_SLV1_BYTE_SW_MASK   	0x40
#define MPU9250_I2C_SLV1_REG_DIS_MASK   	0x20
#define MPU9250_I2C_SLV1_GRP_MASK       	0x10
#define MPU9250_I2C_SLV1_LENG_MASK      	0x0F

//I2C_SLV2_ADDR register masks
#define MPU9250_I2C_SLV2_RNW_MASK       	0x80
#define MPU9250_I2C_ID_2_MASK           	0x7F

//I2C_SLV2_CTRL register masks
#define MPU9250_I2C_SLV2_EN_MASK        	0x80
#define MPU9250_I2C_SLV2_BYTE_SW_MASK   	0x40
#define MPU9250_I2C_SLV2_REG_DIS_MASK   	0x20
#define MPU9250_I2C_SLV2_GRP_MASK       	0x10
#define MPU9250_I2C_SLV2_LENG_MASK      	0x0F

//I2C_SLV3_ADDR register masks
#define MPU9250_I2C_SLV3_RNW_MASK       	0x80
#define MPU9250_I2C_ID_3_MASK           	0x7F

//I2C_SLV3_CTRL register masks
#define MPU9250_I2C_SLV3_EN_MASK        	0x80
#define MPU9250_I2C_SLV3_BYTE_SW_MASK   	0x40
#define MPU9250_I2C_SLV3_REG_DIS_MASK   	0x20
#define MPU9250_I2C_SLV3_GRP_MASK       	0x10
#define MPU9250_I2C_SLV3_LENG_MASK      	0x0F

//I2C_SLV4_ADDR register masks
#define MPU9250_I2C_SLV4_RNW_MASK       	0x80
#define MPU9250_I2C_ID_4_MASK           	0x7F

//I2C_SLV4_CTRL register masks
#define MPU9250_I2C_SLV4_EN_MASK        	0x80
#define MPU9250_SLV4_DONE_INT_EN_MASK   	0x40
#define MPU9250_I2C_SLV4_REG_DIS_MASK   	0x20
#define MPU9250_I2C_MST_DLY_MASK        	0x1F

//I2C_MST_STATUS register masks
#define MPU9250_PASS_THROUGH_MASK       	0x80
#define MPU9250_I2C_SLV4_DONE_MASK      	0x40
#define MPU9250_I2C_LOST_ARB_MASK       	0x20
#define MPU9250_I2C_SLV4_NACK_MASK      	0x10
#define MPU9250_I2C_SLV3_NACK_MASK      	0x08
#define MPU9250_I2C_SLV2_NACK_MASK      	0x04
#define MPU9250_I2C_SLV1_NACK_MASK      	0x02
#define MPU9250_I2C_SLV0_NACK_MASK      	0x01

//INT_PIN_CFG register masks
#define MPU9250_ACTL_MASK               	0x80
#define MPU9250_OPEN_MASK               	0x40
#define MPU9250_LATCH_INT_EN_MASK       	0x20
#define MPU9250_INT_ANYRD_2CLEAR_MASK   	0x10
#define MPU9250_ACTL_FSYNC_MASK         	0x08
#define MPU9250_FSYNC_INT_MODE_EN_MASK  	0x04
#define MPU9250_BYPASS_EN_MASK          	0x02

//INT_ENABLE register masks
#define MPU9250_WOM_EN_MASK             	0x40
#define MPU9250_FIFO_OFLOW_EN_MASK      	0x10
#define MPU9250_FSYNC_INT_EN_MASK       	0x08
#define MPU9250_RAW_RDY_EN_MASK         	0x01

//INT_STATUS register masks
#define MPU9250_WOM_INT_MASK            	0x40
#define MPU9250_FIFO_OFLOW_INT_MASK     	0x10
#define MPU9250_FSYNC_INT_MASK          	0x08
#define MPU9250_RAW_DATA_RDY_INT_MASK   	0x01

//I2C_MST_DELAY_CTRL register masks
#define MPU9250_DELAY_ES_SHADOW_MASK    	0x80
#define MPU9250_I2C_SLV4_DLY_EN_MASK    	0x10
#define MPU9250_I2C_SLV3_DLY_EN_MASK    	0x08
#define MPU9250_I2C_SLV2_DLY_EN_MASK    	0x04
#define MPU9250_I2C_SLV1_DLY_EN_MASK    	0x02
#define MPU9250_I2C_SLV0_DLY_EN_MASK    	0x01

//SIGNAL_PATH_RESET register masks
#define MPU9250_GYRO_RST_MASK           	0x04
#define MPU9250_ACCEL_RST_MASK          	0x02
#define MPU9250_TEMP_RST_MASK           	0x01

//MOT_DETECT_CTRL register masks
#define MPU9250_ACCEL_INTEL_EN_MASK     	0x80
#define MPU9250_ACCEL_INTEL_MODE_MASK   	0x40

//USER_CTRL register masks
#define MPU9250_FIFO_EN_MASK            	0x40
#define MPU9250_I2C_MST_EN_MASK         	0x20
#define MPU9250_I2C_IF_DIS_MASK         	0x10
#define MPU9250_FIFO_RST_MASK           	0x04
#define MPU9250_I2C_MST_RST_MASK        	0x02
#define MPU9250_SIG_COND_RST_MASK       	0x01

//PWR_MGMT_1 register masks
#define MPU9250_H_RESET_MASK            	0x80
#define MPU9250_SLEEP_MASK              	0x40
#define MPU9250_CYCLE_MASK              	0x20
#define MPU9250_GYRO_STANDBY_CYCLE_MASK 	0x10
#define MPU9250_PD_PTAT_MASK            	0x08
#define MPU9250_CLKSEL_MASK             	0x07

//PWR_MGMT_2 register masks
#define MPU9250_DISABLE_XA_MASK         	0x20
#define MPU9250_DISABLE_YA_MASK         	0x10
#define MPU9250_DISABLE_ZA_MASK         	0x08
#define MPU9250_DISABLE_XG_MASK         	0x04
#define MPU9250_DISABLE_YG_MASK         	0x02
#define MPU9250_DISABLE_ZG_MASK         	0x01

#define MPU9250_DISABLE_XYZA_MASK       	0x38
#define MPU9250_DISABLE_XYZG_MASK       	0x07

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, 	// 0.6 mG per LSB
  MFS_16BITS      	// 0.15 mG per LSB
};

#define MPU9250_ACC_LPM           	0x01
#define MPU9250_GYRO            	0x02
#define MPU9250_ACC_GYRO      		0x03
#define MPU9250_ORIENTATION     	0x04
#define MPU9250_MOTION          	0x05

void nrf_drv_twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void twi_init(void);

void MPU9250_Process(void);
                       
void AK8963_GetMres(void);
void MPU9250_GetGres(void);
void MPU9250_GetAres(void) ;
void MPU9250_ReadAccelData(int16_t * destination);
void MPU9250_ReadGyroData(int16_t * destination);
void AK8963_ReadMagData(int16_t * destination);
int16_t MPU9250_ReadTempData(void);
void MPU9250_Reset(void);
void AK8963_Active(void);
void AK8963_PowerDown(void);
void AK8963_Init(float * destination);
void MPU9250_Sleep(void);
void MPU9250_Active(void);
void MPU9250_InitVar(void);
void MPU9250_Init(uint8_t Mode);
void MPU9250_Calibrate(float * dest1, float * dest2);
void MPU9250_SelfTest(float * destination);
void MPU9250_MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MPU9250_MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void Init_MPU9250_DRDY_Interrupt(void);
void Enable_MPU9250_DRDY_Interrupt(void);
void Disable_MPU9250_DRDY_Interrupt(void);

ret_code_t MPU9250_ReadBytes(uint8_t Register, uint8_t Length, uint8_t *MPU9250_Rx_Buffer);//
ret_code_t MPU9250_WriteBytes(uint8_t Register, uint8_t Length, uint8_t *MPU9250_Tx_Buffer);
uint8_t MPU9250_ReadByte(uint8_t Register);//
ret_code_t MPU9250_WriteByte(uint8_t Register, uint8_t RegValue);//

ret_code_t AK8963_ReadBytes(uint8_t Register, uint8_t Length, uint8_t *AK8963_Rx_Buffer);//
uint8_t AK8963_ReadByte(uint8_t Register);//
ret_code_t AK8963_WriteByte(uint8_t Register, uint8_t RegValue);

float invSqrt(float x);

void MPU9250_OneStepProcess(void);
float Caltilt(float AccX, float AccY, float AccZ);
void FallDetection(float AccX, float AccY, float AccZ);
void ActivityStage(void);

void MPU9250_MotionStage(void);


#endif 
