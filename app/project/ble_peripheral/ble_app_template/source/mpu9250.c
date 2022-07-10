#include "mpu9250.h"
#include <math.h>

/* TWI instance. */
static const nrf_drv_twi_t	m_twi_mpu_9250 = NRF_DRV_TWI_INSTANCE(1);

static const nrf_drv_twi_t	m_twi_ak_8963 = NRF_DRV_TWI_INSTANCE(1);

MPU9250Variables MPU9250;
AK8963Variables AK8963;

uint8_t Ascale = AFS_2G;        	/* AFS_2G, AFS_4G, AFS_8G, AFS_16G */
uint8_t Gscale = GFS_500DPS;    	/* GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS */
uint8_t Mscale = MFS_16BITS;    	/* MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution */
uint8_t Mmode = 0x06;           	/* Either 8 Hz (0x02) or 100 Hz (0x06) magnetometer data ODR */  
float aRes, gRes, mRes;         	/* scale resolutions per LSB for the sensors */

uint16_t sampleFreq = 200;

/* parameters for 6 DoF sensor fusion calculations */
float PI = 3.14159265358979323846f;
float GyroMeasError = 0.0f;      		/* gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3 */
float beta = 0.0f;              		/* compute beta */
float GyroMeasDrift = 0.0f;     		/* gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s) */
float zeta = 0.0f;              		/* compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value */
#define Kp   (2.0f * 5.0f)   			/* these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral */
#define Ki    0.0f

#define aaaRes  (2.0f / 32768.0f) 

float pitch = 0.0f, yaw = 0.0f, roll = 0.0f;
float deltat = 0.0f;                         	/* integration interval for both filter schemes */
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  /* vector to hold quaternion */
float eInt[3] = {0.0f, 0.0f, 0.0f};      /* vector to hold integral error for Mahony method */

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

extern volatile uint16_t Motion_TimeOut;
extern uint8_t Device_Number[6];

void mpu9250_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//MPU9250_OneStepProcess();
}

void MPU9250_Process(void)
{
	uint8_t Status_Reg = 0;
  
	Status_Reg = MPU9250_ReadByte(MPU9250_INT_STATUS);
	
	switch(MPU9250.Mode)
	{
		case MPU9250_ACC_LPM :
			if( Status_Reg & 0x01 ) 
			{  				
			}
			break;
		
		case MPU9250_ACC_GYRO :
			if( Status_Reg & 0x01 ) 
			{  
			}
			break;
			
		case MPU9250_ORIENTATION :
			if( Status_Reg & 0x01 ) 
			{  
			}
			break;
		
		case MPU9250_MOTION :
			if( Status_Reg & 0x40 )  
			{  
			}
			break;
			
		default:
			break;
	}
}


void MPU9250_OneStepProcess(void)
{
	uint8_t Status_Reg=0, an=0;
	int32_t xyzbuf=0, axTemp = 0, ayTemp = 0, azTemp = 0;
 
	Status_Reg = MPU9250_ReadByte(MPU9250_INT_STATUS);
	
	if( Status_Reg & 0x01 ) 
	{  	
		MPU9250_ReadAccelData(MPU9250.AccelCount);  

		MPU9250.mxAcc[3] = MPU9250.AccelCount[0];
		MPU9250.myAcc[3] = MPU9250.AccelCount[1];
		MPU9250.mzAcc[3] = MPU9250.AccelCount[2];
		
		axTemp = MPU9250.mxAcc[0];
		ayTemp = MPU9250.myAcc[0];
		azTemp = MPU9250.mzAcc[0];	
		for(an=1;an<4;an++){
            axTemp = axTemp + MPU9250.mxAcc[an];
            ayTemp = ayTemp + MPU9250.myAcc[an];
            azTemp = azTemp + MPU9250.mzAcc[an];
        
            MPU9250.mxAcc[an-1] = MPU9250.mxAcc[an];
            MPU9250.myAcc[an-1] = MPU9250.myAcc[an];
            MPU9250.mzAcc[an-1] = MPU9250.mzAcc[an];
		}
		
		axTemp=axTemp/4;
		ayTemp=ayTemp/4;
		azTemp=azTemp/4;
		
//----------------------------------------------------------
		
		MPU9250.fxTemp = (float)axTemp*aaaRes;  
		MPU9250.fyTemp = (float)ayTemp*aaaRes;   
		MPU9250.fzTemp = (float)azTemp*aaaRes;   
		
//----------------------------------------------------------
		
		xyzbuf = axTemp-MPU9250.baccx;
		if(xyzbuf<0) xyzbuf=xyzbuf*-1;
		MPU9250.dmxAcc[3] = xyzbuf;
		
		xyzbuf = ayTemp-MPU9250.baccy;
		if(xyzbuf<0) xyzbuf=xyzbuf*-1;
		MPU9250.dmyAcc[3] = xyzbuf;

		xyzbuf = azTemp-MPU9250.baccz;
		if(xyzbuf<0) xyzbuf=xyzbuf*-1;
		MPU9250.dmzAcc[3] = xyzbuf;		

		MPU9250.baccx = axTemp;
		MPU9250.baccy = ayTemp;
		MPU9250.baccz = azTemp;
		
		axTemp = MPU9250.dmxAcc[0];
		ayTemp = MPU9250.dmyAcc[0];
		azTemp = MPU9250.dmzAcc[0];					
		for(an=1;an<4;an++){
            axTemp = axTemp + MPU9250.dmxAcc[an];
            ayTemp = ayTemp + MPU9250.dmyAcc[an];
            azTemp = azTemp + MPU9250.dmzAcc[an];
        
            MPU9250.dmxAcc[an-1] = MPU9250.dmxAcc[an];
            MPU9250.dmyAcc[an-1] = MPU9250.dmyAcc[an];
            MPU9250.dmzAcc[an-1] = MPU9250.dmzAcc[an];
		}
		axTemp=axTemp/4;
		ayTemp=ayTemp/4;
		azTemp=azTemp/4;
        
        if( axTemp > 32000 ) MPU9250.Xaxis = 32000;
        else if( axTemp < -32000 ) MPU9250.Xaxis = -32000;
        
        if( ayTemp > 32000 ) MPU9250.Yaxis = 32000;
        else if( ayTemp < -32000 ) MPU9250.Yaxis = -32000;
        
        if( axTemp > 32000 ) MPU9250.Zaxis = 32000;
        else if( axTemp < -32000 ) MPU9250.Xaxis = -32000;
		
        
        MPU9250.Xaxis = (int16_t)axTemp;
        MPU9250.Yaxis = (int16_t)ayTemp;
        MPU9250.Zaxis = (int16_t)azTemp;
        
        MPU9250.nSampleCnt++;
        
        MPU9250.xTxBuff[(MPU9250.nSampleCnt-1)*2]   = (uint8_t)( (MPU9250.Xaxis >> 8) & 0xFF);
        MPU9250.xTxBuff[(MPU9250.nSampleCnt-1)*2+1] = (uint8_t)( (MPU9250.Xaxis & 0xFF) );
        
        MPU9250.yTxBuff[(MPU9250.nSampleCnt-1)*2]   = (uint8_t)( (MPU9250.Yaxis >> 8) & 0xFF);
        MPU9250.yTxBuff[(MPU9250.nSampleCnt-1)*2+1] = (uint8_t)( (MPU9250.Yaxis & 0xFF) );
        
        MPU9250.zTxBuff[(MPU9250.nSampleCnt-1)*2]   = (uint8_t)( (MPU9250.Zaxis >> 8) & 0xFF);
        MPU9250.zTxBuff[(MPU9250.nSampleCnt-1)*2+1] = (uint8_t)( (MPU9250.Zaxis & 0xFF) );
        
        if( MPU9250.nSampleCnt >= 10 ) 
        {
            MPU9250.nSampleCnt = 0;
        }
//-------------------------------------------------------------------------------------------
	}
}


void AK8963_GetMres(void) 
{
	switch (Mscale)
	{
		/* Possible magnetometer scales (and their register bit settings) are:
	       14 bit resolution (0) and 16 bit resolution (1) */
		case MFS_14BITS:
			mRes = 10.0*4219.0/8190.0;        /* Proper scale to return milliGauss */
			break;
		case MFS_16BITS:
			mRes = 10.0*4219.0/32760.0;       /* Proper scale to return milliGauss */
			break;
	}
}

void MPU9250_GetGres(void) 
{
	switch (Gscale)
	{
		/* Possible gyro scales (and their register bit settings) are:
		250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
		Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value: */
		case GFS_250DPS:
			gRes = 250.0/32768.0;
			break;
		case GFS_500DPS:
			gRes = 500.0/32768.0;
			break;
		case GFS_1000DPS:
			gRes = 1000.0/32768.0;
			break;
		case GFS_2000DPS:
			gRes = 2000.0/32768.0;
			break;
	}
}

void MPU9250_GetAres(void) 
{
	switch (Ascale)
	{
		/* Possible accelerometer scales (and their register bit settings) are:
		   2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
		   Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value: */
		case AFS_2G:
			aRes = 2.0/32768.0;
		break;
		case AFS_4G:
			aRes = 4.0/32768.0;
		break;
		case AFS_8G:
			aRes = 8.0/32768.0;
		break;
		case AFS_16G:
			aRes = 16.0/32768.0;
		break;
	}
}

void MPU9250_ReadAccelData(int16_t * destination)
{
	uint8_t rawData[6];   /* x/y/z accel register data stored here */

	MPU9250_ReadBytes(MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);      /* Read the six raw data registers into data array */

	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; /* Turn the MSB and LSB into a signed 16-bit value */
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}

void MPU9250_ReadGyroData(int16_t * destination)
{
	uint8_t rawData[6];   /* x/y/z gyro register data stored here */

	MPU9250_ReadBytes(MPU9250_GYRO_XOUT_H, 6, &rawData[0]);       /* Read the six raw data registers sequentially into data array */

	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; /* Turn the MSB and LSB into a signed 16-bit value */
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}

void AK8963_ReadMagData(int16_t * destination)
{
	uint8_t rawData[7];  /* x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition */

	if(AK8963_ReadByte(AK8963_ST1) & 0x01)        /* wait for magnetometer data ready bit to be set */
	{ 
		AK8963_ReadBytes(AK8963_XOUT_L, 7, &rawData[0]);    /* Read the six raw data and ST2 registers sequentially into data array */
		uint8_t c = rawData[6];     /* End data read by reading ST2 register */
		if(!(c & 0x08)) 
		{ 
			/* Check if magnetic sensor overflow set, if not then report data */
			destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);      /* Turn the MSB and LSB into a signed 16-bit value */
			destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;     /* Data stored as little Endian */
			destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ; 
		}
	}
}

int16_t MPU9250_ReadTempData(void)
{
	uint8_t rawData[2];  /* x/y/z gyro register data stored here */

	MPU9250_ReadBytes(MPU9250_TEMP_OUT_H, 2, &rawData[0]);        /* Read the two raw data registers sequentially into data array */
	return (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]) ;   /* Turn the MSB and LSB into a 16-bit value */
}

void MPU9250_Sleep(void)
{
	uint8_t n;

	n = MPU9250_ReadByte(MPU9250_PWR_MGMT_1);
	MPU9250_WriteByte(MPU9250_PWR_MGMT_1, (n | 0x40));
}

void MPU9250_Active(void)
{
	uint8_t n;

	n = MPU9250_ReadByte(MPU9250_PWR_MGMT_1);
	MPU9250_WriteByte(MPU9250_PWR_MGMT_1, n & (~0x40));
}

void MPU9250_Reset(void) 
{
	MPU9250_WriteByte(MPU9250_PWR_MGMT_1, 0x80);  /* Write a one to bit 7 reset bit; toggle reset device */
	//nrf_delay_ms(100);	
}

void AK8963_Active(void)
{
	/* Configure the magnetometer for continuous read and highest resolution
	   set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	   and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates */
	AK8963_WriteByte(AK8963_CNTL, Mscale << 4 | Mmode); /* Set magnetometer data resolution and sample ODR */ 
}
  
void AK8963_PowerDown(void)
{ 
	AK8963_WriteByte(AK8963_CNTL, 0x00);  /* Power down magnetometer */  
}

void AK8963_Init(float * destination)
{
	/* First extract the factory calibration for each magnetometer axis */
	uint8_t rawData[3];   /* x/y/z gyro calibration data stored here */

	AK8963_WriteByte(AK8963_CNTL, 0x00);  /* Power down magnetometer */  
	nrf_delay_ms(10);
	AK8963_WriteByte(AK8963_CNTL, 0x0F);  /* Enter Fuse ROM access mode */
	nrf_delay_ms(10);

	AK8963_ReadBytes(AK8963_ASAX, 3, &rawData[0]);                /* Read the x-, y-, and z-axis calibration values */
	destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;    /* Return x-axis sensitivity adjustment values, etc. */
	destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
	destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 

	AK8963_WriteByte(AK8963_CNTL, 0x00);  /* Power down magnetometer */  
	nrf_delay_ms(10);

	AK8963_WriteByte(AK8963_CNTL, Mscale << 4 | Mmode); /* Set magnetometer data resolution and sample ODR */
	nrf_delay_ms(10);
}

void MPU9250_InitVar(void)
{  
	ClearMem((unsigned char *)&MPU9250, sizeof(MPU9250Variables));

	GyroMeasError = PI * (60.0f / 180.0f);        /* gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3 */
	beta = sqrt(3.0f / 4.0f) * GyroMeasError;     /* compute beta */
	GyroMeasDrift = PI * (1.0f / 180.0f);         /* gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s) */
	zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;     /* compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value */

	deltat = 0.125f;
}

void MPU9250_Init(uint8_t Mode)
{  
	uint8_t Reg; 

	switch(Mode)
	{
		case MPU9250_ACC_LPM :
			MPU9250.Mode = MPU9250_ACC_LPM;
		
			MPU9250_WriteByte(MPU9250_PWR_MGMT_1, 0x80);  /* Write a one to bit 7 reset bit; toggle reset device */
			nrf_delay_ms(100);   /* Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt */

			MPU9250_WriteByte(MPU9250_PWR_MGMT_1, 0x48);  
			//MPU9250_WriteByte(MPU9250_PWR_MGMT_1, MPU9250_CYCLE_MASK);
			//MPU9250_WriteByte(MPU9250_PWR_MGMT_2, MPU9250_DISABLE_XYZG_MASK);
		
			MPU9250_WriteByte(MPU9250_PWR_MGMT_2, MPU9250_DISABLE_XYZG_MASK);
			
			MPU9250_WriteByte(MPU9250_LP_ACCEL_ODR, 0x09);  /* Low power accel Output Data Rate */		
	
			/* Set accelerometer full-scale range configuration */
			Reg = MPU9250_ReadByte(MPU9250_ACCEL_CONFIG); /* get current ACCEL_CONFIG register value */
			Reg = Reg & ~0x18;        /* Clear AFS bits [4:3] */
			Reg = Reg | (Ascale << 3);  /* Set full scale range for the accelerometer */
			MPU9250_WriteByte(MPU9250_ACCEL_CONFIG, Reg);   /* Write new ACCEL_CONFIG register value */

//			/* Set accelerometer sample rate configuration
//			 It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
//			 accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz */
//			Reg = MPU9250_ReadByte(MPU9250_ACCEL_CONFIG2);  /* get current ACCEL_CONFIG2 register value */
//			Reg = Reg & ~0x0F;        /* Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0]) */  
//			Reg = Reg | 0x03;         /* Set accelerometer rate to 1 kHz and bandwidth to 41 Hz */
//			MPU9250_WriteByte(MPU9250_ACCEL_CONFIG2, Reg);  /* Write new ACCEL_CONFIG2 register value */

			/* The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
			 but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting */

			/* Configure Interrupts and Bypass Enable
			 Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
			 can join the I2C bus and all can be controlled by the Arduino as master */
			 
			MPU9250_WriteByte(MPU9250_INT_PIN_CFG, 0x80);  			
			MPU9250_WriteByte(MPU9250_INT_ENABLE, 0x00);  /* Enable data ready (bit 0) interrupt */
			
			Init_MPU9250_DRDY_Interrupt();
			
			break;
		case MPU9250_ACC_GYRO :
			MPU9250.Mode = MPU9250_ACC_GYRO;
		
			/* Enable Acc & Gyro */
			MPU9250_WriteByte(MPU9250_PWR_MGMT_2, 0x00);

			/* Configure Gyro and Accelerometer
			 Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
			 DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
			 Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate */
			MPU9250_WriteByte(MPU9250_CONFIG, 0x03);  

			/* Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) */
			MPU9250_WriteByte(MPU9250_SMPLRT_DIV, 0x13);  /* Use a 200 Hz rate; the same rate set in CONFIG above */

			/* Set gyroscope full scale range */
			/* Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3 */
			Reg = MPU9250_ReadByte(MPU9250_GYRO_CONFIG);    /* get current GYRO_CONFIG register value */
			// Reg = Reg & ~0xE0;     /* Clear self-test bits [7:5] */ 
			Reg = Reg & ~0x02;        /* Clear Fchoice bits [1:0] */
			Reg = Reg & ~0x18;        /* Clear AFS bits [4:3] */
			Reg = Reg | (Gscale << 3);  /* Set full scale range for the gyro */
			// Reg =| 0x00;         /* Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG */
			MPU9250_WriteByte(MPU9250_GYRO_CONFIG, Reg);   /* Write new GYRO_CONFIG value to register */

			/* Set accelerometer full-scale range configuration */
			Reg = MPU9250_ReadByte(MPU9250_ACCEL_CONFIG); /* get current ACCEL_CONFIG register value */
			// Reg = Reg & ~0xE0;     /* Clear self-test bits [7:5] */ 
			Reg = Reg & ~0x18;        /* Clear AFS bits [4:3] */
			Reg = Reg | (Ascale << 3);  /* Set full scale range for the accelerometer */
			MPU9250_WriteByte(MPU9250_ACCEL_CONFIG, Reg);   /* Write new ACCEL_CONFIG register value */

			/* Set accelerometer sample rate configuration
			 It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
			 accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz */
			Reg = MPU9250_ReadByte(MPU9250_ACCEL_CONFIG2);  /* get current ACCEL_CONFIG2 register value */
			Reg = Reg & ~0x0F;        /* Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0]) */  
			Reg = Reg | 0x03;         /* Set accelerometer rate to 1 kHz and bandwidth to 41 Hz */
			MPU9250_WriteByte(MPU9250_ACCEL_CONFIG2, Reg);  /* Write new ACCEL_CONFIG2 register value */

			/* The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
			 but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting */

			/* Configure Interrupts and Bypass Enable
			 Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
			 can join the I2C bus and all can be controlled by the Arduino as master */
			MPU9250_WriteByte(MPU9250_INT_PIN_CFG, 0x80);    

			MPU9250_WriteByte(MPU9250_INT_ENABLE, 0x01);  /* Enable data ready (bit 0) interrupt */

			Init_MPU9250_DRDY_Interrupt();
			break;
		  
		case MPU9250_ORIENTATION :
			MPU9250.Mode = MPU9250_ORIENTATION;
		
		//	MPU9250_WriteByte(MPU9250_PWR_MGMT_1, 0x00);  /* Clear sleep mode bit (6), enable all sensors */
		//	MPU9250_WriteByte(MPU9250_PWR_MGMT_1, 0x01);  /* Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 */
		
		
			MPU9250_WriteByte(MPU9250_CONFIG, 0x03);  
			MPU9250_WriteByte(MPU9250_SMPLRT_DIV, 0x04);  /* Use a 200 Hz rate; the same rate set in CONFIG above */

			/* Set gyroscope full scale range */
			Reg = MPU9250_ReadByte(MPU9250_GYRO_CONFIG);    /* get current GYRO_CONFIG register value */
			Reg = Reg & ~0x02;        /* Clear Fchoice bits [1:0] */
			Reg = Reg & ~0x18;        /* Clear AFS bits [4:3] */
			Reg = Reg | (Gscale << 3);  /* Set full scale range for the gyro */
			MPU9250_WriteByte(MPU9250_GYRO_CONFIG, Reg);   /* Write new GYRO_CONFIG value to register */

			/* Set accelerometer full-scale range configuration */
			Reg = MPU9250_ReadByte(MPU9250_ACCEL_CONFIG); /* get current ACCEL_CONFIG register value */
			Reg = Reg & ~0x18;        /* Clear AFS bits [4:3] */
			Reg = Reg | (Ascale << 3);  /* Set full scale range for the accelerometer */
			MPU9250_WriteByte(MPU9250_ACCEL_CONFIG, Reg);   /* Write new ACCEL_CONFIG register value */

			/* Set accelerometer sample rate configuration*/
			Reg = MPU9250_ReadByte(MPU9250_ACCEL_CONFIG2);  /* get current ACCEL_CONFIG2 register value */
			Reg = Reg & ~0x0F;        /* Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0]) */  
			Reg = Reg | 0x03;         /* Set accelerometer rate to 1 kHz and bandwidth to 41 Hz */
			MPU9250_WriteByte(MPU9250_ACCEL_CONFIG2, Reg);  /* Write new ACCEL_CONFIG2 register value */

			MPU9250_WriteByte(MPU9250_INT_PIN_CFG, 0x82);    
			MPU9250_WriteByte(MPU9250_INT_ENABLE, 0x01);  /* Enable data ready (bit 0) interrupt */
			
			Init_MPU9250_DRDY_Interrupt();
			
			break;  
			
		case MPU9250_MOTION :
			MPU9250.Mode = MPU9250_MOTION;
		
		//	MPU9250_WriteByte(MPU9250_PWR_MGMT_1, 0x00);  /* Clear sleep mode bit (6), enable all sensors */
		//	MPU9250_WriteByte(MPU9250_PWR_MGMT_1, 0x01);  /* Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 */
		
		
			/* Enable Acc & Gyro */
			MPU9250_WriteByte(MPU9250_PWR_MGMT_2, 0x00);

			/* Configure Gyro and Accelerometer
			 Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
			 DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
			 Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate */
			MPU9250_WriteByte(MPU9250_CONFIG, 0x03);  

			/* Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) */
			MPU9250_WriteByte(MPU9250_SMPLRT_DIV, 0xC7);  /* Use a 200 Hz rate; the same rate set in CONFIG above */

			/* Set gyroscope full scale range */
			/* Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3 */
//			Reg = MPU9250_ReadByte(MPU9250_GYRO_CONFIG);    /* get current GYRO_CONFIG register value */
//			// Reg = Reg & ~0xE0;     /* Clear self-test bits [7:5] */ 
//			Reg = Reg & ~0x02;        /* Clear Fchoice bits [1:0] */
//			Reg = Reg & ~0x18;        /* Clear AFS bits [4:3] */
//			Reg = Reg | (Gscale << 3);  /* Set full scale range for the gyro */
//			// Reg =| 0x00;         /* Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG */
//			MPU9250_WriteByte(MPU9250_GYRO_CONFIG, Reg);   /* Write new GYRO_CONFIG value to register */

			/* Set accelerometer full-scale range configuration */
			Reg = MPU9250_ReadByte(MPU9250_ACCEL_CONFIG); /* get current ACCEL_CONFIG register value */
			// Reg = Reg & ~0xE0;     /* Clear self-test bits [7:5] */ 
			Reg = Reg & ~0x18;        /* Clear AFS bits [4:3] */
			Reg = Reg | (Ascale << 3);  /* Set full scale range for the accelerometer */
			MPU9250_WriteByte(MPU9250_ACCEL_CONFIG, Reg);   /* Write new ACCEL_CONFIG register value */

			/* Set accelerometer sample rate configuration
			 It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
			 accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz */
			Reg = MPU9250_ReadByte(MPU9250_ACCEL_CONFIG2);  /* get current ACCEL_CONFIG2 register value */
			Reg = Reg & ~0x0F;        /* Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0]) */  
			Reg = Reg | 0x03;         /* Set accelerometer rate to 1 kHz and bandwidth to 41 Hz */
			MPU9250_WriteByte(MPU9250_ACCEL_CONFIG2, Reg);  /* Write new ACCEL_CONFIG2 register value */

			/* The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
			 but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting */

			/* Wake-on Motion Threshold
			 This register holds the threshold value for the Wake on Motion Interrupt for accel x/y/z axes. LSB = 4mg. Range is 0mg to 1020mg. */
			Reg = 0x32;
			MPU9250_WriteByte(MPU9250_WOM_THR, Reg);  /* Write new WOM_THR register value */

			/* Accelerometer Interrupt Control
			 bit[7] : This bit enables the Wake-on-Motion detection logic. bit[6] : 1 = Compare the current sample with the previous sample. 0 = Not used */ 
			Reg = 0xC0;
			MPU9250_WriteByte(MPU9250_MOT_DETECT_CTRL, Reg);  

			/* Configure Interrupts and Bypass Enable
			 Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
			 can join the I2C bus and all can be controlled by the Arduino as master */
			MPU9250_WriteByte(MPU9250_INT_PIN_CFG, 0x80);    

			MPU9250_WriteByte(MPU9250_INT_ENABLE, 0x40);  /* Enable data ready (bit 0) interrupt */

			Init_MPU9250_DRDY_Interrupt();
			break;
		default:
			break;	
	}
}

/* Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
   of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers. */
void MPU9250_Calibrate(float * dest1, float * dest2)
{  
	uint8_t data[12];     /* data array to hold accelerometer and gyro x, y, z, data */
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	/* reset device, reset all registers, clear gyro and accelerometer bias registers */
	MPU9250_WriteByte(MPU9250_PWR_MGMT_1, 0x80);  /* Write a one to bit 7 reset bit; toggle reset device */
	nrf_delay_ms(100);  

	/* get stable time source 
	 Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 */
	MPU9250_WriteByte(MPU9250_PWR_MGMT_1, 0x01);  
	MPU9250_WriteByte(MPU9250_PWR_MGMT_2, 0x00); 
	nrf_delay_ms(200);

	/* Configure device for bias calculation */
	MPU9250_WriteByte(MPU9250_INT_ENABLE, 0x00);          /* Disable all interrupts */
	MPU9250_WriteByte(MPU9250_FIFO_EN, 0x00);             /* Disable FIFO */
	MPU9250_WriteByte(MPU9250_PWR_MGMT_1, 0x00);          /* Turn on internal clock source */
	MPU9250_WriteByte(MPU9250_I2C_MST_CTRL, 0x00);        /* Disable I2C master */
	MPU9250_WriteByte(MPU9250_USER_CTRL, 0x00);           /* Disable FIFO and I2C master modes */
	MPU9250_WriteByte(MPU9250_USER_CTRL, 0x0C);           /* Reset FIFO and DMP */
	nrf_delay_ms(15);

	/* Configure MPU9250 gyro and accelerometer for bias calculation */
	MPU9250_WriteByte(MPU9250_CONFIG, 0x01);              /* Set low-pass filter to 188 Hz */
	MPU9250_WriteByte(MPU9250_SMPLRT_DIV, 0x00);          /* Set sample rate to 1 kHz */
	MPU9250_WriteByte(MPU9250_GYRO_CONFIG, 0x00);         /* Set gyro full-scale to 250 degrees per second, maximum sensitivity */
	MPU9250_WriteByte(MPU9250_ACCEL_CONFIG, 0x00);        /* Set accelerometer full-scale to 2 g, maximum sensitivity */

	uint16_t  gyrosensitivity  = 131;     /* = 131 LSB/degrees/sec */
	uint16_t  accelsensitivity = 16384;   /* = 16384 LSB/g */

	/* Configure FIFO to capture accelerometer and gyro data for bias calculation */
	MPU9250_WriteByte(MPU9250_USER_CTRL, 0x40);           /* Enable FIFO */  
	MPU9250_WriteByte(MPU9250_FIFO_EN, 0x78);             /* Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250) */
	nrf_delay_ms(40);    /* accumulate 40 samples in 80 milliseconds = 480 bytes */

	/* At end of sample accumulation, turn off FIFO sensor read */
	MPU9250_WriteByte(MPU9250_FIFO_EN, 0x00);             /* Disable gyro and accelerometer sensors for FIFO */
	MPU9250_ReadBytes(MPU9250_FIFO_COUNTH, 2, &data[0]);  /* read FIFO sample count */
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;         /* How many sets of full gyro and accelerometer data for averaging */

	for (ii = 0; ii < packet_count; ii++) 
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};

		MPU9250_ReadBytes(MPU9250_FIFO_R_W, 12, &data[0]);  /* read data for averaging */

		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;   /* Form signed 16-bit integer for each sample in FIFO */
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0];   /* Sum individual signed 16-bit biases to get accumulated signed 32-bit biases */
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	accel_bias[0] /= (int32_t) packet_count;      /* Normalize sums to get average count biases */
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L)        /* Remove gravity from the z-axis accelerometer bias calculation */
	{
		accel_bias[2] -= (int32_t) accelsensitivity;
	}  
	else 
	{
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	/* Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup */
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;     /* Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format */
	data[1] = (-gyro_bias[0]/4)       & 0xFF;     /* Biases are additive, so change sign on calculated average gyro biases */
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	/* Push gyro biases to hardware registers */
//	MPU9250_WriteByte(XG_OFFSET_H, data[0]);
//	MPU9250_WriteByte(XG_OFFSET_L, data[1]);
//	MPU9250_WriteByte(YG_OFFSET_H, data[2]);
//	MPU9250_WriteByte(YG_OFFSET_L, data[3]);
//	MPU9250_WriteByte(ZG_OFFSET_H, data[4]);
//	MPU9250_WriteByte(ZG_OFFSET_L, data[5]);

	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;      /* construct gyro bias in deg/s for later manual subtraction */
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	/* Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	the accelerometer biases calculated above must be divided by 8. */

	int32_t accel_bias_reg[3] = {0, 0, 0};        /* A place to hold the factory accelerometer trim biases */

	MPU9250_ReadBytes(MPU9250_XA_OFFSET_H, 2, &data[0]);  /* Read factory accelerometer trim values */
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	MPU9250_ReadBytes(MPU9250_YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	MPU9250_ReadBytes(MPU9250_ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	uint32_t mask = 1uL;          /* Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers */
	uint8_t mask_bit[3] = {0, 0, 0};      /* Define array to hold mask bit for each accelerometer bias axis */

	for(ii = 0; ii < 3; ii++) 
	{
		/* If temperature compensation bit is set, record that fact in mask_bit */
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01;  
	}

	/* Construct total accelerometer bias, including calculated average accelerometer bias from above */
	accel_bias_reg[0] -= (accel_bias[0]/8);       /* Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale) */
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0];      /* preserve temperature compensation bit when writing back to accelerometer bias registers */
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1];      /* preserve temperature compensation bit when writing back to accelerometer bias registers */
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2];      /* preserve temperature compensation bit when writing back to accelerometer bias registers */

	/* Apparently this is not working for the acceleration biases in the MPU-9250
	Are we handling the temperature correction bit properly?
	Push accelerometer biases to hardware registers */
//	MPU9250_WriteByte(XA_OFFSET_H, data[0]);
//	MPU9250_WriteByte(XA_OFFSET_L, data[1]);
//	MPU9250_WriteByte(YA_OFFSET_H, data[2]);
//	MPU9250_WriteByte(YA_OFFSET_L, data[3]);
//	MPU9250_WriteByte(ZA_OFFSET_H, data[4]);
//	MPU9250_WriteByte(ZA_OFFSET_L, data[5]);

	/* Output scaled accelerometer biases for manual subtraction in the main program */
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


/* Accelerometer and gyroscope self test; check calibration wrt factory settings */
void MPU9250_SelfTest(float * destination) /* Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass */
{
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;

	MPU9250_WriteByte(MPU9250_SMPLRT_DIV, 0x00);          /* Set gyro sample rate to 1 kHz */
	MPU9250_WriteByte(MPU9250_CONFIG, 0x02);              /* Set gyro sample rate to 1 kHz and DLPF to 92 Hz */
	MPU9250_WriteByte(MPU9250_GYRO_CONFIG, 1<<FS);        /* Set full scale range for the gyro to 250 dps */
	MPU9250_WriteByte(MPU9250_ACCEL_CONFIG2, 0x02);       /* Set accelerometer rate to 1 kHz and bandwidth to 92 Hz */
	MPU9250_WriteByte(MPU9250_ACCEL_CONFIG, 1<<FS);       /* Set full scale range for the accelerometer to 2 g */

	for( int ii = 0; ii < 200; ii++)      /*get average current values of gyro and acclerometer */
	{ 
		MPU9250_ReadBytes(MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);    /* Read the six raw data registers into data array */
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;     /* Turn the MSB and LSB into a signed 16-bit value */
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		MPU9250_ReadBytes(MPU9250_GYRO_XOUT_H, 6, &rawData[0]);     /* Read the six raw data registers sequentially into data array */
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;     /* Turn the MSB and LSB into a signed 16-bit value */
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	/* Get average of 200 values and store as average current readings */
	for (int ii =0; ii < 3; ii++) 
	{ 
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	/* Configure the accelerometer for self-test */
	MPU9250_WriteByte(MPU9250_ACCEL_CONFIG, 0xE0);        /* Enable self test on all three axes and set accelerometer range to +/- 2 g */
	MPU9250_WriteByte(MPU9250_GYRO_CONFIG, 0xE0);         /* Enable self test on all three axes and set gyro range to +/- 250 degrees/s */
	nrf_delay_ms(25);    /* Delay a while to let the device stabilize */

	for( int ii = 0; ii < 200; ii++)  /* get average self-test values of gyro and acclerometer */
	{
		MPU9250_ReadBytes(MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);    /* Read the six raw data registers into data array */
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;   /* Turn the MSB and LSB into a signed 16-bit value */
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		MPU9250_ReadBytes(MPU9250_GYRO_XOUT_H, 6, &rawData[0]);     /* Read the six raw data registers sequentially into data array */
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;   /* Turn the MSB and LSB into a signed 16-bit value */
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	/* Get average of 200 values and store as average self-test readings */
	for (int ii =0; ii < 3; ii++)  
	{
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	/* Configure the gyro and accelerometer for normal operation */
	MPU9250_WriteByte(MPU9250_ACCEL_CONFIG, 0x00);
	MPU9250_WriteByte(MPU9250_GYRO_CONFIG, 0x00);
	nrf_delay_ms(25);    /* Delay a while to let the device stabilize */

	/* Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg */
	selfTest[0] = MPU9250_ReadByte(MPU9250_SELF_TEST_X_ACCEL);    /* X-axis accel self-test results */
	selfTest[1] = MPU9250_ReadByte(MPU9250_SELF_TEST_Y_ACCEL);    /* Y-axis accel self-test results */
	selfTest[2] = MPU9250_ReadByte(MPU9250_SELF_TEST_Z_ACCEL);    /* Z-axis accel self-test results */
	selfTest[3] = MPU9250_ReadByte(MPU9250_SELF_TEST_X_GYRO);     /* X-axis gyro self-test results */
	selfTest[4] = MPU9250_ReadByte(MPU9250_SELF_TEST_Y_GYRO);     /* Y-axis gyro self-test results */
	selfTest[5] = MPU9250_ReadByte(MPU9250_SELF_TEST_Z_GYRO);     /* Z-axis gyro self-test results */

	/* Retrieve factory self-test value from self-test code reads */
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01f , ((float)selfTest[0] - 1.0f) ));      /* FT[Xa] factory trim calculation */
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01f , ((float)selfTest[1] - 1.0f) ));      /* FT[Ya] factory trim calculation */
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01f , ((float)selfTest[2] - 1.0f) ));      /* FT[Za] factory trim calculation */
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01f , ((float)selfTest[3] - 1.0f) ));      /* FT[Xg] factory trim calculation */
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01f , ((float)selfTest[4] - 1.0f) ));      /* FT[Yg] factory trim calculation */
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01f , ((float)selfTest[5] - 1.0f) ));      /* FT[Zg] factory trim calculation */

	/* Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	 To get percent, must multiply by 100 */
	for (int i = 0; i < 3; i++) 
	{
		destination[i] = 100.0f*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0f;        /* Report percent differences */
		destination[i+3] = 100.0f*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0f;    /* Report percent differences */
	}
}

void MPU9250_MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
//	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];     /* short name local variable for readability */
//	float norm = 0.0f;
//	float hx= 0.0f, hy= 0.0f, _2bx= 0.0f, _2bz= 0.0f;
//	float s1= 0.0f, s2= 0.0f, s3= 0.0f, s4= 0.0f;
//	float qDot1= 0.0f, qDot2= 0.0f, qDot3= 0.0f, qDot4= 0.0f;
	float norm = 0.0f;
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	
//	/* Auxiliary variables to avoid repeated arithmetic */
//	float _2q1mx;
//	float _2q1my;
//	float _2q1mz;
//	float _2q2mx;
//	float _4bx;
//	float _4bz;
//	float _2q1 = 2.0f * q1;
//	float _2q2 = 2.0f * q2;
//	float _2q3 = 2.0f * q3;
//	float _2q4 = 2.0f * q4;
//	float _2q1q3 = 2.0f * q1 * q3;
//	float _2q3q4 = 2.0f * q3 * q4;
//	float q1q1 = q1 * q1;
//	float q1q2 = q1 * q2;
//	float q1q3 = q1 * q3;
//	float q1q4 = q1 * q4;
//	float q2q2 = q2 * q2;
//	float q2q3 = q2 * q3;
//	float q2q4 = q2 * q4;
//	float q3q3 = q3 * q3;
//	float q3q4 = q3 * q4;
//	float q4q4 = q4 * q4;

	/* Normalise accelerometer measurement */
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return;     /* handle NaN */
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	/* Normalise magnetometer measurement */
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return;     /* handle NaN */
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;
	
	// Rate of change of quaternion from gyroscope
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Normalise accelerometer measurement
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	if (recipNorm == 0.0f) return;     /* handle NaN */
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	// Normalise magnetometer measurement
	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	if (recipNorm == 0.0f) return;     /* handle NaN */
	mx *= recipNorm;
	my *= recipNorm;
	mz *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
	_2q0mx = 2.0f * q0 * mx;
	_2q0my = 2.0f * q0 * my;
	_2q0mz = 2.0f * q0 * mz;
	_2q1mx = 2.0f * q1 * mx;
	_2q0 = 2.0f * q0;
	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
	_2q0q2 = 2.0f * q0 * q2;
	_2q2q3 = 2.0f * q2 * q3;
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Reference direction of Earth's magnetic field
	hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
	hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
	recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
	s0 *= recipNorm;
	s1 *= recipNorm;
	s2 *= recipNorm;
	s3 *= recipNorm;

	// Apply feedback step
	qDot1 -= beta * s0;
	qDot2 -= beta * s1;
	qDot3 -= beta * s2;
	qDot4 -= beta * s3;

	// Integrate rate of change of quaternion to yield quaternion
//	q0 += qDot1 * deltat;
//	q1 += qDot2 * deltat;
//	q2 += qDot3 * deltat;
//	q3 += qDot4 * deltat;	
	
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;	
	
	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/* Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
   measured ones. */
void MPU9250_MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];     /* short name local variable for readability */
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	/* Auxiliary variables to avoid repeated arithmetic */
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;   

	/* Normalise accelerometer measurement */
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return;     /* handle NaN */
	norm = 1.0f / norm;           /* use reciprocal for division */
	ax *= norm;
	ay *= norm;
	az *= norm;

	/* Normalise magnetometer measurement */
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return;     /* handle NaN */
	norm = 1.0f / norm;           /* use reciprocal for division */
	mx *= norm;
	my *= norm;
	mz *= norm;

	/* Reference direction of Earth's magnetic field */
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	/* Estimated direction of gravity and magnetic field */
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

	/* Error is cross product between estimated direction and measured direction of gravity */
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
		eInt[0] += ex;              /* accumulate integral error */
		eInt[1] += ey;
		eInt[2] += ez;
	}
	else
	{
		eInt[0] = 0.0f;             /* prevent integral wind up */
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	/* Apply feedback terms */
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	/* Integrate rate of change of quaternion */
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	/* Normalise quaternion */
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

void Init_MPU9250_DRDY_Interrupt(void)
{	
	ret_code_t	err_code;
			
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	in_config.pull = NRF_GPIO_PIN_PULLUP;

	err_code = nrf_drv_gpiote_in_init(MPU9250_INTO_PIN, &in_config, mpu9250_in_pin_handler);	

	APP_ERROR_CHECK(err_code);
}

void Enable_MPU9250_DRDY_Interrupt(void)
{
	nrf_drv_gpiote_in_event_enable(MPU9250_INTO_PIN, true);
}

void Disable_MPU9250_DRDY_Interrupt(void)
{
	nrf_drv_gpiote_in_event_disable(MPU9250_INTO_PIN);
}

ret_code_t MPU9250_ReadBytes(uint8_t Register, uint8_t Length, uint8_t *MPU9250_Rx_Buffer)
{   
	ret_code_t err_code;
	
	do
    {
       uint8_t addr8 = (uint8_t)Register;
       err_code = nrf_drv_twi_tx(&m_twi_mpu_9250, MPU9250_ADDR, &addr8, 1, true);
       if(NRF_SUCCESS != err_code)
       {
           break;
       }
       err_code = nrf_drv_twi_rx(&m_twi_mpu_9250, MPU9250_ADDR, MPU9250_Rx_Buffer, Length);
	   APP_ERROR_CHECK(err_code);
    }while(0);
	
	return err_code;
}

ret_code_t MPU9250_WriteBytes(uint8_t Register, uint8_t Length, uint8_t *MPU9250_Tx_Buffer)
{   
	ret_code_t err_code;
	
	do
    {
		uint8_t buffer[1 + Length]; /* Addr + data */
        buffer[0] = (uint8_t)Register;
        memcpy(buffer+1, MPU9250_Tx_Buffer, Length);
        err_code = nrf_drv_twi_tx(&m_twi_mpu_9250, MPU9250_ADDR, buffer, Length+1, false);
		APP_ERROR_CHECK(err_code);
    }while(0);

	return err_code;
}

uint8_t MPU9250_ReadByte(uint8_t Register)
{   
	ret_code_t err_code;
	uint8_t MPU9250_Rx_Buffer = 0; 
  
	do
    {
       uint8_t addr8 = (uint8_t)Register;
       err_code = nrf_drv_twi_tx(&m_twi_mpu_9250, MPU9250_ADDR, &addr8, 1, true);
       if(NRF_SUCCESS != err_code)
       {
           break;
       }
       err_code = nrf_drv_twi_rx(&m_twi_mpu_9250, MPU9250_ADDR, &MPU9250_Rx_Buffer, 1);
	   APP_ERROR_CHECK(err_code);
    }while(0);
  
	/*!< Return Register value */
	return (uint8_t)MPU9250_Rx_Buffer;
}

ret_code_t MPU9250_WriteByte(uint8_t Register, uint8_t RegValue)
{   
	ret_code_t err_code;
	uint8_t MPU9250_Tx_Buffer = RegValue;
  
	do
    {
		uint8_t buffer[2]; /* Addr + data */
        buffer[0] = (uint8_t)Register;
        memcpy(buffer+1, &MPU9250_Tx_Buffer, 1);
        err_code = nrf_drv_twi_tx(&m_twi_mpu_9250, MPU9250_ADDR, buffer, 2, false);
		APP_ERROR_CHECK(err_code);
    }while(0);
    
	return err_code;  
}

/**
  * @brief  Read the specified register from the AK8963.
  * @param  RegName: specifies the AK8963 register to be read.
  *          This parameter can be one of the following values:  
  *            @arg AK8963_REG_TEMP: temperature register
  *            @arg AK8963_REG_TOS: Over-limit temperature register
  *            @arg AK8963_REG_THYS: Hysteresis temperature register
  * @retval AK8963 register value.
  */
ret_code_t AK8963_ReadBytes(uint8_t Register, uint8_t Length, uint8_t *AK8963_Rx_Buffer)
{   
	ret_code_t err_code;
	uint8_t addr8 = (uint8_t)Register;
	
	do
	{
	   err_code = nrf_drv_twi_tx(&m_twi_ak_8963, AK8963_ADDR, &addr8, 1, true);	
	   if(NRF_SUCCESS != err_code)
	   {
		   break;
	   }
	   err_code = nrf_drv_twi_rx(&m_twi_ak_8963, AK8963_ADDR, AK8963_Rx_Buffer, Length);
//	   APP_ERROR_CHECK(err_code);
	}while(0);


	/* return a Reg value */
	return err_code;
}

uint8_t AK8963_ReadByte(uint8_t Register)
{   
	ret_code_t err_code;
	
	uint8_t AK8963_Rx_Buffer = 0; 
  
	do
    {
		uint8_t addr8 = (uint8_t)Register;
		err_code = nrf_drv_twi_tx(&m_twi_ak_8963, AK8963_ADDR, &addr8, 1, true);
		if(NRF_SUCCESS != err_code)
		{
		   break;
		}
		err_code = nrf_drv_twi_rx(&m_twi_ak_8963, AK8963_ADDR, &AK8963_Rx_Buffer, 1);
//	   APP_ERROR_CHECK(err_code);
    }while(0);
  
	/*!< Return Register value */
	return (uint8_t)AK8963_Rx_Buffer; 
}

ret_code_t AK8963_WriteByte(uint8_t Register, uint8_t RegValue)
{   
	ret_code_t err_code;
	uint8_t AK8963_Tx_Buffer = RegValue;
  
	do
    {
		uint8_t buffer[2]; /* Addr + data */
        buffer[0] = (uint8_t)Register;
        memcpy(buffer+1, &AK8963_Tx_Buffer, 1);
        err_code = nrf_drv_twi_tx(&m_twi_ak_8963, AK8963_ADDR, buffer, 2, false);
//		APP_ERROR_CHECK(err_code);	/*  실행시 오작동 */
    }while(0);
    
	return err_code;  
}

