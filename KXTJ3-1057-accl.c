/*
 * KXTJ3-1057-accl.c
 *
 *  Created on: Apr 7, 2023
 *      Author: Alvin
 */
#include "KXTJ3-1057-accl.h"
#include "main.h"
#include "stdlib.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1; // Use the required I2C handle

static float 	accelSampleRate; 	// Sample Rate - 0.781, 1.563, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
static uint8_t accelRange; 			// Accelerometer range = 2, 4, 8, 16g

s_accel_data g_acceldata = {0.0f, 0.0f, 0.0f};


static float prev_acclX_g = 0.0;
static float prev_acclY_g = 0.0;
static float prev_acclZ_g = 0.0;

int16_t dataHighresX_offset = 0;
int16_t dataHighresY_offset = 0;
int16_t dataHighresZ_offset = 0;

accl_status_t KXTJ3_Init()
{

	accl_status_t returnError = ACCL_SUCCESS;

	// Start-up time, Figure 1: Typical StartUp Time - DataSheet
#ifdef HIGH_RESOLUTION
	if			( SAMPLE_RATE < 1)		HAL_Delay(1300);
	else if ( SAMPLE_RATE < 3)		HAL_Delay(650);
	else if ( SAMPLE_RATE < 6)		HAL_Delay(350);
	else if ( SAMPLE_RATE < 25)	HAL_Delay(180);
	else												HAL_Delay(45);
#else
	HAL_Delay(2);
#endif

	//Check the ID register to determine if the operation was a success.
	uint8_t _whoAmI;

	readRegister(&_whoAmI, KXTJ3_WHO_AM_I);

	if( _whoAmI != 0x35 )
	{
		returnError = ACCL_HW_ERROR;
	}

	accelSampleRate = SAMPLE_RATE;
	accelRange 			= ACCEl_RANGE;

	applySettings();
	//KXTJ3_calibrate();

	return returnError;
}

accl_status_t readRegister(uint8_t* outputPointer, uint8_t offset) {
	//Return value
	uint8_t result = 0;
	uint8_t numBytes = 1;
	accl_status_t returnError = ACCL_SUCCESS;
	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_I2C_Mem_Read(&hi2c1, KXTJ3_1057_ADDR, offset, numBytes, &result, numBytes, 1000);

	if( ret != HAL_OK )
	{
		returnError = ACCL_HW_ERROR;
	}

	*outputPointer = result;
	return returnError;
}

//****************************************************************************//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//****************************************************************************//
accl_status_t readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{

	//Return value
	uint8_t numBytes = 1;
	accl_status_t returnError = ACCL_SUCCESS;
	HAL_StatusTypeDef ret = HAL_OK;

	ret = HAL_I2C_Mem_Read(&hi2c1, KXTJ3_1057_ADDR, offset, numBytes, outputPointer, length, 1000);

	if( ret != HAL_OK )
	{
		returnError = ACCL_HW_ERROR;
	}

	return returnError;
}

//****************************************************************************//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//****************************************************************************//
accl_status_t writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	accl_status_t returnError = ACCL_SUCCESS;
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t numBytes = 1;

	ret = HAL_I2C_Mem_Write(&hi2c1, KXTJ3_1057_ADDR, offset, numBytes, &dataToWrite, numBytes, 1000);
	if( ret != HAL_OK )
	{
		returnError = ACCL_HW_ERROR;
	}

	return returnError;
}


void applySettings( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable

	standby( 1 );

	//Build DATA_CTRL_REG

	//  Convert ODR
	if(accelSampleRate < 1)					dataToWrite |= 0x08;	// 0.781Hz
	else if(accelSampleRate < 2)		dataToWrite |= 0x09;	// 1.563Hz
	else if(accelSampleRate < 4)		dataToWrite |= 0x0A;	// 3.125Hz
	else if(accelSampleRate < 8)		dataToWrite |= 0x0B;	// 6.25Hz
	else if(accelSampleRate < 16)		dataToWrite |= 0x00;	// 12.5Hz
	else if(accelSampleRate < 30)		dataToWrite |= 0x01;	// 25Hz
	else if(accelSampleRate < 60)		dataToWrite |= 0x02;	// 50Hz
	else if(accelSampleRate < 150)	dataToWrite |= 0x03;	// 100Hz
	else if(accelSampleRate < 250)	dataToWrite |= 0x04;	// 200Hz
	else if(accelSampleRate < 450)	dataToWrite |= 0x05;	// 400Hz
	else if(accelSampleRate < 850)	dataToWrite |= 0x06;	// 800Hz
	else														dataToWrite	|= 0x07;	// 1600Hz

	//Now, write the patched together data
	writeRegister(KXTJ3_DATA_CTRL_REG, dataToWrite);

	//Build CTRL_REG1

	// LOW power, 8-bit mode
	dataToWrite = 0x80;

#ifdef HIGH_RESOLUTION
	dataToWrite = 0xC0;
#endif

	//  Convert scaling
	switch(accelRange)
	{
	default:
	case 2:
		dataToWrite |= (0x00 << 2);
		break;
	case 4:
		dataToWrite |= (0x02 << 2);
		break;
	case 8:
		dataToWrite |= (0x04 << 2);
		break;
	case 16:
		dataToWrite |= (0x01 << 2);
		break;
	}

	//Now, write the patched together data
	writeRegister(KXTJ3_CTRL_REG1, dataToWrite);
	standby( 0 );
}

accl_status_t standby( uint8_t _en )
{
	uint8_t _ctrl;

	// "Backup" KXTJ3_CTRL_REG1
	readRegister(&_ctrl, KXTJ3_CTRL_REG1);

	if( _en == 1)
		_ctrl &= 0x7E;
	else
		_ctrl |= (0x01 << 7);	// disable standby-mode -> Bit7 = 1 = operating mode

	return writeRegister(KXTJ3_CTRL_REG1, _ctrl);
}


//****************************************************************************//
//  Configure interrupt, stop or move, threshold and duration
//	Durationsteps and maximum values depend on the ODR chosen.
//  WAKEUP_THRESHOLD (counts) = Desired Threshold (g) x 256 (counts/g)
//  WAKEUP_COUNTER (counts) = Desired Delay Time (sec) x OWUF (Hz)
//  OWUF is set as 100 hz in below function
//****************************************************************************//
accl_status_t KXTJ3_intConf(uint16_t threshold, uint8_t moveDur, uint8_t naDur, uint8_t polarity )
{
	// Note that to properly change the value of this register, the PC1 bit in CTRL_REG1must first be set to “0”.
	standby( 1 );

	accl_status_t returnError = ACCL_SUCCESS;

	// Build INT_CTRL_REG1

	uint8_t dataToWrite = 0x22;  		// Interrupt enabled, active LOW, non-latched

	if( polarity == 1 )
		dataToWrite |= (0x01 << 4);		// Active HIGH

	returnError = writeRegister(KXTJ3_INT_CTRL_REG1, dataToWrite);

	// WUFE – enables the Wake-Up (motion detect) function.

	uint8_t _reg1;

	returnError = readRegister(&_reg1, KXTJ3_CTRL_REG1);

	_reg1 |= (0x01 << 1);

	returnError = writeRegister(KXTJ3_CTRL_REG1, _reg1);


	// WUFE – enables the Wake-Up (motion detect) function.

	uint8_t _reg2;

	_reg2 = 0x07;

	returnError = writeRegister(KXTJ3_CTRL_REG2, _reg2);

	// Build INT_CTRL_REG2

	dataToWrite = 0xFF;  // enable interrupt on all axis any direction - Unlatched

	returnError = writeRegister(KXTJ3_INT_CTRL_REG2, dataToWrite);

	// Set WAKE-UP (motion detect) Threshold

	dataToWrite = (uint8_t)(threshold >> 4);

	returnError = writeRegister(KXTJ3_WAKEUP_THRD_H, dataToWrite);

	dataToWrite = (uint8_t)(threshold << 4);

	returnError = writeRegister(KXTJ3_WAKEUP_THRD_L, dataToWrite);

	// WAKEUP_COUNTER -> Sets the time motion must be present before a wake-up interrupt is set
	// WAKEUP_COUNTER (counts) = Wake-Up Delay Time (sec) x Wake-Up Function ODR(Hz)

	dataToWrite = moveDur;

	returnError = writeRegister(KXTJ3_WAKEUP_COUNTER, dataToWrite);

	// Non-Activity register sets the non-activity time required before another wake-up interrupt will be reported.
	// NA_COUNTER (counts) = Non-ActivityTime (sec) x Wake-Up Function ODR(Hz)

	dataToWrite = naDur;

	returnError = writeRegister(KXTJ3_NA_COUNTER, dataToWrite);

	// Set IMU to Operational mode
	returnError = standby( 0 );

	return returnError;
}


//****************************************************************************//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//****************************************************************************//
accl_status_t readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
	//offset |= 0x80; //turn auto-increment bit on
	uint8_t myBuffer[2];
	accl_status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = ((int16_t)myBuffer[0]) | (int16_t)(myBuffer[1] << 8);

	*outputPointer = output;
	return returnError;
}

// Read axis acceleration as Float
float axisAccel( axis_t _axis)
{
	int16_t outRAW;
	uint8_t regToRead = 0;
	switch (_axis)
	{
	case 0:
		// X axis
		regToRead = KXTJ3_OUT_X_L;
		break;
	case 1:
		// Y axis
		regToRead = KXTJ3_OUT_Y_L;
		break;
	case 2:
		// Z axis
		regToRead = KXTJ3_OUT_Z_L;
		break;

	default:
		// Not valid axis return NAN
		return NAN;
		break;
	}

	readRegisterInt16( &outRAW, regToRead );

	float outFloat;

	switch( accelRange )
	{
	case 2:
		outFloat = (float)outRAW / 15987;
		break;
	case 4:
		outFloat = (float)outRAW / 7840;
		break;
	case 8:
		outFloat = (float)outRAW / 3883;
		break;
	case 16:
		outFloat = (float)outRAW / 1280;
		break;
	default:
		outFloat = 0;
		break;
	}

	return outFloat;

}


void KXTJ3_get_data(void)
{

	g_acceldata.accel_xg = axisAccel( X ) ;

	g_acceldata.accel_yg =  axisAccel( Y ) ;

	g_acceldata.accel_zg = axisAccel( Z ) ;


	if ((fabs(g_acceldata.accel_xg - prev_acclX_g) >= MIN_DELTA_G) || (fabs(g_acceldata.accel_yg - prev_acclY_g) >= MIN_DELTA_G) || (fabs(g_acceldata.accel_zg - prev_acclZ_g) >= MIN_DELTA_G))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	}

	prev_acclX_g = g_acceldata.accel_xg ;
	prev_acclY_g = g_acceldata.accel_yg ;
	prev_acclZ_g = g_acceldata.accel_zg ;
}



void KXTJ3_calibrate(void)
{

	prev_acclX_g = axisAccel( X ) ;
	prev_acclY_g = axisAccel( Y ) ;
	prev_acclZ_g = axisAccel( Z ) ;

}

void KXTJ3_processGpio_INT( void )
{
	  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	  }
}
