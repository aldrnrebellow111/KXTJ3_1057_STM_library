/*
 * KXTJ3-1057-accl.h
 *
 *  Created on: Apr 7, 2023
 *      Author: Alvin
 */

#ifndef INC_KXTJ3_1057_ACCL_H_
#define INC_KXTJ3_1057_ACCL_H_
#include "main.h"

// Accelerometer provides different Power modes by changing output bit resolution
//#define LOW_POWER
#define HIGH_RESOLUTION

#define KXTJ3_1057_ADDR (0x0E<<1)

#define SAMPLE_RATE 400 // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
#define ACCEl_RANGE 8 // Accelerometer range = 2, 4, 8, 16g

#define MIN_DELTA_G 3.0 // for calculating minimum change in acceleration
#define INT_MOVE_DURATION 10 // WAKEUP_COUNTER (counts) = Desired Delay Time (sec) x OWUF (Hz), ie 1/100hz
#define INT_NO_MOVE_DURATION 100 // NA_COUNTER (counts) = Desired Delay Time (sec) x OWUF (Hz)
#define INT_PIN_POLARITY 0 // 1 - ACTIVE_HIGH ,0 - ACTIVE_LOW

#define INT_WAKEUP_THRESHOLD  ((float)(MIN_DELTA_G * 256.0)-1.0) // WAKEUP_THRESHOLD (counts) = Desired Threshold (g) x 256 (counts/g)

// Return values
typedef enum
{
	ACCL_SUCCESS,
	ACCL_HW_ERROR,
	ACCL_NOT_SUPPORTED,
	ACCL_GENERIC_ERROR,
	ACCL_OUT_OF_BOUNDS,
	ACCL_ALL_ONES_WARNING,
	//...
} accl_status_t;

typedef enum
{
	X = 0,
	Y,
	Z,
	NAN= 0xff,
} axis_t;

typedef struct Accel_data
{
	float accel_xg;
	float accel_yg;
	float accel_zg;
} s_accel_data;

//Device Registers
#define KXTJ3_WHO_AM_I               0x0F
#define KXTJ3_DCST_RESP							 0x0C	// used to verify proper integrated circuit functionality.It always has a byte value of 0x55

#define KXTJ3_OUT_X_L                0x06
#define KXTJ3_OUT_X_H                0x07
#define KXTJ3_OUT_Y_L                0x08
#define KXTJ3_OUT_Y_H                0x09
#define KXTJ3_OUT_Z_L                0x0A
#define KXTJ3_OUT_Z_H                0x0B

#define KXTJ3_STATUS				         0x18
#define KXTJ3_INT_SOURCE1            0x16
#define KXTJ3_INT_SOURCE2            0x17

#define KXTJ3_CTRL_REG1              0x1B // *
#define KXTJ3_CTRL_REG2              0x1D // *

#define KXTJ3_INT_CTRL_REG1          0x1E	// *
#define KXTJ3_INT_CTRL_REG2          0x1F	// *

#define KXTJ3_DATA_CTRL_REG					 0x21	// *
#define KXTJ3_WAKEUP_COUNTER				 0x29	// *
#define KXTJ3_NA_COUNTER						 0x2A	// *

#define KXTJ3_WAKEUP_THRD_H				 	 0x6A	// *
#define KXTJ3_WAKEUP_THRD_L					 0x6B	// *

// * Note that to properly change the value of this register, the PC1 bit in CTRL_REG1 must first be set to “0”.


accl_status_t KXTJ3_Init();
accl_status_t readRegister(uint8_t* outputPointer, uint8_t offset);
accl_status_t readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length);
accl_status_t writeRegister(uint8_t offset, uint8_t dataToWrite);
void applySettings( void );
accl_status_t standby( uint8_t _en );
accl_status_t KXTJ3_intConf(uint16_t threshold, uint8_t moveDur, uint8_t naDur, uint8_t polarity );
void KXTJ3_get_data(void);
accl_status_t readRegisterInt16( int16_t* outputPointer, uint8_t offset );
float axisAccel( axis_t _axis);
void KXTJ3_calibrate(void);
void KXTJ3_processGpio_INT( void );


#endif /* INC_KXTJ3_1057_ACCL_H_ */
