
/**
  ******************************************************************************
  * @file           : INA226.h
  ******************************************************************************
  * Functions for the INA226 current monitor over I2C
  *
  ******************************************************************************
  */

#ifndef __INA226_H
#define __INA226_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

#define INA226_ADDRESS_WRITE		0x90 // INA226 I2C Write Address, shifted once left
#define INA226_ADDRESS_READ			0x91 // INA226 I2C Read Address, shifted once left

#define INA226_CONFIG		0x00 // Configuration Register (R/W)
#define INA226_SHUNTV		0x01 // Shunt Voltage (R)
#define INA226_BUSV			0x02 // Bus Voltage (R)
#define INA226_POWER		0x03 // Power (R)
#define INA226_CURRENT		0x04 // Current (R)
#define INA226_CALIB		0x05 // Calibration (R/W)

#define INA226_MODE_CONT	0x07 // Continuous Shunt and Bus Voltage

// Shunt Voltage Conversion Time
#define INA226_VSH_140uS			0x00<<3
#define INA226_VSH_204uS			0x01<<3
#define INA226_VSH_332uS			0x02<<3
#define INA226_VSH_588uS			0x03<<3
#define INA226_VSH_1100uS			0x04<<3
#define INA226_VSH_2116uS			0x05<<3
#define INA226_VSH_4156uS			0x06<<3
#define INA226_VSH_8244uS			0x07<<3

// Bus Voltage Conversion Time
#define INA226_VBUS_140uS			0x00<<6
#define INA226_VBUS_204uS			0x01<<6
#define INA226_VBUS_332uS			0x02<<6
#define INA226_VBUS_588uS			0x03<<6
#define INA226_VBUS_1100uS			0x04<<6
#define INA226_VBUS_2116uS			0x05<<6
#define INA226_VBUS_4156uS			0x06<<6
#define INA226_VBUS_8244uS			0x07<<6

// Averaging Mode
#define INA226_AVG_1				0x00<<9
#define INA226_AVG_4				0x01<<9
#define INA226_AVG_16				0x02<<9
#define INA226_AVG_64				0x03<<9
#define INA226_AVG_128				0x04<<9
#define INA226_AVG_256				0x05<<9
#define INA226_AVG_512				0x06<<9
#define INA226_AVG_1024				0x07<<9

#define INA226_I2C_TO 				15

void INA226_setConfig(I2C_HandleTypeDef *, uint16_t addr, uint16_t avg, uint16_t busconv, uint16_t shuntconv, uint16_t mode);
void INA226_setCal(I2C_HandleTypeDef *, uint16_t addr, uint16_t calVal);

uint16_t INA226_getConfig(I2C_HandleTypeDef *, uint16_t addr);
uint32_t INA226_getBusVoltage(I2C_HandleTypeDef *, uint16_t addr);
uint32_t INA226_getShuntVoltage(I2C_HandleTypeDef *, uint16_t addr);
uint32_t INA226_getCurrentVoltage(I2C_HandleTypeDef *, uint16_t addr);
uint32_t INA226_getPowerVoltage(I2C_HandleTypeDef *, uint16_t addr);

#endif /* INC_INA226_H_ */
