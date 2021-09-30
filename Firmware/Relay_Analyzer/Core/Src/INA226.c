#include "INA226.h"

void INA226_setConfig(I2C_HandleTypeDef *hi2c1, uint16_t addr, uint16_t avg, uint16_t busconv, uint16_t shuntconv, uint16_t mode) {
	uint16_t config = (0x4000 | avg | busconv | shuntconv | mode);
//	uint8_t sendData[3] = {0x00, (uint8_t)(config>>8), (uint8_t)config};
//	HAL_I2C_Master_Transmit(&hi2c1, addr, sendData, 3, INA226_I2C_TO);		// write to config reg

	uint8_t sendData[2] = {0b01000111, 0b00100111};
	HAL_I2C_Mem_Write(&hi2c1, addr, 0x00, 2, sendData, 2, INA226_I2C_TO);		// write to config reg
}

void INA226_setCal(I2C_HandleTypeDef *hi2c1, uint16_t addr, uint16_t calVal) {
	uint8_t sendData[3] = {0x05, (uint8_t)(calVal>>8), (uint8_t)(calVal)};
	HAL_I2C_Master_Transmit(&hi2c1, addr, sendData, 3, INA226_I2C_TO);		// write to calibration reg
}

uint16_t INA226_getConfig(I2C_HandleTypeDef *hi2c1, uint16_t addr){

}

uint32_t INA226_getBusVoltage(I2C_HandleTypeDef *hi2c1, uint16_t addr){

}

uint32_t INA226_getShuntVoltage(I2C_HandleTypeDef *hi2c1, uint16_t addr){

}

uint32_t INA226_getCurrentVoltage(I2C_HandleTypeDef *hi2c1, uint16_t addr){

}

uint32_t INA226_getPowerVoltage(I2C_HandleTypeDef *hi2c1, uint16_t addr){

}
