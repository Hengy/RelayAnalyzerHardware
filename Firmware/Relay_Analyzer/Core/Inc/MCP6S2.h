/*
 * MCP6S2.h
 *
 *  Created on: Jun 18, 2021
 *      Author: Kevin MacIntosh
 */

#include<stdio.h>
#include "stm32f7xx_hal.h"

#ifndef INC_MCP6S2_H_
#define INC_MCP6S2_H_

void setGain(uint8_t gain);

void SPI_Error_Handler(void);

#endif /* INC_MCP6S2_H_ */
