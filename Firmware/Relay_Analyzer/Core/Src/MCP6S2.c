/*
 * MCP6S2.c
 *
 *  Created on: Jun 18, 2021
 *      Author: Kevin MacIntosh
 */
#include "MCP6S2.h"
#include <stdio.h>

SPI_HandleTypeDef hspi2;

const uint8_t NOP = 0b00000000;
const uint8_t GAIN_SELECT =  0b01000000;
const uint8_t GAIN_1 = 0b00000000;
const uint8_t GAIN_2 = 0b00000001;
const uint8_t GAIN_4 = 0b00000010;
const uint8_t GAIN_5 = 0b00000011;
const uint8_t GAIN_8 = 0b00000100;
const uint8_t GAIN_10 = 0b00000101;
const uint8_t GAIN_16 = 0b00000110;
const uint8_t GAIN_32 = 0b00000111;

uint16_t buf;

char * Error_output;

void setGain(uint8_t gain){
	//construct instruction for desired gain
	switch(gain){
	case 1:
		buf = GAIN_SELECT<<8 | GAIN_1;
		break;
	case 2:
		buf = GAIN_SELECT<<8 | GAIN_2;
		break;
	case 4:
		buf = GAIN_SELECT<<8 | GAIN_4;
		break;
	case 5:
		buf = GAIN_SELECT<<8 | GAIN_5;
		break;
	case 8:
		buf = GAIN_SELECT<<8 | GAIN_8;
		break;
	case 10:
		buf = GAIN_SELECT<<8 | GAIN_10;
		break;
	case 16:
		buf = GAIN_SELECT<<8 | GAIN_16;
		break;
	case 32:
		buf = GAIN_SELECT<<8 | GAIN_32;
		break;
	}

	//transmit the gain select command and desired gain code over SPI to MCP6S2
	if(HAL_SPI_Transmit(&hspi2, (uint8_t*)&buf, 1, 100) != HAL_OK)
			SPI_Error_Handler();
}

void SPI_Error_Handler(void)
{
   //display error message
   sprintf(Error_output, "SPI Error.");
}
