/**
  ******************************************************************************
  * @file           : USART_Packet.h
  * @brief          : Header for USART_Packet.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include <stdbool.h>
  
/* Private defines -----------------------------------------------------------*/
//#define PACKET_START	0b1;
#define BOUNCE			0b00;
#define COIL			0b01;
#define TRIGGER			0b10;
#define COMMAND			0b11;

 //function for data packets for USART
//packets will have a 2 bit start/end opcode, 2 bit opcode for what type of data (bounce info, coil resistance, etc) and the data
//need defines for data types
uint8_t USART_tx_pkt(uint8_t dType, uint32_t msg);

//sends packet over usart connection
//void USART_pkt_tx(uint8_t pkt);
