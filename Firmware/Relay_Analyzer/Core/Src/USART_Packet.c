/**
  ******************************************************************************
  * @file           : USART_Packet.c
  * @brief          : USART_Packet program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "USART_Packet.h"

 //function for data packets for USART
//packets will have a 2 bit start/end opcode, 2 bit opcode for what type of data (bounce info, coil resistance, etc) and the data
uint8_t USART_tx_pkt(uint8_t dType, uint32_t msg){
	uint8_t pkt = 0b00000000;
	int msgSize = 32;
	uint32_t mask;
	uint8_t loc = 0;


	pkt = (0b1 << 7) | (dType << 5) | (msg >> (msgSize - 5)); //make packet with 1 at start (use bit mask here instead?)

	//if(HAL_UART_Transmit(&huart3, pkt, ADC_BUF_LEN, 100) != HAL_OK)
	//		  Error_Handler();

	//packets with 0 at MSB
	//mask 7 bits at desired location (must move location each packet
	pkt = (0b0 << 7) | (((1 << 7) - 1) & (msg >> (loc - 1)));
}
