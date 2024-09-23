/*
 * stm32f411re_usart_driver.h
 *
 *  Created on: May 31, 2024
 *      Author: Admin
 */

#ifndef INC_STM32F411RE_USART_DRIVER_H_
#define INC_STM32F411RE_USART_DRIVER_H_

#include "stm32f411re_by_belt.h"

typedef struct{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
 	uint32_t CR[3];
 	volatile uint32_t GTPR;
}USART_RegDef_t;

#define USART1			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2			((USART_RegDef_t*)USART2_BASEADDR)
#define USART6			((USART_RegDef_t*)USART6_BASEADDR)

void USART_peripheral_clock_enable(RCC_RegDef_t* pRCC,uint8_t name_clock,uint8_t	peripheral);
void USART_init(USART_RegDef_t* pUSART,uint8_t mode,uint8_t length,uint8_t stopbit);
/*function parity*/
void USART_baudrate();
void USART_Senddata();
void USART_Receivedata();
void USART_Ringbuffer();

#define NORMAL_MODE  		0
#define RX_INTERRUPT_MODE	1
#define WORD_8_bit 			0
#define STOP_1_BIT			0
#define STOP_Half_BIT		1
#define STOP_2_BIT			2

#endif /* INC_STM32F411RE_USART_DRIVER_H_ */
