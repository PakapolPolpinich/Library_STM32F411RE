/*
 * stm32f411re_usart_driver.c
 *
 *  Created on: May 31, 2024
 *      Author: Admin
 */

#include "stm32f411re_usart_driver.h"

void USART_init(USART_RegDef_t* pUSART,uint8_t mode,uint8_t length,uint8_t stopbit){
    if(mode == NORMAL_MODE){
        pUSART->CR[0] |= (1<<2)|(1<<3)|(1<<13);
    }else if (mode == RX_INTERRUPT_MODE){
        pUSART->CR[0] |= (1<<2)|(1<<3)|(1<<5)|(1<<13);
    }

    if(length == WORD_8_bit){
        pUSART->CR[0]   &= ~(1<<12);
    }else {
        pUSART->CR[0]   |= (1<<12);
    }

    if(stopbit == STOP_1_BIT){
        pUSART->CR[1]   &= ~(0x3<<12);
    }else if(stopbit == STOP_Half_BIT){
        pUSART->CR[1]   &= ~(0x3<<12);
        pUSART->CR[1]   |= (1<<12);
    }else if (stopbit == STOP_2_BIT){
        pUSART->CR[1]   |= (0x3 <<12);
    }
}


