/*
 * stm32f411re_gpio_driver.c
 *
 *  Created on: May 30, 2024
 *      Author: Admin
 */


#include "stm32f411re_gpio_driver.h"
/*********************************************************************
 * @fn      		  - RCC_peripheral_clock_enable
 * @brief             -
 *
 * @param[in]         -	*pRCC points to the RCC peripheral base address form structure RCC
 * @param[in]         -	name_clock 
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Resolve all the TODOs

 *//////////////////////////////////////////////////////////////////////
void RCC_peripheral_clock_enable(RCC_RegDef_t* pRCC,uint8_t name_clock,uint8_t	peripheral){
	switch (peripheral){
		case 1:
			pRCC->AHB1ENR	|=  (1<<name_clock);
			break;
		case 2:
			pRCC->AHB2ENR	|= 	(1<<name_clock);
			break;
		case 3:
			pRCC->APB1ENR	|=  (1<<name_clock);
			break;
		case 4:
			pRCC->APB2ENR	|= 	(1<<name_clock);
			break;
	}
}


/*********************************************************************
 * @fn      		  - GPIO_pinMode
 * @brief             - Configures the mode, speed, pull-up/pull-down, and alternate function of a GPIO pin.
 *
 * @param[in]         -	*pGPIO points to the GPIO peripheral base address form structure GPIO
 * @param[in]         -	pin for GPIO 0-15
 * @param[in]         -	mode configure  INPUT,OUTPUT,ALTERNATE_FUNCTION,ANALOG
 * @param[in]         -	speed configure LOW_SPEED, MEDIUM_SPEED, FAST_SPEED, HIGH_SPEED
 * @param[in]         -	pupd (pullup_pulldown) configure NO_PUPD, PULL_UP, PULL_DOWN 
 * @param[in]         -	AFRLH (alternate function high and low) AF0-AF15
 *
 * @return            -	None
 *
 * @Note              -

 *//////////////////////////////////////////////////////////////////////
void GPIO_pinMode(GPIO_RegDef_t *pGPIO,uint8_t pin,uint8_t mode,uint8_t speed,uint8_t pupd,uint8_t AFRLH){
	pGPIO->MODER	&= 	~(0x3 << (pin*2));
	pGPIO->MODER 	|= 	(mode << (pin*2));

	pGPIO->OSPEEDR 	&= 	~(0x3 << (pin*2));
	pGPIO->OSPEEDR 	|= 	(speed << (pin*2));
	pGPIO->PUPDR 	&= 	~(0x3 << (pin*2));
	pGPIO->PUPDR 	|= 	(pupd << (pin*2));

	if(mode == 0x10){
		pGPIO->AFR[pin/8] &= ~(0xF << ((pin%8)*4));
		pGPIO->AFR[pin/8] |= (AFRLH << ((pin%8)*4));
	}
}


/*********************************************************************
 * @fn      		  - GPIO_digitalWrite
 * @brief             - send 1 or 0 for GPIO output
 *
 * @param[in]         -	*pGPIO points to the GPIO peripheral base address form structure GPIO 
 * @param[in]         -	pin for GPIO pin 0-15
 * @param[in]         - value for HIGH/LOW
 *
 * @return            - None
 *
 * @Note              -  

 *//////////////////////////////////////////////////////////////////////
void GPIO_digitalWrite(GPIO_RegDef_t *pGPIO,uint8_t pin,uint8_t value){
	if(value == 1){
		pGPIO->ODR |= (1<<pin);
	}else{
		pGPIO->ODR &= ~(1<<pin);
	}
}


/*********************************************************************
 * @fn      		  - GPIO_digitalRead
 * @brief             - Read value for GPIO pin 0 or 1
 *
 * @param[in]         -	*pGPIO points to the GPIO peripheral base address form structure GPIO 
 * @param[in]         - pin for GPIO pin 0-15
 *
 * @return            - None
 *
 * @Note              - 

 *//////////////////////////////////////////////////////////////////////
uint8_t GPIO_digitalRead(GPIO_RegDef_t *pGPIO,uint8_t pin){
	return (uint8_t)(pGPIO->IDR >> pin & 0x01);
}


/*********************************************************************
 * @fn      		  - GPIO_digitalToggleWrite
 * @brief             - toggle 1 or 0 for GPIO output
 *
 * @param[in]         -	*pGPIO points to the GPIO peripheral base address form structure GPIO 
 * @param[in]         -	pin for GPIO pin 0-15
 *
 * @return            -	 None
 *
 * @Note              -  

 *//////////////////////////////////////////////////////////////////////
void GPIO_digitalToggleWrite(GPIO_RegDef_t *pGPIO,uint8_t pin){
	pGPIO->ODR ^= (1<<pin);
}


/*********************************************************************
 * @fn      		  - Exti_init
 * @brief             - Configures the external interrupt for a specific GPIO pin by setting up the 
 *                      System Configuration external interrupt and EXTI lines.
 *
 * @param[in]         -	port PA,PB,PC,PD,PE,PH
 * @param[in]         -	pin for GPIO pin 0-15
 * @param[in]         -	mode trigger mode for EXTI RISING_TRIGGER or FALLING_TRIGGER
 *
 * @return            - None
 *
 * @Note              -  

 *//////////////////////////////////////////////////////////////////////
void Exti_init(uint8_t port,uint8_t pin,uint8_t mode){

	/* set System Configuration external interrupt */
	SYSCFG->EXTICR[pin/4] &= ~(0xF << (pin%4)*4);
	SYSCFG->EXTICR[pin/4] |= (port << (pin%4)*4);

	/* set interrupt mask register mean set enable Exti line for detect event rising or falling */
	EXTI->IMR |= (1<<pin); 

	if(mode == FALLING_TRIGGER){
		EXTI->FTSR |= (1<<pin);
	}else{
			EXTI->RTSR |= (1<<pin);
		}
}


/*********************************************************************
 * @fn      		  - Nvic_EnableIRQ
 * @brief             -	Enables or disables a specific interrupt in the Nested Vectored Interrupt Controller (NVIC).
 *
 * @param[in]         -	position The interrupt number (IRQn) 0-85 to be enabled or disabled. 
 *                               Ex. IRQ_EXTI0, IRQ_USART2,...
 * @param[in]         -	ENorDISN Flag to enable or disable the interrupt. 
 *                                Use 1 to enable and 0 to disable.
 *
 * @return            - None
 *
 * @Note              - This function directly manipulates the NVIC registers to enable or disable 
 *                      interrupts based on their position.
 *                    - NVIC_ISE0, NVIC_ISE1, and NVIC_ISE2 correspond to different interrupt set-enable registers,
 *                      handling interrupts in blocks of 32.
 *                    - NVIC_ICE0, NVIC_ICE1, and NVIC_ICE2 correspond to interrupt clear-enable registers, 
 *                      used to disable interrupts.

 *//////////////////////////////////////////////////////////////////////
void Nvic_EnableIRQ(uint8_t position,uint8_t ENorDISN){
	if(ENorDISN == ENABLE){
		/*check number position IRQ (0-85) for choose ISE0,1,2*/
		if(position <32){
			*NVIC_ISE0 |= (1<< position);
		}else if (position<64){
			*NVIC_ISE1 |= (1<< position%32);
		}else if(position<86){
			*NVIC_ISE2 |= (1<< position%64);
		}
	}else{
		if(position <32){
			*NVIC_ICE0 |= (1<< position);
		}else if (position<64){
			*NVIC_ICE1 |= (1<< position%32);
		}else if(position<86){
			*NVIC_ICE2 |= (1<< position%64);
		}
	}
}


/*********************************************************************
 * @fn      		  - Nvic_SetProrityIRQ
 * @brief             -	Sets the priority of a specific interrupt in the Nested Vectored Interrupt Controller (NVIC).
 *
 * @param[in]         -position The interrupt number (IRQn) 0-85 to be enabled or disabled. 
 *                              Ex. IRQ_EXTI0, IRQ_USART2,...
 * @param[in]         -	priority The priority level to set for the interrupt (0-15, lower value means higher priority).
 *
 *
 * @return            - None
 *
 * @Note              -	This function directly manipulates the NVIC Interrupt Priority Registers (IPR).
 *					  -	

 *//////////////////////////////////////////////////////////////////////
void Nvic_SetProrityIRQ(uint8_t position,uint32_t priority){
	/*  
		*NVIC_IPR(position/4) NVIC Interrupt Priority Register (IPR)  see in datasheet pm0214 or DUI0553
		*why is position/4 because each IPR register manages the priority for four interrupts, each with its own 8-bit field.
		*and address offset NVIC_IRQ is 0x400 + (x*0x04) -> x = 0 to 59 
		*For instance, if IRQ_USART2 is interrupt number 38, then 38/4 gives 9, mean IRQ_USART2 range is in IPR9 
		*
		*Each IPR is  32 bit wide have 4 field (4 offset) each field is 8 bit IP[N]-> N is (4 * x +byte offset )
		*in IPR9, bits 0-7 for IRQ 36, bits 8-15 for IRQ 37, bits 16-23 for IRQ 38, and bits 24-31 for IRQ 39.
		*
		* Only bits [7:4] of each field are used to set the priority, while bits [3:0] are ignored and read as zero.
		*
		* The expression (position % 4) identifies the specific field within the 32-bit IPR register (4 offset)
	 	* for the given interrupt number. The shift (8 * (position % 4)) moves to the correct field.
	 	* Finally, an additional shift by 4 aligns to bits [7:4] for the actual priority setting.
	*/

	*NVIC_IPR(position/4) &= ~(0xF  << ((8*(position %4))+4)); /*reset priority*/
	*NVIC_IPR(position/4) |= priority << ((8*(position %4))+4);/*set priority*/
}



	
