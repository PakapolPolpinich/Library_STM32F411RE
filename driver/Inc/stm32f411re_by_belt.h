/*
 * stm32f411re_by_belt.h
 *
 *  Created on: May 30, 2024
 *      Author: Admin
 */

#ifndef INC_STM32F411RE_BY_BELT_H_
#define INC_STM32F411RE_BY_BELT_H_

#include<stdint.h>

#define FLASH_BASEADDR	0x08000000U /* default address when executing  in stm32*/
#define SRAM1_BASEADDR	0x20000000U
#define ROM_BASEADDR	0X1FFF0000U /*system memory*/
#define SRAM1			SRAM1_BASEADDR


#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR+0x1000)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR+0x1C00)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR+0x3800)

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR+0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR+0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR+0x5C00)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR+0x4400)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR+0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR+0x3C00)
#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR+0x0000)
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR+0x0400)
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR+0x0800)
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR+0x0C00)


#define TIM1_BASEADDR			(APB2PERIPH_BASEADDR+0x0000)
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR+0x4000)
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR+0x4400)
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR+0x4800)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR+0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR+0x3800)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR+0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR+0x1400)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR+0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR+0x3400)
#define SPI5_BASEADDR			(APB2PERIPH_BASEADDR+0x5000)

typedef struct
{
	volatile uint32_t CR; 	/*GPIO port mode register */
	volatile uint32_t PLLCFGR;	/*GPIO port output type register */
	volatile uint32_t CFGR;	/*GPIO port output speed register */
	volatile uint32_t CIR;		/*GPIO port pull-up/pull-down register */
	volatile uint32_t AHB1RSTR;		/*GPIO port input data register */
	volatile uint32_t AHB2RSTR;
	uint32_t RESERVED0[2];
	volatile uint32_t APB1RSTR;		/*GPIO port bit set/reset register */
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;		/*GPIO port bit set/reset register */
	volatile uint32_t AHB2ENR;
	uint32_t RESERVED2[2];/*GPIO port configuration lock register*/
	volatile uint32_t APB1ENR;		/*GPIO port input data register */
	volatile uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;		/*GPIO port bit set/reset register */
	volatile uint32_t AHB2LPENR;
	uint32_t RESERVED4[2];
	volatile uint32_t APB1LPENR;		/*GPIO port bit set/reset register */
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;		/*GPIO port input data register */
	volatile uint32_t CSR;
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;		/*GPIO port bit set/reset register */
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t DCKCFGR;
}RCC_RegDef_t;

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

typedef struct
{
	volatile uint32_t MODER; 	/*GPIO port mode register */
	volatile uint32_t OTYPER;	/*GPIO port output type register */
	volatile uint32_t OSPEEDR;	/*GPIO port output speed register */
	volatile uint32_t PUPDR;	/*GPIO port pull-up/pull-down register */
	volatile uint32_t IDR;		/*GPIO port input data register */
	volatile uint32_t ODR;		/*GPIO port output data register */
	volatile uint32_t BSRR;		/*GPIO port bit set/reset register */
	volatile uint32_t LCKR;		/*GPIO port configuration lock register*/
	volatile uint32_t AFR[2];	/*alternate function L is 0 H is 1*/
}GPIO_RegDef_t;

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)

typedef struct
{
	volatile uint32_t IMR; 	/*GPIO port mode register */
	volatile uint32_t EMR;	/*GPIO port output type register */
	volatile uint32_t RTSR;	/*GPIO port output speed register */
	volatile uint32_t FTSR;		/*GPIO port pull-up/pull-down register */
	volatile uint32_t SWIER;		/*GPIO port input data register */
	volatile uint32_t PR;		/*GPIO port output data register */
}EXTI_RegDef_t;


#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)

typedef struct{
	volatile uint32_t MEMRMP; 	/*GPIO port mode register */
	volatile uint32_t PMC;	/*GPIO port output type register */
	volatile uint32_t EXTICR[4];	/*GPIO port output speed register */
	uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;		/*GPIO port pull-up/pull-down register */
}SYSCFG_RegDef_t;

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define NVIC_ISE0	((volatile uint32_t*)0xE000E100)
#define NVIC_ISE1	((volatile uint32_t*)0xE000E104)
#define NVIC_ISE2	((volatile uint32_t*)0xE000E108)
#define NVIC_ISE3	((volatile uint32_t*)0xE000E10C)

#define NVIC_ICE0	((volatile uint32_t*)0xE000E180)
#define NVIC_ICE1	((volatile uint32_t*)0xE000E184)
#define NVIC_ICE2	((volatile uint32_t*)0xE000E188)
#define NVIC_ICE3	((volatile uint32_t*)0xE000E18C)

#define NVIC_IPR(n) ((volatile uint32_t*)(0xE000E400 + (n) * 4))




#define ENABLE	1
#define DISABLE	0


#endif /* INC_STM32F411RE_BY_BELT_H_ */
