/*
 * stm32f411re_gpio_driver.h
 *
 *  Created on: May 30, 2024
 *      Author: Admin
 */

#ifndef INC_STM32F411RE_GPIO_DRIVER_H_
#define INC_STM32F411RE_GPIO_DRIVER_H_

#include "stm32f411re_by_belt.h"


#define GPIOAEN     0
#define GPIOBEN     1
#define GPIOCEN     2
#define GPIODEN     3
#define GPIOEEN     4
#define GPIOHEN     7

#define RCC_AHB1PERIPH  0
#define RCC_AHB2PERIPH  1
#define RCC_APB1PERIPH  2
#define RCC_APB2PERIPH  3



#define PIN_0					0
#define PIN_1					1
#define PIN_2					2
#define PIN_3					3
#define PIN_4					4
#define PIN_5					5
#define PIN_6					6
#define PIN_7					7
#define PIN_8					8
#define PIN_9					9
#define PIN_10					10
#define PIN_11					11
#define PIN_12					12
#define PIN_13					13
#define PIN_14					14
#define PIN_15					15

#define INPUT					0x0
#define	OUTPUT					0x1
#define ALTERNATE_FUNCTION		0x2
#define ANALOG					0x3

#define LOW_SPEED				0x0
#define MEDIUM_SPEED			0x1
#define FAST_SPEED				0x2
#define HIGH_SPEED				0x3

#define NO_PUPD					0x0
#define PULL_UP					0x1
#define PULL_DOWN				0x2
#define RESERVED				0x3

#define AF0						0x0
#define AF1						0x1
#define AF2						0x2
#define AF3						0x3
#define AF4						0x4
#define AF5						0x5
#define AF6						0x6
#define AF7						0x7
#define AF8						0x8
#define AF9						0x9
#define AF10					0xA
#define AF11					0xB
#define AF12					0xC
#define AF13					0xD
#define AF14					0xE
#define AF15					0xF

#define RISING_TRIGGER		0
#define FALLING_TRIGGER		1

#define IRQ_WWDG                     0
#define IRQ_EXTI16_PVD               1
#define IRQ_EXTI21_TAMP_STAMP        2
#define IRQ_EXTI22_RTC_WKUP          3
#define IRQ_FLASH                    4
#define IRQ_RCC                      5
#define IRQ_EXTI0                    6
#define IRQ_EXTI1                    7
#define IRQ_EXTI2                    8
#define IRQ_EXTI3                    9
#define IRQ_EXTI4                    10
#define IRQ_DMA1_Stream0             11
#define IRQ_DMA1_Stream1             12
#define IRQ_DMA1_Stream2             13
#define IRQ_DMA1_Stream3             14
#define IRQ_DMA1_Stream4             15
#define IRQ_DMA1_Stream5             16
#define IRQ_DMA1_Stream6             17
#define IRQ_ADC                      18
#define IRQ_EXTI9_5                  23
#define IRQ_TIM1_BRK_TIM9            24
#define IRQ_TIM1_UP_TIM10            25
#define IRQ_TIM1_TRG_COM_TIM11       26
#define IRQ_TIM1_CC                  27
#define IRQ_TIM2                     28
#define IRQ_TIM3                     29
#define IRQ_TIM4                     30
#define IRQ_I2C1_EV                  31
#define IRQ_I2C1_ER                  32
#define IRQ_I2C2_EV                  33
#define IRQ_I2C2_ER                  34
#define IRQ_SPI1                     35
#define IRQ_SPI2                     36
#define IRQ_USART1                   37
#define IRQ_USART2                   38
#define IRQ_EXTI15_10                40
#define IRQ_EXTI17_RTC_Alarm         41
#define IRQ_EXTI18_OTG_FS_WKUP       42
#define IRQ_DMA1_Stream7             47
#define IRQ_SDIO                     49
#define IRQ_TIM5                     50
#define IRQ_SPI3                     51
#define IRQ_DMA2_Stream0             56
#define IRQ_DMA2_Stream1             57
#define IRQ_DMA2_Stream2             58
#define IRQ_DMA2_Stream3             59
#define IRQ_DMA2_Stream4             60
#define IRQ_OTG_FS                   67
#define IRQ_DMA2_Stream5             68
#define IRQ_DMA2_Stream6             69
#define IRQ_DMA2_Stream7             70
#define IRQ_USART6                   71
#define IRQ_I2C3_EV                  72
#define IRQ_I2C3_ER                  73
#define IRQ_FPU                      81
#define IRQ_SPI4                     84
#define IRQ_SPI5                     85


void RCC_peripheral_clock_enable(RCC_RegDef_t* pRCC,uint8_t name_clock,uint8_t	peripheral);

void GPIO_pinMode(GPIO_RegDef_t *pGPIO,uint8_t pin,uint8_t mode,uint8_t speed,uint8_t pupd,uint8_t AFRL);

void GPIO_digitalWrite(GPIO_RegDef_t *pGPIO,uint8_t pin,uint8_t value);

uint8_t GPIO_digitalRead(GPIO_RegDef_t *pGPIO,uint8_t pin);

void GPIO_digitalToggleWrite(GPIO_RegDef_t *pGPIO,uint8_t pin);

void Exti_init(uint8_t port,uint8_t pin,uint8_t mode);

void Nvic_EnableIRQ(uint8_t position,uint8_t ENorDISN);

void Nvic_SetProrityIRQ(uint8_t position,uint32_t priority);

#endif /* INC_STM32F411RE_GPIO_DRIVER_H_ */
